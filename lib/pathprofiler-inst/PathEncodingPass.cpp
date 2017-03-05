

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Analysis/LoopInfo.h"

#include "PathEncodingPass.h"

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
// todo: remove
#include <iostream>


using namespace llvm;
using namespace pathprofiling;


namespace pathprofiling {
  char PathEncodingPass::ID = 0;
}


bool pathprofiling::countPaths (llvm::DenseMap<llvm::BasicBlock*, uint64_t>& out, LoopInfo& info, llvm::Function& function)
{
  // breadth wise reverse search
  std::list<BasicBlock*> preds;
  std::unordered_set<BasicBlock*> visited;
  // store every node visited in visited set
  for (auto& bb : function)
  {
    // all blocks without successors are potential exit nodes
    size_t init = 0;
    if (llvm::succ_empty(&bb))
    {
      init = 1;
      // add predecessor to pred 'queue'
      preds.insert(preds.end(),
                   llvm::pred_begin(&bb), llvm::pred_end(&bb));
      visited.emplace(&bb);
    }
    out[&bb] = init;
  }

  bool backedged = false;
  std::vector<BasicBlock*> entrances;
  while (false == backedged && !preds.empty())
  {
    BasicBlock* b = preds.front();
    preds.pop_front();
    if (visited.end() == visited.find(b))
    {
      auto* ll = info.getLoopFor(b);

	  // only sum up successors if they've all been visited
	  if (std::all_of(succ_begin(b), succ_end(b),
	  [&visited](BasicBlock* s)
	  {
	    return visited.end() != visited.find(s);
	  }))
	  {
        for (auto succ : successors(b))
        {
          size_t succ_count = out.find(
            static_cast<BasicBlock*>(succ))->second;
          out[b] += succ_count;
        }
        visited.insert(b);
	  }
	  else
	  {
	  	preds.push_back(b);
	  }
      for (auto pred : predecessors(b))
      {
        BasicBlock* p = static_cast<BasicBlock*>(pred);
        auto* pll = info.getLoopFor(p);
        if (ll && pll == ll)
        {
          // ignore all backward facing edges
          backedged = true;
        }
        else
        {
          preds.push_back(p);
        }
      }
    }
  }
//for (auto& bb : function) { std::cout << out[&bb] << " "; }
//std::cout << "\n";
  return backedged;
}


bool
PathEncodingPass::runOnModule(Module& module) {
  for (auto& f : module) {
    if (!f.isDeclaration()) {
      encode(f);
    }
  }

  return true;
}


void
PathEncodingPass::encode(llvm::Function& function) {
  // goal: computes the compact number for a CFG... (addresses efficiency concerns).
  // instrument a local variable ID and update for specific branches

  // step 1: count the number of paths by stacking the blocks and retrace
  LoopInfo& linfo = getAnalysis<LoopInfoWrapperPass>(function).getLoopInfo();
  if (countPaths(numPaths, linfo, function))
  {
    return;
  }

  // step 2: partition encoding space locally then instrument
  auto *int64Ty = Type::getInt64Ty(function.getContext());
  ConstantInt* zero = ConstantInt::get(int64Ty, 0);
  ArrayType* idTy = ArrayType::get(int64Ty, 1);
  AllocaInst* id;
  {
  	Instruction* first = function.getEntryBlock().getFirstNonPHI();
    IRBuilder<> builder(first);
    id = builder.CreateAlloca(idTy, nullptr, EPPID);
    auto* gep = builder.CreateInBoundsGEP(int64Ty, id, zero);
    builder.CreateStore(zero, gep);
  }

  // order basic blocks by their path count DESC
  // this accounts for multiple successors (>2) like switch, or try catches
  std::priority_queue<BasicBlock*,
     std::vector<BasicBlock*>,
     std::function<bool(BasicBlock*, BasicBlock*)> > countorder(
     [this](BasicBlock* lhs, BasicBlock* rhs) -> bool
	 {
	   return numPaths[lhs] > numPaths[rhs];
	 });

  // encode by topographical order to ensure deterministic decoding
  for (auto& bb : function)
  {
    if (!llvm::succ_empty(&bb))
    {
      // get the successor of minimum number of paths
      for (auto succ : successors(&bb))
      {
        countorder.push(succ);
      }

      uint64_t nextdiff = numPaths[countorder.top()];
	  // traverse countorder
	  if (false == countorder.empty())
	  {
	  	countorder.pop(); // never instrument the successor of max path count
	  }
	  while (!countorder.empty())
	  {
	    BasicBlock* minBlock = countorder.top();
	    countorder.pop();

	    edges[{&bb, minBlock}] = nextdiff;
	    IRBuilder<> builder(minBlock->getFirstNonPHI());
	    auto* incr = ConstantInt::get(int64Ty, nextdiff, false);
	    auto* gep = builder.CreateInBoundsGEP(int64Ty, id, zero);
	    auto* temp = builder.CreateAlignedLoad(gep, 8);
	    auto* add = builder.CreateNSWAdd(temp, incr);
	    builder.CreateStore(add, gep);

        nextdiff += numPaths[minBlock];
      }
    }
  }
}
