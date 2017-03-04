

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
  while (!preds.empty())
  {
    BasicBlock* b = preds.front();
    preds.pop_front();
    if (visited.end() == visited.find(b))
    {
      visited.insert(b);

      auto* ll = info.getLoopFor(b);
      for (auto succ : successors(b))
      {
        size_t succ_count = out.find(
                static_cast<BasicBlock*>(succ))->second;
        out[b] += succ_count;
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
    IRBuilder<> builder(function.getEntryBlock().getFirstNonPHI());
    id = builder.CreateAlloca(idTy, nullptr, EPPID);
    auto* gep = builder.CreateInBoundsGEP(int64Ty, id, zero);
    builder.CreateStore(zero, gep);
  }

  // encode by topographical order to ensure deterministic decoding
  for (auto& bb : function)
  {
    if (!llvm::succ_empty(&bb))
    {
      // get the successor of minimum number of paths
      auto it = succ_begin(&bb);
      BasicBlock* minBlock = *it;
      ++it;
      for (auto et = succ_end(&bb);
          it != et; ++it)
      {
        BasicBlock* s = *it;
        if (numPaths[s] < numPaths[minBlock])
        {
          minBlock = s;
        }
      }

      if (size_t countdiff = numPaths[&bb] - numPaths[minBlock])
      {
        // edge to minBlock = countdiff
        edges[{&bb, minBlock}] = countdiff;

        IRBuilder<> builder(minBlock->getFirstNonPHI());
        auto* incr = ConstantInt::get(int64Ty, countdiff, false);
        auto* gep = builder.CreateInBoundsGEP(int64Ty, id, zero);
        auto* temp = builder.CreateAlignedLoad(gep, 8);
        auto* add = builder.CreateNSWAdd(temp, incr);
        builder.CreateStore(add, gep);
      }
    }
  }
}
