

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"

#include <unordered_map>
#include <fstream>
#include <llvm/IR/Constants.h>
#include <queue>

#include "ProfileDecodingPass.h"


using namespace llvm;
using namespace pathprofiling;


namespace pathprofiling {
char ProfileDecodingPass::ID = 0;
}

using DUAL = std::pair<uint64_t, uint64_t>;

// Given a sequence of basic blocks composing a path, this function prints
// out the filename and line numbers associated with that path in CSV format.
void
printPath(llvm::raw_ostream& out, std::vector<llvm::BasicBlock*>& blocks) {
  unsigned line = 0;
  llvm::StringRef file;
  for (auto* bb : blocks) {
    for (auto& instruction : *bb) {
      llvm::DILocation* loc = instruction.getDebugLoc();
      if (loc && (loc->getLine() != line || loc->getFilename() != file)) {
        line = loc->getLine();
        file = loc->getFilename();
        out << ", " << file.str() << ", " << line;
      }
    }
  }
}


bool
ProfileDecodingPass::runOnModule(Module &module) {
  // expect infile to have format <count>\n ordered by encoding id
  // so encoding id = 0 is the first line, id = 1 is second line, etc.
  // encoding id captures block topological order (for all back-edge-less functions)

  // step 1: map encoding id to {function, pathid}
  std::vector<std::pair<Function*, uint64_t> > fencoding;
  for (auto& f : module)
  {
    if (!f.isDeclaration())
    {
      llvm::DenseMap<llvm::BasicBlock*, uint64_t> blockCount;
      LoopInfo& linfo = getAnalysis<LoopInfoWrapperPass>(f).getLoopInfo();
      if (false == countPaths(blockCount, linfo, f))
      {
        BasicBlock& entry = f.getEntryBlock();
        uint64_t nPaths = blockCount[&entry];
        for (size_t i = 0; i < nPaths; ++i)
        {
          fencoding.push_back({&f, i});
        }
	  }
    }
  }

  // step 2: identify top n paths
  // topN formatted as <encoding id>, <count>
  std::vector<DUAL> topN;
  std::ifstream in(infilename);
  if (in.is_open())
  {
    size_t encoding_id = 0;
    while (in.good())
    {
      std::string l;
      in >> l;
      if (!l.empty())
      {
        uint64_t count = atoi(l.data());
        topN.push_back({encoding_id, count});
        encoding_id++;
	  }
    }
  }
  in.close();
  // sort topN
  std::sort(topN.begin(), topN.end(),
  [](const DUAL& lhs, const DUAL& rhs)
  {
    // DESCENDING COUNT ORDER
    return lhs.second > rhs.second;
  });

  // step 3: decode top n paths in order then output
  std::error_code EC;
  raw_fd_ostream out(outfilename, EC, sys::fs::F_None);
  uint64_t N = std::min(topN.size(), numberToReturn);
  for (uint64_t i = 0; i < N; ++i)
  {
    DUAL& d = topN[i];
    if (d.second)
    {
	  std::pair<Function*, uint64_t> params = fencoding[d.first];
	  std::vector<llvm::BasicBlock*> sequence = decode(params.first, params.second);

  	  out << d.second << ", " << params.first->getName();
	  printPath(out, sequence);
	  out << "\n";
    }
  }

  return true;
}


std::vector<llvm::BasicBlock*>
ProfileDecodingPass::decode(llvm::Function* function, uint64_t pathID) {
  std::vector<llvm::BasicBlock*> sequence;

  llvm::DenseMap<llvm::BasicBlock*, uint64_t> CFG;
  LoopInfo& linfo = getAnalysis<LoopInfoWrapperPass>(*function).getLoopInfo();
  assert(!countPaths(CFG, linfo, *function));

  // order exactly like encoding
  std::priority_queue<BasicBlock*,
    std::vector<BasicBlock*>,
    std::function<bool(BasicBlock*, BasicBlock*)> > countorder(
    [&CFG](BasicBlock* lhs, BasicBlock* rhs) -> bool
    {
        return CFG[lhs] < CFG[rhs];
    });

  // propagate through CFG, decrement pathID to 0
  BasicBlock* next = &function->getEntryBlock();
  while (next)
  {
    sequence.push_back(next);
    for (auto succ : successors(next))
    {
      countorder.push(succ);
    }
    next = nullptr;
    // determine the next
    while (nullptr == next && !countorder.empty())
    {
      BasicBlock* block = countorder.top();
      countorder.pop();
      uint64_t npaths = CFG[block];
      if (pathID < npaths) // block is next
      {
        next = block;
      }
      else // check next block in countorder
      {
        pathID -= npaths;
      }
    }
  }

  return sequence;
}

