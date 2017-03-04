

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Transforms/Utils/ModuleUtils.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

#include "PathProfilingPass.h"


using namespace llvm;
using namespace pathprofiling;


namespace pathprofiling {
char PathProfilingPass::ID = 0;
}


bool
PathProfilingPass::runOnModule(llvm::Module &module) {
  // functionIDs are implicitly encoded by its number of paths
  // revealed in topographic order
  auto* int64Ty = Type::getInt64Ty(module.getContext());
  size_t nentries = 0;
  for (auto& f : module)
  {
    if (!f.isDeclaration())
    {
      nentries += instrument(module, f, nentries);
    }
  }
  ++nentries;
  std::vector<Constant*> paths;
  for (size_t i = 0; i < nentries; i++)
  {
    paths.push_back(llvm::ConstantInt::get(int64Ty, 0, false));
  }
  ArrayType* arrTy = ArrayType::get(int64Ty, nentries);
  auto* pathcount = ConstantArray::get(arrTy, paths);
  new GlobalVariable(module, arrTy, false,
    GlobalValue::ExternalLinkage,
    pathcount, "PaThPrOfIlInG_paths");


  auto* npathcount = ConstantInt::get(int64Ty, nentries, false);
  new GlobalVariable(module, int64Ty, false,
	GlobalValue::ExternalLinkage,
	npathcount, "PaThPrOfIlInG_nPaths");

  auto printfunc = module.getOrInsertFunction("PaThPrOfIlInG_print",
    Type::getVoidTy(module.getContext()), nullptr);
  appendToGlobalDtors(module, cast<llvm::Function>(printfunc), 0);

  return true;
}


// return the number of paths in function
uint64_t
PathProfilingPass::instrument(llvm::Module& module,
                              llvm::Function& function,
                              uint64_t functionID) {
  llvm::DenseMap<llvm::BasicBlock*, uint64_t> blockCount;
  LoopInfo& linfo = getAnalysis<LoopInfoWrapperPass>(function).getLoopInfo();
  if (countPaths(blockCount, linfo, function))
  {
    return 0;
  }

  llvm::Value* id = nullptr;
  auto* voidTy = Type::getVoidTy(function.getContext());
  auto* int64Ty = Type::getInt64Ty(function.getContext());
  llvm::ConstantInt* funcId = llvm::ConstantInt::get(int64Ty, functionID, false);
  BasicBlock& entry = function.getEntryBlock();

  for (auto& inst : entry)
  {
    if (EPPID == inst.getName())
    {
      id = &inst;
    }
  }
  assert(id);

  // goal: instrument at select instructions to minimize the execution of instrumented function
  // encoding was responsible for instrumenting EPPID
  // path profiling should now increment global table by EPPID
  // at all termination blocks

  // step 1: locate termination blocks
  uint64_t nPaths = blockCount[&entry];

  std::vector<llvm::BasicBlock*> terms;
  for (auto& bb : function)
  {
    if (blockCount[&bb] == 1 && llvm::succ_empty(&bb))
    {
      terms.push_back(&bb);
    }
  }

  auto countfunc = module.getOrInsertFunction("PaThPrOfIlInG_count",
    FunctionType::get(voidTy, {int64Ty, int64Ty}, false));

  // step 2: instrument
  for (llvm::BasicBlock* bb : terms)
  {
    IRBuilder<> builder(bb->getFirstNonPHI());
    auto* gep = builder.CreateInBoundsGEP(int64Ty, id, ConstantInt::get(int64Ty, 0));
    auto* temp = builder.CreateAlignedLoad(gep, 8);
    builder.CreateCall(countfunc, {funcId, temp});
  }

  return nPaths;
}

