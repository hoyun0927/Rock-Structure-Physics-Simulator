#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "design_shared.h"

/* Mode2 공개 API */
void getOptimizerInputs(MaterialProperties *mat, LoadInputs *loads, ArmComponent *verticalArm, SafetyLimits *limits, OptimizerInputs *optInputs);
void runOptimizationLoops(const MaterialProperties *mat, const LoadInputs *loads, const ArmComponent *verticalArm, const SafetyLimits *limits, const OptimizerInputs *optInputs, CombinedOptResult *outResult);
void printOptimizerSummary(const CombinedOptResult *res);

SingleOptResult findOptimalFrontArm(const MaterialProperties *mat, const LoadInputs *loads, const SafetyLimits *limits, const OptimizerInputs *optimizerInputs);
SingleOptResult findOptimalVerticalArm(double finalVerticalLoad, const MaterialProperties *mat, const ArmComponent *verticalArmBase, const SafetyLimits *limits, const OptimizerInputs *optimizerInputs);

#endif /* OPTIMIZER_H */
