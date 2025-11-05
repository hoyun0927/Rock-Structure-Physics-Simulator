#include "design_shared.h"
#include "optimizer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 외부 유틸리티 함수 선언 */
extern double inputDoubleDefault(const char *prompt, double def);
extern void print_separator(void);
extern double rhsArea(double width, double height, double thickness);
extern double rhsInertia(double width, double height, double thickness);
extern double rhsSectionModulus(double width, double height, double thickness);
extern double cantileverDeflection(double pointLoad, double udlTotal, double length, double elasticModulus, double inertia);
extern double calculateCriticalBucklingStress(double length, double kFactor, double elasticModulus, double yieldStrength, double inertia, double area);

/* Mode2 구현 */

/* Mode2 진입점: 입력 수집 -> 탐색 실행 -> 결과 출력 */
void runDesignOptimizer(void) {
    print_separator();
    printf("Mode 2: 최적 설계 탐색 실행\n");
    print_separator();

    MaterialProperties mat = {0};
    LoadInputs loads = {0};
    ArmComponent verticalArm = {0};
    SafetyLimits limits = {0};
    OptimizerInputs optInputs = {0};
    CombinedOptResult result = {0};

    getOptimizerInputs(&mat, &loads, &verticalArm, &limits, &optInputs);
    runOptimizationLoops(&mat, &loads, &verticalArm, &limits, &optInputs, &result);
    printOptimizerSummary(&result);

    print_separator();
    printf("Mode 2 탐색 완료\n");
    print_separator();
}

void getOptimizerInputs(MaterialProperties *mat, LoadInputs *loads, ArmComponent *verticalArm, SafetyLimits *limits, OptimizerInputs *optInputs) {
    if (!mat || !loads || !verticalArm || !limits || !optInputs) return;

    printf("\n--- Mode 2: 최적 설계 입력 (기본값 엔터) ---\n");

    /* 재료/하중 기본값 재사용 */
    mat->elasticModulus = inputDoubleDefault("강의 탄성계수 E (Pa)", DEFAULT_E_PA);
    mat->yieldStrength  = inputDoubleDefault("강의 항복강도 (Pa)", DEFAULT_YIELD_PA);
    mat->density        = inputDoubleDefault("강의 밀도 (kg/m^3)", DEFAULT_DENSITY);

    loads->boxMass = inputDoubleDefault("박스 질량 (kg)", DEFAULT_BOX_MASS);
    loads->horizontalArmLength = inputDoubleDefault("전방암 길이 L (m)", DEFAULT_ARM_LENGTH);
    loads->armAngle = inputDoubleDefault("전방암 각도 (deg)", DEFAULT_ARM_ANGLE);

    /* 수직 암 기본 (길이, K계수는 고정된 입력 사용) */
    verticalArm->length = inputDoubleDefault("수직암 유효 길이 L (m)", DEFAULT_VERT_LENGTH);
    verticalArm->kFactor = inputDoubleDefault("수직암 좌굴 K 계수", DEFAULT_K_FACTOR);

    limits->safetyFactor = inputDoubleDefault("안전계수 (SF)", DEFAULT_SAFETY_FACTOR);
    limits->allowableStress = mat->yieldStrength / limits->safetyFactor;
    limits->deflectionLimit = loads->horizontalArmLength / DEFAULT_DEFLECTION_DIVISOR; /* 기본 L/360 */
    limits->fatigueDetailCategory = inputDoubleDefault("피로 상세 등급 (MPa)", DEFAULT_FATIGUE_CATEGORY);

    /* 탐색 범위 입력 (m 단위) */
    printf("\n--- 전방 암 탐색 범위 (m) ---\n");
    optInputs->frontWidthRange.min = inputDoubleDefault("전방 폭 최소 (m)", DEFAULT_FRONT_B_MIN);
    optInputs->frontWidthRange.max = inputDoubleDefault("전방 폭 최대 (m)", DEFAULT_FRONT_B_MAX);
    optInputs->frontWidthRange.step = inputDoubleDefault("전방 폭 스텝 (m)", DEFAULT_FRONT_B_STEP);

    optInputs->frontHeightRange.min = inputDoubleDefault("전방 높이 최소 (m)", DEFAULT_FRONT_H_MIN);
    optInputs->frontHeightRange.max = inputDoubleDefault("전방 높이 최대 (m)", DEFAULT_FRONT_H_MAX);
    optInputs->frontHeightRange.step = inputDoubleDefault("전방 높이 스텝 (m)", DEFAULT_FRONT_H_STEP);

    optInputs->frontThicknessRange.min = inputDoubleDefault("전방 두께 최소 (m)", DEFAULT_FRONT_T_MIN);
    optInputs->frontThicknessRange.max = inputDoubleDefault("전방 두께 최대 (m)", DEFAULT_FRONT_T_MAX);
    optInputs->frontThicknessRange.step = inputDoubleDefault("전방 두께 스텝 (m)", DEFAULT_FRONT_T_STEP);

    printf("\n--- 수직 암 탐색 범위 (m) ---\n");
    optInputs->verticalWidthRange.min = inputDoubleDefault("수직 폭 최소 (m)", DEFAULT_VERT_B_MIN);
    optInputs->verticalWidthRange.max = inputDoubleDefault("수직 폭 최대 (m)", DEFAULT_VERT_B_MAX);
    optInputs->verticalWidthRange.step = inputDoubleDefault("수직 폭 스텝 (m)", DEFAULT_VERT_B_STEP);

    optInputs->verticalHeightRange.min = inputDoubleDefault("수직 높이 최소 (m)", DEFAULT_VERT_H_MIN);
    optInputs->verticalHeightRange.max = inputDoubleDefault("수직 높이 최대 (m)", DEFAULT_VERT_H_MAX);
    optInputs->verticalHeightRange.step = inputDoubleDefault("수직 높이 스텝 (m)", DEFAULT_VERT_H_STEP);

    optInputs->verticalThicknessRange.min = inputDoubleDefault("수직 두께 최소 (m)", DEFAULT_VERT_T_MIN);
    optInputs->verticalThicknessRange.max = inputDoubleDefault("수직 두께 최대 (m)", DEFAULT_VERT_T_MAX);
    optInputs->verticalThicknessRange.step = inputDoubleDefault("수직 두께 스텝 (m)", DEFAULT_VERT_T_STEP);
}

void runOptimizationLoops(const MaterialProperties *mat, const LoadInputs *loads, const ArmComponent *verticalArm, const SafetyLimits *limits, const OptimizerInputs *optInputs, CombinedOptResult *outResult) {
    if (!mat || !loads || !verticalArm || !limits || !optInputs || !outResult) return;

    outResult->totalIterations = 0;
    outResult->front = findOptimalFrontArm(mat, loads, limits, optInputs);
    outResult->vertical = findOptimalVerticalArm(loads->boxMass * GRAVITY + (mat->density * rhsArea(optInputs->frontWidthRange.min, optInputs->frontHeightRange.min, optInputs->frontThicknessRange.min) * loads->horizontalArmLength * GRAVITY), mat, verticalArm, limits, optInputs);

    outResult->totalIterations = (double)(outResult->front.iterations + outResult->vertical.iterations);
}

void printOptimizerSummary(const CombinedOptResult *res) {
    if (!res) return;
    print_separator();
    printf("=== Mode 2: 최적 설계 요약 ===\n");

    if (res->front.found) {
        printf("[전방 암]\n");
        printf("  폭 (m): %.4f, 높이 (m): %.4f, 두께 (m): %.4f\n", res->front.bestSection.width, res->front.bestSection.height, res->front.bestSection.thickness);
        printf("  질량 (kg): %.4f, 탐색수: %ld\n", res->front.bestMass, res->front.iterations);
    } else {
        printf("[전방 암] 조건 만족 설계 없음\n");
    }

    if (res->vertical.found) {
        printf("[수직 암]\n");
        printf("  폭 (m): %.4f, 높이 (m): %.4f, 두께 (m): %.4f\n", res->vertical.bestSection.width, res->vertical.bestSection.height, res->vertical.bestSection.thickness);
        printf("  질량 (kg): %.4f, 탐색수: %ld\n", res->vertical.bestMass, res->vertical.iterations);
    } else {
        printf("[수직 암] 조건 만족 설계 없음\n");
    }

    printf("총 탐색 반복수 (대략): %.0f\n", res->totalIterations);
    print_separator();
}

SingleOptResult findOptimalFrontArm(const MaterialProperties *mat, const LoadInputs *loads, const SafetyLimits *limits, const OptimizerInputs *optimizerInputs) {
    SingleOptResult out = {0};
    double bestMass = OPTIMIZER_INIT_MASS;
    long iterations = 0;

    if (!mat || !loads || !limits || !optimizerInputs) return out;

    double L = loads->horizontalArmLength;
    double boxW = loads->boxMass * GRAVITY;

    for (double w = optimizerInputs->frontWidthRange.min; w <= optimizerInputs->frontWidthRange.max; w += optimizerInputs->frontWidthRange.step) {
        for (double h = optimizerInputs->frontHeightRange.min; h <= optimizerInputs->frontHeightRange.max; h += optimizerInputs->frontHeightRange.step) {
            for (double t = optimizerInputs->frontThicknessRange.min; t <= optimizerInputs->frontThicknessRange.max; t += optimizerInputs->frontThicknessRange.step) {
                iterations++;
                double A = rhsArea(w, h, t);
                double I = rhsInertia(w, h, t);
                double W = rhsSectionModulus(w, h, t);
                if (A <= 0.0 || I <= 0.0 || W <= 0.0) continue;

                double armSelfWeight = mat->density * A * L * GRAVITY;
                double M = boxW * L + armSelfWeight * L / 2.0;
                double sigma_Pa = M / W; /* Pa */
                double sigma_MPa = sigma_Pa / PA_TO_MPA;

                double def_m = cantileverDeflection(boxW, armSelfWeight, L, mat->elasticModulus, I);

                /* 조건: 응력 및 처짐 한계 */
                if (sigma_MPa > limits->allowableStress / PA_TO_MPA) continue;
                if (def_m > limits->deflectionLimit) continue;

                double mass = A * L * mat->density;
                if (mass < bestMass) {
                    bestMass = mass;
                    out.bestSection.width = w;
                    out.bestSection.height = h;
                    out.bestSection.thickness = t;
                    out.bestMass = mass;
                    out.found = true;
                }
            }
        }
    }

    out.iterations = iterations;
    return out;
}

SingleOptResult findOptimalVerticalArm(double finalVerticalLoad, const MaterialProperties *mat, const ArmComponent *verticalArmBase, const SafetyLimits *limits, const OptimizerInputs *optimizerInputs) {
    SingleOptResult out = {0};
    double bestMass = OPTIMIZER_INIT_MASS;
    long iterations = 0;

    if (!mat || !verticalArmBase || !limits || !optimizerInputs) return out;

    double L = verticalArmBase->length;
    double k = verticalArmBase->kFactor;

    for (double w = optimizerInputs->verticalWidthRange.min; w <= optimizerInputs->verticalWidthRange.max; w += optimizerInputs->verticalWidthRange.step) {
        for (double h = optimizerInputs->verticalHeightRange.min; h <= optimizerInputs->verticalHeightRange.max; h += optimizerInputs->verticalHeightRange.step) {
            for (double t = optimizerInputs->verticalThicknessRange.min; t <= optimizerInputs->verticalThicknessRange.max; t += optimizerInputs->verticalThicknessRange.step) {
                iterations++;
                double A = rhsArea(w, h, t);
                double I = rhsInertia(w, h, t);
                if (A <= 0.0 || I <= 0.0) continue;

                double sigmaComp_Pa = finalVerticalLoad / A;
                double sigmaComp_MPa = sigmaComp_Pa / PA_TO_MPA;

                double sigmaCr_Pa = calculateCriticalBucklingStress(L, k, mat->elasticModulus, mat->yieldStrength, I, A);
                double sigmaCr_MPa = sigmaCr_Pa / PA_TO_MPA;

                /* 조건: 압축응력 < 임계좌굴응력 및 허용응력 */
                if (sigmaComp_MPa >= sigmaCr_MPa) continue;
                if (sigmaComp_MPa > limits->allowableStress / PA_TO_MPA) continue;

                double mass = A * L * mat->density;
                if (mass < bestMass) {
                    bestMass = mass;
                    out.bestSection.width = w;
                    out.bestSection.height = h;
                    out.bestSection.thickness = t;
                    out.bestMass = mass;
                    out.found = true;
                }
            }
        }
    }

    out.iterations = iterations;
    return out;
}
