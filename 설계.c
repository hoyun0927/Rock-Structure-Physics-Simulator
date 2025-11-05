
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>

/* ====================================================================== */
/* === 1. 상수 정의 (Define) ===                                            */
/* ====================================================================== */

/* 물리 상수 */
#define GRAVITY 9.81
#define PI 3.14159265358979323846

/* 단위/변환 */
#define PA_TO_MPA 1e6
#define M_TO_MM 1000.0
#define N_TO_KN 1000.0
#define DEG_TO_RAD (PI/180.0)

/* 계산 상수 (설계 철학에 맞춰 필요시 조정) */
#define RHS_THICKNESS_FACTOR 2.0
#define RHS_I_DENOMINATOR 12.0
#define RHS_W_NUMERATOR 2.0
#define DEFLECT_POINT_DENOM 3.0
#define DEFLECT_UDL_DENOM 8.0
#define VON_MISES_SHEAR_FACTOR 3.0
#define BUCKLING_PI_FACTOR 2.0
#define BUCKLING_JOHNSON_DENOM 4.0
#define PAD_AREA_FACTOR 2.0

/* 피로 상수 */
#define FATIGUE_SLOPE 3.0
#define FATIGUE_REF_CYCLES 2e6
#define FATIGUE_INFINITE_CYCLES 1.0e15

/* 최적화 초기값 */
#define OPTIMIZER_INIT_MASS 1e99

/* Defaults (Mode1) */
#define DEFAULT_E_PA 210e9
#define DEFAULT_YIELD_PA 355e6
#define DEFAULT_DENSITY 7850.0
#define DEFAULT_BOX_MASS 100.0
#define DEFAULT_ARM_LENGTH 1.0
#define DEFAULT_BOX_OFFSET 0.0
#define DEFAULT_ARM_ANGLE 0.0
#define DEFAULT_FRONT_B 0.080
#define DEFAULT_FRONT_H 0.040
#define DEFAULT_FRONT_T 0.005
#define DEFAULT_VERT_B 0.100
#define DEFAULT_VERT_H 0.100
#define DEFAULT_VERT_T 0.006
#define DEFAULT_VERT_LENGTH 2.0
#define DEFAULT_K_FACTOR 1.0
#define DEFAULT_SAFETY_FACTOR 2.0
#define DEFAULT_OUTRIGGER_SPACING 0.9
#define DEFAULT_PAD_AREA 0.04
#define DEFAULT_USE_COUNTERWEIGHT 1
#define DEFAULT_COUNTER_DIST 0.4
#define DEFAULT_BOLT_LEVER_ARM 0.45
#define DEFAULT_TENSION_BOLTS 2
#define DEFAULT_FATIGUE_CATEGORY 80.0
#define DEFAULT_DEFLECTION_DIVISOR 360.0

/* Defaults (Mode2) */
#define DEFAULT_FRONT_B_MIN 0.050
#define DEFAULT_FRONT_B_MAX 0.150
#define DEFAULT_FRONT_B_STEP 0.010
#define DEFAULT_FRONT_H_MIN 0.050
#define DEFAULT_FRONT_H_MAX 0.150
#define DEFAULT_FRONT_H_STEP 0.010
#define DEFAULT_FRONT_T_MIN 0.003
#define DEFAULT_FRONT_T_MAX 0.010
#define DEFAULT_FRONT_T_STEP 0.001
#define DEFAULT_VERT_B_MIN 0.080
#define DEFAULT_VERT_B_MAX 0.200
#define DEFAULT_VERT_B_STEP 0.010
#define DEFAULT_VERT_H_MIN 0.080
#define DEFAULT_VERT_H_MAX 0.200
#define DEFAULT_VERT_H_STEP 0.010
#define DEFAULT_VERT_T_MIN 0.004
#define DEFAULT_VERT_T_MAX 0.012
#define DEFAULT_VERT_T_STEP 0.001

/* 안전한 최소/최대 반복 한계 (무한루프 방지) */
#define MAX_OPT_ITER 20000000L

/* ====================================================================== */
/* === 공통 타입/상수 공유 헤더 포함 ===                                   */
/* ====================================================================== */
#include "design_shared.h"

/* ====================================================================== */
/* === 3. 함수 프로토타입 (Function declarations) ===                     */
/* ====================================================================== */

/* 메인 오케스트레이터 */
void runDesignCheck(void);
void runDesignOptimizer(void);

/* Mode1 */
void getDesignCheckInputs(MaterialProperties *mat, LoadInputs *loads, ArmComponent *frontArm, ArmComponent *verticalArm, SafetyLimits *limits, DesignCheckInputs *checkInputs);
void performDesignAnalysis(const MaterialProperties *mat, const LoadInputs *loads, const ArmComponent *frontArm, const ArmComponent *verticalArm, const SafetyLimits *limits, const DesignCheckInputs *checkInputs, DesignCheckResults *results);
void printDesignCheckResults(const SafetyLimits *limits, const LoadInputs *loads, const DesignCheckInputs *checkInputs, const DesignCheckResults *results);

/* Mode2: optimizer.h로 이동됨 */
#include "optimizer.h"

/* 공통 유틸리티 */
double inputDoubleDefault(const char *prompt, double def);
int    inputIntDefault(const char *prompt, int def);
void   flushStdin(void);

/* 단면/물리 계산 함수 */
double rhsInertia(double width, double height, double thickness);
double rhsSectionModulus(double width, double height, double thickness);
double rhsArea(double width, double height, double thickness);
double cantileverDeflection(double pointLoad, double udlTotal, double length, double elasticModulus, double inertia);
double calculateCounterweight(double moment, double distance, double gravity);
double calculateBoltTension(double moment, double leverArm, int numBolts);
double calculateMaxShearStress(double shearForce, double height, double thickness);
double calculateCriticalBucklingStress(double length, double kFactor, double elasticModulus, double yieldStrength, double inertia, double area);
double calculateFatigueLife(double maxStress_MPa, double detailCategory_MPa);
double calculatePadPressure(double moment, double spacing, double padArea);

/* 기타 유틸리티 */
double safe_pow(double base, double exp);
bool   approx_equal(double a, double b, double tol);

/* ====================================================================== */
/* === 4. 공통 유틸리티 함수 구현 ===                                      */
/* ====================================================================== */

void flushStdin(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}
}

/* 입력 처리: 기본값 제공 (개행만 입력하면 기본값 사용) */
double inputDoubleDefault(const char *prompt, double def) {
    char buf[256];
    printf("%s [기본값 %.6g]: ", prompt, def);
    if (!fgets(buf, sizeof(buf), stdin)) {
        return def;
    }
    /* trim newline */
    size_t n = strcspn(buf, "\n");
    if (n == 0) return def; /* 엔터만 쳤을 때 */
    buf[n] = '\0';
    double val;
    if (sscanf(buf, "%lf", &val) == 1) return val;
    return def;
}

int inputIntDefault(const char *prompt, int def) {
    char buf[256];
    printf("%s [기본값 %d]: ", prompt, def);
    if (!fgets(buf, sizeof(buf), stdin)) {
        return def;
    }
    size_t n = strcspn(buf, "\n");
    if (n == 0) return def;
    buf[n] = '\0';
    int v;
    if (sscanf(buf, "%d", &v) == 1) return v;
    return def;
}

/* 작은 보조 함수: 약간의 안전성 부여 */
double safe_pow(double base, double exp) {
    /* 제한된 범위에서만 pow 호출 */
    if (base <= 0.0) {
        if (exp > 0.0) return 0.0;
        /* 음수/0에 대한 비정상적 exp는 NaN을 발생시키므로 guard */
        return 0.0;
    }
    return pow(base, exp);
}

bool approx_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

/* ====================================================================== */
/* === 5. 단면(직사각 중공) 및 기초 물리 계산 구현 ===                      */
/* ====================================================================== */

/*
 * rhsArea:
 *  outer area - inner area (inner dims = outer - 2*t ?)
 *  여기서는 RHS_THICKNESS_FACTOR를 사용 (사용자정의 계수)
 */
double rhsArea(double width, double height, double thickness) {
    double tf = RHS_THICKNESS_FACTOR;
    if (thickness <= 0.0) return 0.0;
    if (width <= tf * thickness || height <= tf * thickness) return 0.0;
    double innerW = width - tf * thickness;
    double innerH = height - tf * thickness;
    if (innerW <= 0.0 || innerH <= 0.0) return 0.0;
    return (width * height) - (innerW * innerH);
}

/*
 * rhsInertia:
 *  I = (B*H^3 - b*h^3) / 12
 */
double rhsInertia(double width, double height, double thickness) {
    double tf = RHS_THICKNESS_FACTOR;
    if (thickness <= 0.0) return 0.0;
    if (width <= tf * thickness || height <= tf * thickness) return 0.0;
    double innerW = width - tf * thickness;
    double innerH = height - tf * thickness;
    if (innerW <= 0.0 || innerH <= 0.0) return 0.0;
    double outerI = width * safe_pow(height, 3) / RHS_I_DENOMINATOR;
    double innerI = innerW * safe_pow(innerH, 3) / RHS_I_DENOMINATOR;
    double I = outerI - innerI;
    if (I < 0.0) return 0.0;
    return I;
}

/*
 * rhsSectionModulus: W = 2 * I / h (approx for symmetric)
 */
double rhsSectionModulus(double width, double height, double thickness) {
    double I = rhsInertia(width, height, thickness);
    if (I <= 0.0) return 0.0;
    return RHS_W_NUMERATOR * I / height;
}

/* Cantilever deflection (point load at tip + UDL equivalent total)
 * Use simplified closed-form approximations:
 *  - point load P at tip: delta_P = P * L^3 / (3 E I)
 *  - UDL w_total (N) over length: delta_w = w * L^3 / (8 E I)  (w total = q*L)
 */
double cantileverDeflection(double pointLoad, double udlTotal, double length, double elasticModulus, double inertia) {
    if (elasticModulus <= 0.0 || inertia <= 0.0 || length <= 0.0) return 0.0;
    double dp = 0.0, du = 0.0;
    dp = (pointLoad * safe_pow(length, 3)) / (DEFLECT_POINT_DENOM * elasticModulus * inertia);
    du = (udlTotal * safe_pow(length, 3)) / (DEFLECT_UDL_DENOM * elasticModulus * inertia);
    return dp + du;
}

/* calculateCounterweight: moment (N*m) / (distance (m) * g) -> kg */
double calculateCounterweight(double moment, double distance, double gravity) {
    if (distance <= 0.0 || gravity <= 0.0) return 0.0;
    double mass = moment / (distance * gravity);
    if (mass < 0.0) return 0.0;
    return mass;
}

/* calculateBoltTension: moment / leverArm / n_bolts -> N per bolt */
double calculateBoltTension(double moment, double leverArm, int numBolts) {
    if (leverArm <= 0.0 || numBolts <= 0) return 0.0;
    double total = moment / leverArm;
    return total / (double)numBolts;
}

/* shear stress approx: V / A_web
 * web area approximated by vertical web area: t * (h - 2*t)
 * but using RHS_THICKNESS_FACTOR to account for two webs, etc.
 */
double calculateMaxShearStress(double shearForce, double height, double thickness) {
    double tf = RHS_THICKNESS_FACTOR;
    double webArea = tf * thickness * (height - tf * thickness);
    if (webArea <= 0.0) return 0.0;
    return shearForce / webArea;
}

/*
 * calculateCriticalBucklingStress:
 *  - compute radius of gyration r = sqrt(I/A)
 *  - slenderness lambda = k*L / r
 *  - compare with columnConstant = sqrt(2*pi^2*E / sigma_y) (if using BUCKLING_PI_FACTOR)
 *  - Euler: sigma_cr = pi^2 * E / lambda^2
 *  - Johnson: sigma_cr = sigma_y * (1 - (sigma_y * lambda^2) / (4*pi^2*E))
 * returns sigma_cr in Pa
 */
double calculateCriticalBucklingStress(double length, double kFactor, double elasticModulus, double yieldStrength, double inertia, double area) {
    if (length <= 0.0 || kFactor <= 0.0 || elasticModulus <= 0.0 || yieldStrength <= 0.0 || inertia <= 0.0 || area <= 0.0) {
        return 0.0;
    }
    double radiusGyr = sqrt(inertia / area);
    if (radiusGyr <= 0.0) return 0.0;
    double slenderness = kFactor * length / radiusGyr;
    double columnConstant = sqrt(BUCKLING_PI_FACTOR * PI * PI * elasticModulus / yieldStrength);
    if (slenderness >= columnConstant) {
        /* Euler */
        return (PI * PI * elasticModulus) / (slenderness * slenderness);
    } else {
        /* Johnson */
        double denom = BUCKLING_JOHNSON_DENOM * PI * PI * elasticModulus;
        double term = (yieldStrength * safe_pow(slenderness, 2)) / denom;
        double sigma = yieldStrength * (1.0 - term);
        if (sigma < 0.0) sigma = 0.0;
        return sigma;
    }
}

/*
 * calculateFatigueLife:
 *  - uses simplistic S-N power law: N = C / (deltaSigma)^m
 *  - C = (sigma_ref)^m * N_ref
 *  - returns cycles. if deltaSigma < detailCategory => infinite approximation
 *  - inputs: maxStress in MPa, detailCategory in MPa
 */
double calculateFatigueLife(double maxStress_MPa, double detailCategory_MPa) {
    if (detailCategory_MPa <= 0.0) return 0.0;
    double m = FATIGUE_SLOPE;
    double Nref = FATIGUE_REF_CYCLES;
    double C = safe_pow(detailCategory_MPa, m) * Nref;
    double deltaSigma = maxStress_MPa;
    if (deltaSigma <= 0.0) return -1.0;
    if (deltaSigma < detailCategory_MPa) return FATIGUE_INFINITE_CYCLES;
    double N = C / safe_pow(deltaSigma, m);
    return N;
}

/*
 * Pad pressure:
 *  - assume overturning moment M (N*m) resisted by two front pads at spacing s (m)
 *  - front reaction force sum = M / s  (N)
 *  - total contact area = 2 * padArea (m^2)
 *  - return pressure in Pa
 */
double calculatePadPressure(double moment, double spacing, double padArea) {
    if (spacing <= 0.0 || padArea <= 0.0) return 0.0;
    double totalFrontForce = moment / spacing;
    double totalContactArea = PAD_AREA_FACTOR * padArea;
    return totalFrontForce / totalContactArea;
}

/* ====================================================================== */
/* === 6. Sanity helpers / printing helpers (작은 유틸) ===                   */
/* ====================================================================== */

void print_separator(void) {
    printf("------------------------------------------------------------\n");
}

/* 안전값 체크: double이 NaN 또는 inf인지 검사 */
static bool is_finite_double(double x) {
    return (isfinite(x) != 0);
}

/* 안전한 출력용: double -> string with fallback */
static const char *dstr(double x, char *buf, size_t bufsz) {
    if (!is_finite_double(x)) {
        snprintf(buf, bufsz, "NaN");
    } else {
        snprintf(buf, bufsz, "%.6g", x);
    }
    return buf;
}

/* 간단한 출력 유틸: 가로 구분선 n글자 출력 */
void printLine(int n) {
    for (int i = 0; i < n; ++i) putchar('-');
    putchar('\n');
}

void printSectionHeader(const char *title) {
    printLine(60);
    printf("%s\n", title);
    printLine(60);
}

/* Mode2 진입점: optimizer.h로 이동됨 */
extern void runDesignOptimizer(void);

/* ====================================================================== */
/* === Part 2/4: Mode 1 (단일 설계 검토) 구현 ===                         */
/* ====================================================================== */

/**
 * @brief 모드 1 진입점: 사용자 입력 -> 분석 -> 출력
 */
void runDesignCheck(void) {
    print_separator();
    printf("Mode 1: 단일 설계 검토 실행\n");
    print_separator();

    MaterialProperties mat = {0};
    LoadInputs loads = {0};
    ArmComponent frontArm = {0};
    ArmComponent verticalArm = {0};
    SafetyLimits limits = {0};
    DesignCheckInputs checkInputs = {0};
    DesignCheckResults results = {0};

    /* 입력 */
    getDesignCheckInputs(&mat, &loads, &frontArm, &verticalArm, &limits, &checkInputs);

    /* 분석 */
    performDesignAnalysis(&mat, &loads, &frontArm, &verticalArm, &limits, &checkInputs, &results);

    /* 출력 */
    printDesignCheckResults(&limits, &loads, &checkInputs, &results);

    print_separator();
    printf("Mode 1 분석 완료\n");
    print_separator();
}

/**
 * @brief Mode1: 사용자로부터 모든 입력을 안전하게 받음.
 *
 * 주의: 기본값을 제공하므로 엔터만 눌러도 진행됨.
 */
void getDesignCheckInputs(MaterialProperties *mat, LoadInputs *loads, ArmComponent *frontArm, ArmComponent *verticalArm, SafetyLimits *limits, DesignCheckInputs *checkInputs) {
    if (!mat || !loads || !frontArm || !verticalArm || !limits || !checkInputs) return;

    /* 재료 물성 */
    mat->elasticModulus = inputDoubleDefault("강의 탄성계수 E (Pa)", DEFAULT_E_PA);
    mat->yieldStrength  = inputDoubleDefault("강의 항복강도 (Pa)", DEFAULT_YIELD_PA);
    mat->density        = inputDoubleDefault("강의 밀도 (kg/m^3)", DEFAULT_DENSITY);

    /* 하중 및 기본 치수 */
    loads->boxMass = inputDoubleDefault("\n박스 질량 (kg)", DEFAULT_BOX_MASS);
    loads->horizontalArmLength = inputDoubleDefault("전방암 수평 길이 L (m) (박스 중심까지)", DEFAULT_ARM_LENGTH);
    checkInputs->boxOffset = inputDoubleDefault("박스 중심 - 암 끝단 수평 offset (m) (tip=0)", DEFAULT_BOX_OFFSET);
    loads->armAngle = inputDoubleDefault("전방암 각도 (degrees from horizontal, 0=수평)", DEFAULT_ARM_ANGLE);

    /* 전방 암 단면 (RHS) */
    printf("\n--- 전방 암 (RHS) 단면 입력 ---\n");
    frontArm->section.width = inputDoubleDefault("RHS 전방암 외부 폭 b (m)", DEFAULT_FRONT_B);
    frontArm->section.height = inputDoubleDefault("RHS 전방암 외부 높이 h (m)", DEFAULT_FRONT_H);
    frontArm->section.thickness = inputDoubleDefault("RHS 전방암 두께 t (m)", DEFAULT_FRONT_T);
    frontArm->length = loads->horizontalArmLength + checkInputs->boxOffset;

    /* 수직 암 단면 (RHS) */
    printf("\n--- 수직 암 (RHS) 단면 입력 ---\n");
    verticalArm->section.width = inputDoubleDefault("RHS 수직암 외부 폭 b (m)", DEFAULT_VERT_B);
    verticalArm->section.height = inputDoubleDefault("RHS 수직암 외부 높이 h (m)", DEFAULT_VERT_H);
    verticalArm->section.thickness = inputDoubleDefault("RHS 수직암 두께 t (m)", DEFAULT_VERT_T);
    verticalArm->length = inputDoubleDefault("수직암 유효 길이 L (m)", DEFAULT_VERT_LENGTH);
    verticalArm->kFactor = inputDoubleDefault("수직암 좌굴 K 계수 (Pin-Pin=1.0, Fixed-Free~2.0)", DEFAULT_K_FACTOR);

    /* 안전계수 및 허용치 */
    limits->safetyFactor = inputDoubleDefault("\n안전계수 (SF)", DEFAULT_SAFETY_FACTOR);
    limits->allowableStress = mat->yieldStrength / limits->safetyFactor;
    printf("=> 허용응력 = %.3f MPa (%.0f MPa / SF %.2f)\n", limits->allowableStress / PA_TO_MPA, mat->yieldStrength / PA_TO_MPA, limits->safetyFactor);

    /* 처짐 제한 (L/X) 기본값 L/360 */
    double defDiv = inputDoubleDefault("\n처짐 제한 L/X 입력 (예: 360이면 L/360)", DEFAULT_DEFLECTION_DIVISOR);
    limits->deflectionLimit = frontArm->length / defDiv;
    printf("=> 허용 처짐 = %.3f mm (L/%.0f)\n", limits->deflectionLimit * M_TO_MM, defDiv);

    /* 아웃리거 / 패드 */
    checkInputs->outriggerSpacing = inputDoubleDefault("\n베이스 전방/후방 지지대(아웃리거) 간격 (m)", DEFAULT_OUTRIGGER_SPACING);
    checkInputs->outriggerPadArea = inputDoubleDefault("아웃리거 패드 1개당 접촉 면적 (m^2)", DEFAULT_PAD_AREA);

    /* 카운터웨이트 / 볼트 */
    checkInputs->useCounterweight = inputIntDefault("\n카운터웨이트 사용 (1=사용, 0=미사용)", DEFAULT_USE_COUNTERWEIGHT);
    checkInputs->counterweightDistance = DEFAULT_COUNTER_DIST;
    if (checkInputs->useCounterweight) {
        checkInputs->counterweightDistance = inputDoubleDefault("카운터웨이트 레버암 d_back (m)", DEFAULT_COUNTER_DIST);
    }
    checkInputs->boltLeverArm = inputDoubleDefault("베이스 볼트 x-방향 레버암 (m)", DEFAULT_BOLT_LEVER_ARM);
    checkInputs->numTensionBolts = inputIntDefault("인장력을 받는 볼트 개수", DEFAULT_TENSION_BOLTS);

    /* 피로 등급 */
    limits->fatigueDetailCategory = inputDoubleDefault("\n피로 상세 등급 (MPa) (예: 80, 71, 56)", DEFAULT_FATIGUE_CATEGORY);
}

/**
 * @brief Mode1: 물리 계산 전체 흐름
 */
void performDesignAnalysis(const MaterialProperties *mat, const LoadInputs *loads, const ArmComponent *frontArm, const ArmComponent *verticalArm, const SafetyLimits *limits, const DesignCheckInputs *checkInputs, DesignCheckResults *results) {
    if (!mat || !loads || !frontArm || !verticalArm || !limits || !checkInputs || !results) return;

    /* 초기화 */
    memset(results, 0, sizeof(DesignCheckResults));

    /* 기초 값 */
    double leverArm = frontArm->length; /* m */
    double alphaRad = loads->armAngle * DEG_TO_RAD;

    /* 박스 및 자중 */
    results->boxWeight = loads->boxMass * GRAVITY; /* N */
    results->frontArmArea = rhsArea(frontArm->section.width, frontArm->section.height, frontArm->section.thickness);
    results->armSelfWeight = mat->density * results->frontArmArea * leverArm * GRAVITY; /* N */
    results->totalVerticalLoad = results->boxWeight + results->armSelfWeight;

    /* 굽힘모멘트 (수직성분 고려) */
    double verticalMoment = results->boxWeight * leverArm + results->armSelfWeight * leverArm / 2.0;
    results->totalMoment = verticalMoment * cos(alphaRad); /* N·m, 단순히 각도 반영 */

    /* 전방 암 단면 성능 */
    results->frontArmInertia = rhsInertia(frontArm->section.width, frontArm->section.height, frontArm->section.thickness);
    results->frontArmSectionModulus = rhsSectionModulus(frontArm->section.width, frontArm->section.height, frontArm->section.thickness);

    if (results->frontArmSectionModulus > 0.0) {
        double sigmaMax_Pa = results->totalMoment / results->frontArmSectionModulus; /* Pa */
        double tauMax_Pa = calculateMaxShearStress(results->totalVerticalLoad, frontArm->section.height, frontArm->section.thickness);
        results->maxBendingStress = sigmaMax_Pa / PA_TO_MPA; /* MPa */
        results->maxShearStress = tauMax_Pa / PA_TO_MPA; /* MPa */
        results->vonMisesStress = sqrt(pow(results->maxBendingStress, 2) + VON_MISES_SHEAR_FACTOR * pow(results->maxShearStress, 2)); /* MPa */
    } else {
        results->maxBendingStress = 0.0;
        results->maxShearStress = 0.0;
        results->vonMisesStress = 0.0;
    }

    /* 피로 수명 추정 (단순화) */
    results->fatigueLifeCycles = calculateFatigueLife(results->maxBendingStress, limits->fatigueDetailCategory);

    /* 전방 암 처짐 (m -> convert to mm for storage) */
    if (results->frontArmInertia > 0.0 && mat->elasticModulus > 0.0) {
        double def_m = cantileverDeflection(results->boxWeight, results->armSelfWeight, leverArm, mat->elasticModulus, results->frontArmInertia);
        results->deflection = def_m * M_TO_MM; /* mm */
    } else {
        results->deflection = 0.0;
    }

    /* 수직 암 자중 및 좌굴 계산 */
    results->verticalArmArea = rhsArea(verticalArm->section.width, verticalArm->section.height, verticalArm->section.thickness);
    results->verticalArmInertia = rhsInertia(verticalArm->section.width, verticalArm->section.height, verticalArm->section.thickness);

    if (results->verticalArmArea > 0.0 && results->verticalArmInertia > 0.0) {
        results->verticalArmSelfWeight = mat->density * results->verticalArmArea * verticalArm->length * GRAVITY;
        results->totalCompressiveLoad = results->totalVerticalLoad + results->verticalArmSelfWeight;

        double sigmaCr_Pa = calculateCriticalBucklingStress(verticalArm->length, verticalArm->kFactor, mat->elasticModulus, mat->yieldStrength, results->verticalArmInertia, results->verticalArmArea);
        results->criticalBucklingStress = sigmaCr_Pa / PA_TO_MPA; /* MPa */

        double sigmaComp_Pa = results->totalCompressiveLoad / results->verticalArmArea;
        results->compressiveStress = sigmaComp_Pa / PA_TO_MPA; /* MPa */
    } else {
        results->verticalArmSelfWeight = 0.0;
        results->totalCompressiveLoad = results->totalVerticalLoad;
        results->criticalBucklingStress = 0.0;
        results->compressiveStress = 0.0;
    }

    /* 베이스 압력 (kPa) */
    double padPress_Pa = calculatePadPressure(results->totalMoment, checkInputs->outriggerSpacing, checkInputs->outriggerPadArea);
    results->basePressure = padPress_Pa / N_TO_KN; /* kPa */

    /* 카운터웨이트 필요량 (kg) */
    if (checkInputs->useCounterweight) {
        results->requiredCounterweight = calculateCounterweight(results->totalMoment, checkInputs->counterweightDistance, GRAVITY);
    } else {
        results->requiredCounterweight = 0.0;
    }

    /* 볼트 인장 근사 (N) */
    results->boltTension = calculateBoltTension(results->totalMoment, checkInputs->boltLeverArm, checkInputs->numTensionBolts);
}

/**
 * @brief Mode1 결과 출력 (콘솔)
 */
void printDesignCheckResults(const SafetyLimits *limits, const LoadInputs *loads, const DesignCheckInputs *checkInputs, const DesignCheckResults *results) {
    if (!limits || !loads || !checkInputs || !results) return;

    char buf[128];
    print_separator();
    printf("=== 계산 결과 요약 (Mode 1) ===\n\n");

    /* 하중/치수 요약 */
    printf("총 수직 하중 V = %.3f N (박스 %.3f N + 자중 %.3f N)\n", results->totalVerticalLoad, results->boxWeight, results->armSelfWeight);
    printf("전방 레버암 길이 = %.3f m, 각도 = %.2f deg\n", (loads->horizontalArmLength + checkInputs->boxOffset), loads->armAngle);
    printf("최대 굽힘모멘트 M = %.3f N·m\n", results->totalMoment);

    /* 전방 암 강도 */
    print_separator();
    printf("[전방 암 강도 검토]\n");
    if (results->frontArmSectionModulus <= 0.0 || results->frontArmArea <= 0.0) {
        printf("전방암 단면이 유효하지 않습니다. 단면 치수를 확인하세요.\n");
    } else {
        printf("단면적 A = %s m^2\n", dstr(results->frontArmArea, buf, sizeof(buf)));
        printf("관성모멘트 I = %s m^4\n", dstr(results->frontArmInertia, buf, sizeof(buf)));
        printf("단면계수 W = %s m^3\n", dstr(results->frontArmSectionModulus, buf, sizeof(buf)));
        printf("발생 최대 굽힘응력 σ_bend = %.3f MPa\n", results->maxBendingStress);
        printf("발생 최대 전단응력 τ = %.3f MPa\n", results->maxShearStress);
        printf("조합 응력 (von Mises) = %.3f MPa (허용 σ_allow = %.3f MPa)\n", results->vonMisesStress, limits->allowableStress / PA_TO_MPA);

        double vmPa = results->vonMisesStress * PA_TO_MPA;
        if (vmPa > limits->allowableStress) {
            double margin = (limits->allowableStress - vmPa) / limits->allowableStress * 100.0;
            printf("결과: 허용응력 초과 (여유율 = %.2f%%, 음수는 초과)\n", margin);
        } else {
            double margin = (limits->allowableStress - vmPa) / limits->allowableStress * 100.0;
            printf("결과: 허용내 (여유율 = %.2f%%)\n", margin);
        }
    }

    /* 피로 수명 */
    print_separator();
    printf("[전방 암 피로 수명]\n");
    if (results->fatigueLifeCycles > FATIGUE_INFINITE_CYCLES/10.0) {
        printf("예상 피로 수명: %g 사이클 초과 (무한 수명 근사)\n", FATIGUE_REF_CYCLES);
    } else if (results->fatigueLifeCycles > 0.0) {
        printf("예상 피로 수명: %.3g 사이클\n", results->fatigueLifeCycles);
    } else {
        printf("피로 수명 계산 불가 (입력/응력 확인)\n");
    }

    /* 변위 */
    print_separator();
    printf("[전방 암 변위]\n");
    if (results->deflection > 0.0) {
        printf("탄성 처짐 δ = %.3f mm (허용 처짐 = %.3f mm)\n", results->deflection, limits->deflectionLimit * M_TO_MM);
        if (results->deflection > limits->deflectionLimit * M_TO_MM) {
            printf("경고: 처짐 한계 초과\n");
        }
    } else {
        printf("처짐 계산 불가 또는 매우 작음\n");
    }

    /* 수직 암 좌굴 */
    print_separator();
    printf("[수직 암 좌굴검토]\n");
    if (results->verticalArmArea <= 0.0 || results->verticalArmInertia <= 0.0) {
        printf("수직 암 단면이 유효하지 않습니다.\n");
    } else {
        printf("수직 암 총 압축 하중 P = %.3f N (자중 포함)\n", results->totalCompressiveLoad);
        printf("임계 좌굴응력 σ_cr = %.3f MPa\n", results->criticalBucklingStress);
        printf("압축 응력 σ_comp = %.3f MPa\n", results->compressiveStress);
        if (results->compressiveStress > results->criticalBucklingStress && results->criticalBucklingStress > 0.0) {
            printf("경고: 압축응력이 임계 좌굴응력을 초과합니다 (좌굴 위험)\n");
        } else if (results->criticalBucklingStress > 0.0) {
            double safety = results->criticalBucklingStress / (results->compressiveStress + 1e-12);
            printf("좌굴 안전율 (σ_cr / σ_comp) = %.2f\n", safety);
        }
    }

    /* 베이스 압력 */
    print_separator();
    printf("[베이스 압력]\n");
    printf("아웃리거 간격 = %.3f m, 패드 면적(1개) = %.6f m^2\n", checkInputs->outriggerSpacing, checkInputs->outriggerPadArea);
    printf("근사 최대 베이스 압력 = %.3f kPa\n", results->basePressure);
    printf("-> 지반 허용지지력과 비교하세요.\n");

    /* 카운터웨이트 */
    print_separator();
    if (checkInputs->useCounterweight) {
        if (results->requiredCounterweight > 0.0) {
            printf("카운터웨이트 필요질량 (d_back = %.3f m) >= %.3f kg\n", checkInputs->counterweightDistance, results->requiredCounterweight);
        } else {
            printf("카운터웨이트 필요 없음 또는 계산 불가\n");
        }
    } else {
        printf("카운터웨이트 사용하지 않음\n");
    }

    /* 앵커볼트 */
    print_separator();
    if (results->boltTension > 0.0) {
        printf("근사 각 볼트 인장력 = %.3f N (%.3f kgf)\n", results->boltTension, results->boltTension / GRAVITY);
    } else {
        printf("볼트 인장 근사 계산값 없음\n");
    }

    print_separator();
    printf("Mode 1 출력 종료\n\n");
}

/* ====================================================================== */
/*
/* ====================================================================== */
/* === 10. 최적 설계 탐색 모듈 (Mode 2) - 간단 구현 ===================== */
/*
 * 이 구현은 기존 상단에 선언된 타입/프로토타입과 일치하도록 단순화
 * 된 탐색 루틴을 제공합니다. 더 정교한 목적(단위 변환, 출력 형식 등)은
 * 필요 시 보강하세요.
 */

/* getOptimizerInputs: optimizer.c로 이동됨 */

/* 단순 최적화 루프: 전방/수직 각각 독립 탐색 */
/* Mode2 최적화 함수들은 optimizer.c로 이동됨 */

/* 결과 요약 출력 */
/* printOptimizerSummary: optimizer.c로 이동됨 */

/* 전방 암 최적화: 간단한 규칙 기반 탐색 (응력 및 처짐 허용) */
/* findOptimalFrontArm: optimizer.c로 이동됨 */

/* 수직 암 최적화: 축응력 및 좌굴조건 기반 */
/* findOptimalVerticalArm: optimizer.c로 이동됨 */

/* ====================================================================== */

/* ====================================================================== */
/* === 13. 프로그램 실행 루틴 ========================================= */
/* ====================================================================== */

void runDesignCheck(void);
void runDesignOptimizer(void);

void showBanner(void) {
    printLine(60);
    printf("STRUCTURAL DESIGN CHECKER v7 (INTEGRATED)\n");
    printLine(60);
    printf("모드 1: 단일 설계 검토\n");
    printf("모드 2: 최적 설계 탐색\n");
    printLine(60);
}

/* ---------------------------------------------------------------------- */
/* === 13.1. 메인 루틴 ================================================= */
/* ---------------------------------------------------------------------- */

#ifndef TEST_MODE
int main(void) {
    setbuf(stdout, NULL);
    showBanner();

    int mode = 0;
    printf("실행 모드를 선택하세요 (1: 단일 설계 / 2: 최적 설계): ");
    if (scanf("%d", &mode) != 1) {
        fprintf(stderr, "입력 오류\n");
        return 1;
    }

    switch (mode) {
        case 1:
            runDesignCheck();
            break;
        case 2:
            runDesignOptimizer();
            break;
        default:
            fprintf(stderr, "잘못된 모드입니다.\n");
            break;
    }

    printf("\n프로그램 종료.\n");
    return 0;
}
#endif

/* ====================================================================== */
/* === 14. 테스트용 샘플 데이터 ======================================= */
/* ====================================================================== */

#ifdef TEST_MODE
/* 테스트용 간단한 자동 실행 (gcc -DTEST_MODE 옵션 사용)
 * 단순히 기본값으로 Mode1과 Mode2를 실행
 */
int main(void) {
    setbuf(stdout, NULL);
    printf("=== TEST MODE ===\n");

    /* Mode 1: 단일 설계 검토 */
    printf("\n[ Mode 1 테스트 실행 ]\n");
    runDesignCheck();

    printf("\n[ Mode 2 테스트 실행 ]\n");
    runDesignOptimizer();

    return 0;
}
#endif

/* ====================================================================== */
/* === End of File ====================================================== */
