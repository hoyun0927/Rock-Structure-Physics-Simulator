#ifndef DESIGN_SHARED_H
#define DESIGN_SHARED_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* 물리 상수 */
#ifndef GRAVITY
#define GRAVITY 9.81
#endif
#ifndef PI
#define PI 3.14159265358979323846
#endif

/* 단위/변환 */
#ifndef PA_TO_MPA
#define PA_TO_MPA 1e6
#endif
#ifndef M_TO_MM
#define M_TO_MM 1000.0
#endif
#ifndef N_TO_KN
#define N_TO_KN 1000.0
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI/180.0)
#endif

/* 계산 상수 (일부만 필요시 추가) */
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

/* 공용 타입들 (설계/최적화에서 사용) */
typedef struct {
    double elasticModulus; /* Pa */
    double yieldStrength;  /* Pa */
    double density;        /* kg/m^3 */
} MaterialProperties;

/* RHS 직사각 중공 단면 (공통 타입으로 유지) */
typedef struct {
    double width;    /* 외부 폭 b (m) */
    double height;   /* 외부 높이 h (m) */
    double thickness;/* 두께 t (m) */
} SectionRHS;

typedef struct {
    SectionRHS section;
    double length;  /* m */
    double kFactor; /* 좌굴 K계수 */
} ArmComponent;

typedef struct {
    double boxMass;            /* kg */
    double horizontalArmLength;/* m */
    double armAngle;           /* deg (from horizontal) */
} LoadInputs;

typedef struct {
    double safetyFactor;          /* 무차원 */
    double allowableStress;       /* Pa */
    double deflectionLimit;       /* m */
    double fatigueDetailCategory; /* MPa (S-N reference) */
} SafetyLimits;

typedef struct {
    double boxOffset;           /* m */
    double outriggerSpacing;    /* m */
    double outriggerPadArea;    /* m^2 (1 pad) */
    int    useCounterweight;    /* 0/1 */
    double counterweightDistance;/* m */
    double boltLeverArm;         /* m */
    int    numTensionBolts;
} DesignCheckInputs;

typedef struct {
    double boxWeight; /* N */
    double armSelfWeight; /* N */
    double totalVerticalLoad; /* N */
    double totalMoment; /* N·m */
    double frontArmArea; /* m^2 */
    double frontArmInertia; /* m^4 */
    double frontArmSectionModulus; /* m^3 */
    double maxBendingStress; /* MPa */
    double maxShearStress; /* MPa */
    double vonMisesStress; /* MPa */
    double fatigueLifeCycles; /* cycles or large number for infinite */
    double deflection; /* mm */
    double verticalArmArea; /* m^2 */
    double verticalArmInertia; /* m^4 */
    double verticalArmSelfWeight; /* N */
    double totalCompressiveLoad; /* N */
    double criticalBucklingStress; /* MPa */
    double compressiveStress; /* MPa */
    double basePressure; /* kPa */
    double requiredCounterweight; /* kg */
    double boltTension; /* N per bolt approx */
} DesignCheckResults;

/* 탐색 범위 (for Mode2) */
typedef struct {
    double min;
    double max;
    double step;
} SearchRange;

/* Mode2 입력 범위 집합 */
typedef struct {
    SearchRange frontWidthRange;
    SearchRange frontHeightRange;
    SearchRange frontThicknessRange;
    SearchRange verticalWidthRange;
    SearchRange verticalHeightRange;
    SearchRange verticalThicknessRange;
} OptimizerInputs;

/* 단일 최적화 결과: 전방 또는 수직용 */
typedef struct {
    SectionRHS bestSection; /* 선택된 치수 */
    double bestMass;        /* kg */
    bool found;             /* 조건 만족 여부 */
    long iterations;        /* 탐색 횟수 */
} SingleOptResult;

/* 통합 최적화 결과 (양쪽) */
typedef struct {
    SingleOptResult front;
    SingleOptResult vertical;
    double totalIterations;
} CombinedOptResult;

/* 공용 함수 프로토타입(optimizer.c에서 사용) */
double rhsInertia(double width, double height, double thickness);
double rhsSectionModulus(double width, double height, double thickness);
double rhsArea(double width, double height, double thickness);
double cantileverDeflection(double pointLoad, double udlTotal, double length, double elasticModulus, double inertia);
double calculateCriticalBucklingStress(double length, double kFactor, double elasticModulus, double yieldStrength, double inertia, double area);

/* 입력 헬퍼(설계.c에 정의됨) */
double inputDoubleDefault(const char *prompt, double def);

/* 출력 헬퍼 */
void print_separator(void);

#endif /* DESIGN_SHARED_H */
