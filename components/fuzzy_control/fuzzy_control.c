/*
 * fuzzy_control.c
 *
 * Mamdani MIMO Fuzzy Logic Controller — Implementation
 *
 * 2 inputs  (pH 0–14, TDS 0–1300 ppm)
 * 3 outputs (pH_Up ms, pH_Down ms, Nutrient ms)
 * 9 rules, Min-Max inference, Centre-of-Gravity defuzzification.
 *
 * Design constraints:
 *   - Zero dynamic memory allocation (all buffers are static)
 *   - Zero external dependencies (pure C99, no ESP-IDF headers required)
 *   - Pre-discretised output universes for deterministic COG computation
 */

#include "fuzzy_control.h"

#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Discretisation Constants                                                   */
/* -------------------------------------------------------------------------- */

#define PH_DISC_STEPS    51          /* points in 0..5000 ms */
#define NUT_DISC_STEPS   51          /* points in 0..20000 ms */

#define PH_STEP          100.0f      /* 5000 / 50 */
#define NUT_STEP         400.0f      /* 20000 / 50 */

#define PH_OUT_MIN        0.0f
#define PH_OUT_MAX     5000.0f
#define NUT_OUT_MIN       0.0f
#define NUT_OUT_MAX   20000.0f

/* -------------------------------------------------------------------------- */
/* Input Membership Function Indices                                           */
/* -------------------------------------------------------------------------- */

typedef enum {
    PH_ASAM = 0,
    PH_NETRAL,
    PH_BASA,
    PH_MF_COUNT
} ph_mf_t;

typedef enum {
    TDS_S_KURANG = 0,
    TDS_KURANG,
    TDS_CUKUP,
    TDS_MF_COUNT
} tds_mf_t;

/* -------------------------------------------------------------------------- */
/* Output Membership Function Indices                                         */
/* -------------------------------------------------------------------------- */

typedef enum {
    OUT_STOP = 0,
    OUT_SINGKAT,
    OUT_SEDANG,
    OUT_LAMA,
    OUT_MF_COUNT
} out_mf_t;

/* -------------------------------------------------------------------------- */
/* Trapezoidal MF Parameters  { a, b, c, d }                                  */
/* -------------------------------------------------------------------------- */

/* Input: pH (0–14) */
static const float ph_mf_params[PH_MF_COUNT][4] = {
    /* Asam   */ {  0.0f,  0.0f,  6.0f,  6.5f },
    /* Netral */ {  6.0f,  6.5f,  7.5f,  8.0f },
    /* Basa   */ {  7.5f,  8.0f, 14.0f, 14.0f },
};

/* Input: TDS (0–1300 ppm) */
static const float tds_mf_params[TDS_MF_COUNT][4] = {
    /* SKurang */ {   0.0f,   0.0f,  300.0f,  550.0f },
    /* Kurang  */ { 300.0f, 550.0f,  800.0f, 1050.0f },
    /* Cukup   */ { 800.0f,1050.0f, 1300.0f, 1300.0f },
};

/* Output: pH Up / pH Down (0–5000 ms) */
static const float ph_out_mf_params[OUT_MF_COUNT][4] = {
    /* Stop    */ {    0.0f,    0.0f,    0.0f,    0.0f },
    /* Singkat */ {    0.0f,    0.0f, 1500.0f, 2000.0f },
    /* Sedang  */ { 1500.0f, 2000.0f, 3000.0f, 3500.0f },
    /* Lama    */ { 3000.0f, 3500.0f, 5000.0f, 5000.0f },
};

/* Output: Nutrient (0–20000 ms) */
static const float nut_out_mf_params[OUT_MF_COUNT][4] = {
    /* Stop    */ {     0.0f,     0.0f,     0.0f,     0.0f },
    /* Singkat */ {     0.0f,     0.0f,  4000.0f,  8000.0f },
    /* Sedang  */ {  4000.0f,  8000.0f, 12000.0f, 16000.0f },
    /* Lama    */ { 12000.0f, 16000.0f, 20000.0f, 20000.0f },
};

/* -------------------------------------------------------------------------- */
/* Rule Base (9 rules)                                                        */
/* -------------------------------------------------------------------------- */

typedef struct {
    ph_mf_t   ph;       /* pH membership function index */
    tds_mf_t  tds;      /* TDS membership function index */
    out_mf_t  ph_up;    /* pH Up output MF index */
    out_mf_t  ph_dn;    /* pH Down output MF index */
    out_mf_t  nut;      /* Nutrient output MF index */
} rule_t;

static const rule_t rules[] = {
    /* R1 */ { PH_ASAM,   TDS_S_KURANG, OUT_LAMA,    OUT_STOP,    OUT_LAMA    },
    /* R2 */ { PH_ASAM,   TDS_KURANG,   OUT_SEDANG,  OUT_STOP,    OUT_SEDANG  },
    /* R3 */ { PH_ASAM,   TDS_CUKUP,    OUT_SINGKAT, OUT_STOP,    OUT_STOP    },
    /* R4 */ { PH_NETRAL, TDS_S_KURANG, OUT_STOP,    OUT_STOP,    OUT_LAMA    },
    /* R5 */ { PH_NETRAL, TDS_KURANG,   OUT_STOP,    OUT_STOP,    OUT_SEDANG  },
    /* R6 */ { PH_NETRAL, TDS_CUKUP,    OUT_STOP,    OUT_STOP,    OUT_STOP    },
    /* R7 */ { PH_BASA,   TDS_S_KURANG, OUT_STOP,    OUT_LAMA,    OUT_LAMA    },
    /* R8 */ { PH_BASA,   TDS_KURANG,   OUT_STOP,    OUT_SEDANG,  OUT_SEDANG  },
    /* R9 */ { PH_BASA,   TDS_CUKUP,    OUT_STOP,    OUT_SINGKAT, OUT_STOP    },
};

#define RULE_COUNT (sizeof(rules) / sizeof(rules[0]))

/* -------------------------------------------------------------------------- */
/* Pre-computed Output MF Look-up Tables (filled once at first call)          */
/* -------------------------------------------------------------------------- */

/* pH outputs:   ph_out_lut[mf][i] = μ(z_i) for discrete point z_i */
static float ph_out_lut[OUT_MF_COUNT][PH_DISC_STEPS];
/* Nutrient:    nut_out_lut[mf][i] = μ(z_i) */
static float nut_out_lut[OUT_MF_COUNT][NUT_DISC_STEPS];

/* Aggregation working buffers (three independent outputs) */
static float ph_up_agg[PH_DISC_STEPS];
static float ph_dn_agg[PH_DISC_STEPS];
static float nut_agg[NUT_DISC_STEPS];

static bool s_lut_ready = false;

/* -------------------------------------------------------------------------- */
/* Trapezoidal Membership Function                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Evaluate a trapezoidal membership function at point x.
 *
 * Parameters [a, b, c, d] define the trapezoid:
 *   μ = 0                 if x ≤ a  or  x ≥ d
 *   μ = (x-a)/(b-a)       if a < x < b
 *   μ = 1                 if b ≤ x ≤ c
 *   μ = (d-x)/(d-c)       if c < x < d
 *
 * Degenerate cases:
 *   a == b  → left shoulder:  μ = 1 for x ≤ c, then ramp down
 *   c == d  → right shoulder: μ = 1 for x ≥ b, ramp up before
 */
static float fuzzify_trap(float x, float a, float b, float c, float d)
{
    if (x <= a || x >= d) {
        return 0.0f;
    }

    /* Left shoulder (a == b): no rising edge, flat top starts at a */
    if (a == b) {
        if (x <= c) return 1.0f;
        return (d - x) / (d - c);          /* ramp down */
    }

    /* Right shoulder (c == d): no falling edge */
    if (c == d) {
        if (x >= b) return 1.0f;
        return (x - a) / (b - a);          /* ramp up */
    }

    /* Standard trapezoid with both rising and falling edges */
    if (x < b) {
        return (x - a) / (b - a);          /* ramp up */
    }
    if (x <= c) {
        return 1.0f;                       /* flat top */
    }
    return (d - x) / (d - c);              /* ramp down */
}

/* -------------------------------------------------------------------------- */
/* LUT Initialisation                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Pre-compute all output membership values at each discrete point.
 *
 * For Stop (singleton at 0), we set μ = 1 at z = 0 and μ = 0 elsewhere,
 * so that COG correctly yields 0 when only Stop fires.
 */
static void build_output_luts(void)
{
    if (s_lut_ready) return;

    /* ---- pH outputs ---- */
    for (int mf = 0; mf < OUT_MF_COUNT; mf++) {
        if (mf == OUT_STOP) {
            /* Singleton at z = 0 */
            memset(ph_out_lut[mf], 0, sizeof(ph_out_lut[mf]));
            ph_out_lut[mf][0] = 1.0f;
            continue;
        }
        float a = ph_out_mf_params[mf][0];
        float b = ph_out_mf_params[mf][1];
        float c = ph_out_mf_params[mf][2];
        float d = ph_out_mf_params[mf][3];
        for (int i = 0; i < PH_DISC_STEPS; i++) {
            float z = (float)i * PH_STEP;
            ph_out_lut[mf][i] = fuzzify_trap(z, a, b, c, d);
        }
    }

    /* ---- Nutrient outputs ---- */
    for (int mf = 0; mf < OUT_MF_COUNT; mf++) {
        if (mf == OUT_STOP) {
            memset(nut_out_lut[mf], 0, sizeof(nut_out_lut[mf]));
            nut_out_lut[mf][0] = 1.0f;
            continue;
        }
        float a = nut_out_mf_params[mf][0];
        float b = nut_out_mf_params[mf][1];
        float c = nut_out_mf_params[mf][2];
        float d = nut_out_mf_params[mf][3];
        for (int i = 0; i < NUT_DISC_STEPS; i++) {
            float z = (float)i * NUT_STEP;
            nut_out_lut[mf][i] = fuzzify_trap(z, a, b, c, d);
        }
    }

    s_lut_ready = true;
}

/* -------------------------------------------------------------------------- */
/* Centre-of-Gravity Defuzzification                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compute the COG of an aggregated membership array.
 *
 * @param agg   Aggregated μ values at each discrete point.
 * @param steps Number of discrete points.
 * @param step  Spacing between points (in output units).
 * @return COG z-coordinate, or 0.0f if denominator is zero.
 */
static float defuzzify_cog(const float *agg, int steps, float step)
{
    float num = 0.0f;
    float den = 0.0f;

    for (int i = 0; i < steps; i++) {
        float z = (float)i * step;
        float mu = agg[i];

        if (mu <= 0.0f) continue;      /* skip — no contribution */

        num += z * mu;
        den += mu;
    }

    if (den < 1e-9f) {
        return 0.0f;                    /* all μ ≈ 0 → Stop */
    }

    return num / den;
}

/* -------------------------------------------------------------------------- */
/* MIMO Fuzzy Inference                                                       */
/* -------------------------------------------------------------------------- */

void fuzzy_evaluate(float pH, float tds,
                    float *ph_up_ms, float *ph_dn_ms, float *nut_ms)
{
    /* Ensure LUTs are built (first-call init) */
    build_output_luts();

    /* ---- Step 1: Fuzzify inputs ---- */
    float mu_ph[PH_MF_COUNT];
    float mu_tds[TDS_MF_COUNT];

    for (int mf = 0; mf < PH_MF_COUNT; mf++) {
        float a = ph_mf_params[mf][0];
        float b = ph_mf_params[mf][1];
        float c = ph_mf_params[mf][2];
        float d = ph_mf_params[mf][3];
        mu_ph[mf] = fuzzify_trap(pH, a, b, c, d);
    }
    for (int mf = 0; mf < TDS_MF_COUNT; mf++) {
        float a = tds_mf_params[mf][0];
        float b = tds_mf_params[mf][1];
        float c = tds_mf_params[mf][2];
        float d = tds_mf_params[mf][3];
        mu_tds[mf] = fuzzify_trap(tds, a, b, c, d);
    }

    /* ---- Step 2 & 3: Inference + Aggregation (Min-Max) ---- */

    /* Clear aggregation buffers */
    memset(ph_up_agg, 0, sizeof(ph_up_agg));
    memset(ph_dn_agg, 0, sizeof(ph_dn_agg));
    memset(nut_agg,   0, sizeof(nut_agg));

    for (size_t r = 0; r < RULE_COUNT; r++) {
        const rule_t *rule = &rules[r];

        /* Firing strength: AND = min(μ_pH, μ_TDS) */
        float strength = mu_ph[rule->ph];
        if (mu_tds[rule->tds] < strength) {
            strength = mu_tds[rule->tds];
        }

        if (strength <= 0.0f) continue;   /* rule not activated */

        /* ---- pH Up: clip & aggregate ---- */
        for (int i = 0; i < PH_DISC_STEPS; i++) {
            float val = ph_out_lut[rule->ph_up][i];
            if (val > strength) val = strength;
            if (val > ph_up_agg[i]) ph_up_agg[i] = val;
        }

        /* ---- pH Down: clip & aggregate ---- */
        for (int i = 0; i < PH_DISC_STEPS; i++) {
            float val = ph_out_lut[rule->ph_dn][i];
            if (val > strength) val = strength;
            if (val > ph_dn_agg[i]) ph_dn_agg[i] = val;
        }

        /* ---- Nutrient: clip & aggregate ---- */
        for (int i = 0; i < NUT_DISC_STEPS; i++) {
            float val = nut_out_lut[rule->nut][i];
            if (val > strength) val = strength;
            if (val > nut_agg[i]) nut_agg[i] = val;
        }
    }

    /* ---- Step 4: Defuzzification (COG) ---- */

    *ph_up_ms = defuzzify_cog(ph_up_agg, PH_DISC_STEPS, PH_STEP);
    *ph_dn_ms = defuzzify_cog(ph_dn_agg, PH_DISC_STEPS, PH_STEP);
    *nut_ms   = defuzzify_cog(nut_agg,   NUT_DISC_STEPS, NUT_STEP);
}
