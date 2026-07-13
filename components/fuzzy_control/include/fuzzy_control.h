/*
 * fuzzy_control.h
 *
 * Mamdani MIMO Fuzzy Logic Controller for AQUAHARU
 *
 * Pure-C library — zero dynamic allocation, zero external dependencies.
 *
 * Inputs:   pH (0–14), TDS ppm (0–1300)
 * Outputs:  pH Up duration (ms), pH Down duration (ms), Nutrient duration (ms)
 * Rules:    9-rule base, Min-Max inference, Centre-of-Gravity defuzzification
 */

#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Evaluate the full Mamdani fuzzy inference engine.
 *
 * @param[in]  pH        Current pH value (0–14).
 * @param[in]  tds       Current TDS value in ppm (0–1300).
 * @param[out] ph_up_ms  Dosing duration for pH-Up pump (0–5000 ms).
 * @param[out] ph_dn_ms  Dosing duration for pH-Down pump (0–5000 ms).
 * @param[out] nut_ms    Dosing duration for nutrient pumps (0–20000 ms).
 */
void fuzzy_evaluate(float pH, float tds,
                    float *ph_up_ms, float *ph_dn_ms, float *nut_ms);

#ifdef __cplusplus
}
#endif

#endif /* FUZZY_CONTROL_H */
