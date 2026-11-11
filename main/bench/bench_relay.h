#ifndef AQUAHARU_BENCH_RELAY_H
#define AQUAHARU_BENCH_RELAY_H

/* Bench only: initialize six active-HIGH relay inputs OFF, then wait for
 * explicit serial commands. Never starts normal control or sensor tasks. */
void bench_relay_start(void);

#endif
