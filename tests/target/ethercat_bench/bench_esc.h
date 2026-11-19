#ifndef DCU_BENCH_ESC_H
#define DCU_BENCH_ESC_H

#include "board.h"
#include "hpm_esc_drv.h"

typedef bool (*dcu_bench_sii_reader_t)(uint32_t word, unsigned size, uint64_t *data);

hpm_stat_t dcu_bench_esc_start(dcu_bench_sii_reader_t reader, unsigned timeout_ms);
bool dcu_bench_sii_service(dcu_bench_sii_reader_t reader);
bool dcu_bench_sii_prepare_read(dcu_bench_sii_reader_t reader, uint32_t word, unsigned size);
bool dcu_bench_sii_prepare_reload(dcu_bench_sii_reader_t reader);

#endif
