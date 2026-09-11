#include "rdes.h"

/*
 * Shared library exported implementations of RDES functions.
 * Full attribution to Kennan (Kenneract) for the original scheme.
 */

RDES_EXPORT size_t rdes_encode_row(uint8_t* out_buf, const uint32_t* current_row, uint32_t* last_row, size_t num_cols, bool force_origin) {
    return rdes_encode_row_inline(out_buf, current_row, last_row, num_cols, force_origin);
}

RDES_EXPORT size_t rdes_decode_row(const uint8_t* in_buf, uint32_t* current_row, uint32_t* last_row, size_t num_cols) {
    return rdes_decode_row_inline(in_buf, current_row, last_row, num_cols);
}

RDES_EXPORT size_t rdes_decompress_log(const uint8_t* in_buf, size_t in_size, uint32_t* out_buf, size_t max_rows, size_t num_cols) {
    return rdes_decompress_log_inline(in_buf, in_size, out_buf, max_rows, num_cols);
}

