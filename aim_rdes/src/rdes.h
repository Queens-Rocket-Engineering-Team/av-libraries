#ifndef RDES_H
#define RDES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Realtime Deviation Encoding Scheme (RDES)
 * Original implementation and concept by Kennan (Kenneract)
 *
 * This implementation provides C/C++ variants of RDES encoding
 * and decoding, utilizing a static inline approach for performance,
 * as well as exported symbols via librdes for FFI.
 */

// Core RDES3 settings
#define RDES_LVL_1_MAX 31U
#define RDES_LVL_2_MAX 4095U
#define RDES_LVL_3_MAX 1048575UL
#define RDES_RAW_32_PREFIX 0xFFU

/**
 * Encodes a row of 32-bit integers using full RDES3 specification.
 * @param out_buf Output buffer for compressed data. Must be large enough (up to 5 bytes per column).
 * @param current_row The current row values to encode.
 * @param last_row The previous row values (updated in place).
 * @param num_cols The number of columns in the row.
 * @param force_origin If true, forces raw origin encoding for all columns.
 * @return The number of bytes written to out_buf.
 */
static inline size_t rdes_encode_row_inline(uint8_t* out_buf, const uint32_t* current_row, uint32_t* last_row, size_t num_cols, bool force_origin) {
    size_t ptr = 0;
    if (force_origin) {
        for (size_t col = 0; col < num_cols; ++col) {
            uint32_t val = current_row[col];
            last_row[col] = val;
            if (val & 0x80000000U) { // raw32: 11111111
                out_buf[ptr++] = RDES_RAW_32_PREFIX;
                out_buf[ptr++] = (uint8_t)(val >> 24);
                out_buf[ptr++] = (uint8_t)(val >> 16);
                out_buf[ptr++] = (uint8_t)(val >> 8);
                out_buf[ptr++] = (uint8_t)val;
            } else { // raw31: 0xxxxxxx
                out_buf[ptr++] = (uint8_t)(0x7FU & (val >> 24));
                out_buf[ptr++] = (uint8_t)(val >> 16);
                out_buf[ptr++] = (uint8_t)(val >> 8);
                out_buf[ptr++] = (uint8_t)val;
            }
        }
    } else {
        for (size_t col = 0; col < num_cols; ++col) {
            const uint32_t lastVal = last_row[col];
            const uint32_t curVal = current_row[col];
            const bool signAdd = (curVal >= lastVal);
            const uint32_t offset = signAdd ? (curVal - lastVal) : (lastVal - curVal);

            if (offset <= RDES_LVL_1_MAX) { // Level 1: 1 byte (deltas <= 31)
                out_buf[ptr++] = 0x80U | (signAdd ? 0x40U : 0U) | (uint8_t)(offset & 0x1FU);
            } else if (offset <= RDES_LVL_2_MAX) { // Level 2: 2 bytes (deltas <= 4095)
                out_buf[ptr++] = 0xA0U | (signAdd ? 0x40U : 0U) | (uint8_t)((offset >> 8) & 0x0FU);
                out_buf[ptr++] = (uint8_t)(offset & 0xFFU);
            } else if (offset <= RDES_LVL_3_MAX) { // Level 3: 3 bytes (deltas <= 1048575)
                out_buf[ptr++] = 0xB0U | (signAdd ? 0x40U : 0U) | (uint8_t)((offset >> 16) & 0x0FU);
                out_buf[ptr++] = (uint8_t)(offset >> 8);
                out_buf[ptr++] = (uint8_t)offset;
            } else if (curVal <= 0x7FFFFFFFU) { // Raw 31: 4 bytes
                out_buf[ptr++] = (uint8_t)(0x7FU & (curVal >> 24));
                out_buf[ptr++] = (uint8_t)(curVal >> 16);
                out_buf[ptr++] = (uint8_t)(curVal >> 8);
                out_buf[ptr++] = (uint8_t)curVal;
            } else { // Raw 32: 5 bytes
                out_buf[ptr++] = RDES_RAW_32_PREFIX;
                out_buf[ptr++] = (uint8_t)(curVal >> 24);
                out_buf[ptr++] = (uint8_t)(curVal >> 16);
                out_buf[ptr++] = (uint8_t)(curVal >> 8);
                out_buf[ptr++] = (uint8_t)curVal;
            }
            last_row[col] = curVal;
        }
    }
    return ptr;
}

/**
 * Decodes a row of 32-bit integers using full RDES3 specification.
 * @param in_buf Input buffer containing compressed data.
 * @param current_row Output buffer to store decompressed values.
 * @param last_row The previous row values (used as reference, updated in place).
 * @param num_cols The number of columns in the row.
 * @return The number of bytes read from in_buf.
 */
static inline size_t rdes_decode_row_inline(const uint8_t* in_buf, uint32_t* current_row, uint32_t* last_row, size_t num_cols) {
    size_t ptr = 0;
    for (size_t col = 0; col < num_cols; ++col) {
        uint8_t b1 = in_buf[ptr];
        if ((b1 & 0x80) == 0) { // raw31: 0xxxxxxx (4 bytes)
            uint32_t val = ((uint32_t)(b1 & 0x7F) << 24) | ((uint32_t)in_buf[ptr+1] << 16) | ((uint32_t)in_buf[ptr+2] << 8) | in_buf[ptr+3];
            current_row[col] = val;
            last_row[col] = val;
            ptr += 4;
        } else if (b1 == RDES_RAW_32_PREFIX) { // raw32: 5 bytes
            uint32_t val = ((uint32_t)in_buf[ptr+1] << 24) | ((uint32_t)in_buf[ptr+2] << 16) | ((uint32_t)in_buf[ptr+3] << 8) | in_buf[ptr+4];
            current_row[col] = val;
            last_row[col] = val;
            ptr += 5;
        } else {
            bool signAdd = (b1 & 0x40) != 0;
            if ((b1 & 0x20) == 0) { // Level 1: 1 byte (deltas <= 31)
                uint32_t offset = b1 & 0x1FU;
                uint32_t val = signAdd ? (last_row[col] + offset) : (last_row[col] - offset);
                current_row[col] = val;
                last_row[col] = val;
                ptr += 1;
            } else if ((b1 & 0x10) == 0) { // Level 2: 2 bytes (deltas <= 4095)
                uint32_t offset = (((uint32_t)(b1 & 0x0FU)) << 8) | in_buf[ptr+1];
                uint32_t val = signAdd ? (last_row[col] + offset) : (last_row[col] - offset);
                current_row[col] = val;
                last_row[col] = val;
                ptr += 2;
            } else { // Level 3: 3 bytes (deltas <= 1048575)
                uint32_t offset = (((uint32_t)(b1 & 0x0FU)) << 16) | ((uint32_t)in_buf[ptr+1] << 8) | in_buf[ptr+2];
                uint32_t val = signAdd ? (last_row[col] + offset) : (last_row[col] - offset);
                current_row[col] = val;
                last_row[col] = val;
                ptr += 3;
            }
        }
    }
    return ptr;
}

static inline size_t rdes_decompress_log_inline(const uint8_t* in_buf, size_t in_size,
                                               uint32_t* out_buf, size_t max_rows,
                                               size_t num_cols) {
    size_t in_ptr = 0;
    size_t row_count = 0;
    uint32_t last_row[16] = {0};
    while (in_ptr < in_size && row_count < max_rows) {
        uint32_t* current_row = &out_buf[row_count * num_cols];
        size_t bytes_read = rdes_decode_row_inline(in_buf + in_ptr, current_row, last_row, num_cols);
        if (bytes_read == 0) break;
        in_ptr += bytes_read;
        row_count++;
    }
    return row_count;
}

// Prototypes for exported functions
#ifdef _WIN32
  #define RDES_EXPORT __declspec(dllexport)
#else
  #define RDES_EXPORT __attribute__((visibility("default")))
#endif

RDES_EXPORT size_t rdes_encode_row(uint8_t* out_buf, const uint32_t* current_row, uint32_t* last_row, size_t num_cols, bool force_origin);
RDES_EXPORT size_t rdes_decode_row(const uint8_t* in_buf, uint32_t* current_row, uint32_t* last_row, size_t num_cols);
RDES_EXPORT size_t rdes_decompress_log(const uint8_t* in_buf, size_t in_size, uint32_t* out_buf, size_t max_rows, size_t num_cols);

#ifdef __cplusplus
}
#endif

#endif // RDES_H
