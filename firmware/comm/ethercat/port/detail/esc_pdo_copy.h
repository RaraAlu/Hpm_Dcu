#ifndef DCU_ESC_PDO_COPY_H
#define DCU_ESC_PDO_COPY_H

#include <stdbool.h>
#include <stdint.h>

static inline __attribute__((always_inline)) bool dcu_esc_pdo_read_words(
    void *destination, const volatile void *source, uint16_t size)
{
    const unsigned width = sizeof(uint32_t);
    /* Never read outside the SM2 range, including at the end of the buffer. */
    if (((uintptr_t)source & (width - 1U)) || size % width)
        return false;
    const volatile uint32_t *src = source;
    uint8_t *dst = destination;
    const unsigned words = size / width;
    if (((uintptr_t)dst & (width - 1U)) == 0) {
        uint8_t *aligned = __builtin_assume_aligned(dst, sizeof(uint32_t));
        for (unsigned i = 0; i < words; ++i) {
            const uint32_t word = src[i];
            __builtin_memcpy(aligned + i * width, &word, width);
        }
    } else {
        for (unsigned i = 0; i < words; ++i) {
            const uint32_t word = src[i];
            __builtin_memcpy(dst + i * width, &word, width);
        }
    }
    return true;
}

static inline __attribute__((always_inline)) void dcu_esc_pdo_copy(
    volatile uint32_t *dest, const void *source, uint16_t size)
{
    const uint8_t *src = source;
    const unsigned words = size / sizeof(*dest);
    if (((uintptr_t)src & (sizeof(*dest) - 1U)) == 0) {
        const uint8_t *aligned = __builtin_assume_aligned(src, sizeof(*dest));
        unsigned i = 0;
        /* The SSC image is aligned. Batch its loads, but keep each PDI write
         * 32-bit and in order so the last access still hands off the SM. */
        for (; i + 4U <= words; i += 4U) {
            uint32_t w0, w1, w2, w3;
            __builtin_memcpy(&w0, aligned + (i + 0U) * sizeof(w0), sizeof(w0));
            __builtin_memcpy(&w1, aligned + (i + 1U) * sizeof(w1), sizeof(w1));
            __builtin_memcpy(&w2, aligned + (i + 2U) * sizeof(w2), sizeof(w2));
            __builtin_memcpy(&w3, aligned + (i + 3U) * sizeof(w3), sizeof(w3));
            *dest++ = w0;
            *dest++ = w1;
            *dest++ = w2;
            *dest++ = w3;
        }
        for (; i < words; ++i) {
            uint32_t word;
            __builtin_memcpy(&word, aligned + i * sizeof(word), sizeof(word));
            *dest++ = word;
        }
    } else {
        /* The SSC source may be only halfword-aligned. */
        for (unsigned i = 0; i < words; ++i) {
            uint32_t word;
            __builtin_memcpy(&word, src + i * sizeof(word), sizeof(word));
            *dest++ = word;
        }
    }
    src += words * sizeof(*dest);
    size %= sizeof(*dest);
    volatile uint8_t *tail = (volatile uint8_t *)dest;
    while (size--)
        *tail++ = *src++;
}

#endif
