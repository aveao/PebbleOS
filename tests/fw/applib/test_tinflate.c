/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "applib/vendor/tinflate/tinflate.h"

#include <stdlib.h>
#include <string.h>

// Stubs & Fakes
///////////////////////////////////////////////////////////

#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_pbl_malloc.h"

// Helpers
///////////////////////////////////////////////////////////

//! Inflate from a heap copy of `src`, so a read past the declared end lands on a guard page
//! rather than quietly succeeding off neighbouring stack data.
static int prv_inflate(const uint8_t *src, size_t src_len, uint8_t *dest, unsigned int *dest_len) {
  uint8_t *heap_src = malloc(src_len);
  memcpy(heap_src, src, src_len);
  const int res = tinflate_uncompress(dest, dest_len, heap_src, src_len);
  free(heap_src);
  return res;
}

static const uint8_t s_plain[] = {
  0x70, 0x65, 0x62, 0x62, 0x6c, 0x65, 0x20, 0x70, 0x65, 0x62, 0x62, 0x6c, 0x65, 0x20, 0x70, 0x65,
  0x62, 0x62, 0x6c, 0x65, 0x20, 0x70, 0x65, 0x62, 0x62, 0x6c, 0x65, 0x20, 0x70, 0x65, 0x62, 0x62,
  0x6c, 0x65, 0x21,
};
static const uint8_t s_stream[] = {
  0x2b, 0x48, 0x4d, 0x4a, 0xca, 0x49, 0x55, 0x28, 0xc0, 0x4d, 0x29, 0x02, 0x00,
};

// Tests
///////////////////////////////////////////////////////////

void test_tinflate__valid_stream(void) {
  uint8_t out[sizeof(s_plain)];
  unsigned int len = sizeof(out);
  cl_assert_equal_i(prv_inflate(s_stream, sizeof(s_stream), out, &len), TINF_OK);
  cl_assert_equal_i(len, sizeof(s_plain));
  cl_assert(memcmp(out, s_plain, sizeof(s_plain)) == 0);
}

void test_tinflate__truncated_source_rejected(void) {
  // The decoder must stop at sourceLen rather than reading on past the buffer.
  uint8_t out[sizeof(s_plain)];
  for (size_t n = 1; n < sizeof(s_stream); ++n) {
    unsigned int len = sizeof(out);
    cl_assert(prv_inflate(s_stream, n, out, &len) != TINF_OK);
  }
}

void test_tinflate__stored_block_lying_length_rejected(void) {
  // A stored block whose declared length runs past the end of the source
  const uint8_t stream[] = { 0x01, 0xff, 0xff, 0x00, 0x00 };
  uint8_t out[64];
  unsigned int len = sizeof(out);
  cl_assert(prv_inflate(stream, sizeof(stream), out, &len) != TINF_OK);
}

void test_tinflate__back_reference_before_start_rejected(void) {
  // A fixed-Huffman block whose first symbol is a match, so the copy source precedes the
  // output buffer.
  const uint8_t stream[] = { 0x03, 0x52 };
  uint8_t out[64];
  unsigned int len = sizeof(out);
  cl_assert_equal_i(prv_inflate(stream, sizeof(stream), out, &len), TINF_DATA_ERROR);
}

void test_tinflate__dest_overflow_rejected(void) {
  // A well-formed stream that inflates to 300 bytes, into a caller buffer of 64
  const uint8_t stream[] = { 0x73, 0x74, 0x1c, 0x05, 0xc4, 0x02, 0x00 };
  uint8_t out[64];
  unsigned int len = sizeof(out);
  cl_assert_equal_i(prv_inflate(stream, sizeof(stream), out, &len), TINF_DEST_OVERFLOW);
}

void test_tinflate__garbage_rejected(void) {
  const uint8_t stream[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  uint8_t out[64];
  unsigned int len = sizeof(out);
  cl_assert(prv_inflate(stream, sizeof(stream), out, &len) != TINF_OK);
}
