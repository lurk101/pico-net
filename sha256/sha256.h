#pragma once

#include <stdint.h>

#define SHA256_HASH_BYTES 32

struct sha256 {
    uint32_t h[8];
    uint64_t bytes;
    unsigned char chunk[64];
    long chunk_size;
};

void sha256_init(struct sha256* self);
void sha256_update(struct sha256* self, const void* buf, uint32_t bytes);
void sha256_digest(struct sha256* self, void* digest);
