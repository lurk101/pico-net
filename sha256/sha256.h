#pragma once

#include <stdint.h>

#define SHA256_HASH_BYTES 32

typedef struct {
    uint32_t h[8];
    uint64_t bytes;
    uint8_t chunk[64];
    uint32_t chunk_size;
} sha256_t;

void sha256_init(sha256_t* self);
void sha256_update(sha256_t* self, const void* buf, uint32_t bytes);
void sha256_digest(sha256_t* self, void* digest);
