#include "sha256.h"

#include <memory.h>
#include <stdint.h>

// see: https://en.wikipedia.org/wiki/SHA-2#Pseudocode
static const uint32_t K[] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
static const uint32_t H[] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                             0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
static inline uint32_t big32(uint32_t v) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return v;
#else
    union {
        struct {
            uint8_t b0, b1, b2, b3;
        };
        uint32_t v;
    } le = {.v = v};
    union {
        struct {
            uint8_t b3, b2, b1, b0;
        };
        uint32_t v;
    } be = {.b0 = le.b0, .b1 = le.b1, .b2 = le.b2, .b3 = le.b3};
    return be.v;
#endif
}
static inline uint64_t big64(uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return v;
#else
    union {
        struct {
            uint8_t b0, b1, b2, b3, b4, b5, b6, b7;
        };
        uint64_t v;
    } le = {.v = v};
    union {
        struct {
            uint8_t b7, b6, b5, b4, b3, b2, b1, b0;
        };
        uint64_t v;
    } be = {
        .b0 = le.b0,
        .b1 = le.b1,
        .b2 = le.b2,
        .b3 = le.b3,
        .b4 = le.b4,
        .b5 = le.b5,
        .b6 = le.b6,
        .b7 = le.b7,
    };
    return be.v;
#endif
}
static inline uint32_t rotr(uint32_t v, uint32_t n) { return (v >> n) | (v << (32 - n)); }
static void update(struct sha256* self);
static void tail(struct sha256* self);

extern void sha256_init(struct sha256* self) {
    memcpy(self->h, H, sizeof H);
    self->bytes = 0;
    self->chunk_size = 0;
}
extern void sha256_update(struct sha256* self, const void* buf, uint32_t bytes) {
    if (self->chunk_size < 0)
        return;
    const char* cur = buf;
    unsigned char* chunk = self->chunk;
    uint32_t chunk_size = self->chunk_size;
    self->bytes += bytes;
    const char* end = &cur[bytes];
    for (const char* next = &cur[64 - chunk_size]; next <= end; next += 64) {
        memcpy(&chunk[chunk_size], cur, next - cur);
        update(self);
        cur = next;
        chunk_size = 0;
    }
    uint32_t rest = end - cur;
    memcpy(&chunk[chunk_size], cur, rest);
    self->chunk_size = chunk_size + rest;
}
extern void sha256_digest(struct sha256* self, void* digest) {
    if (self->chunk_size >= 0) {
        tail(self);
        self->chunk_size = -1;
    }
    uint32_t* buf = (uint32_t*)digest;
    for (int i = 0; i < 8; i++) {
        buf[i] = big32(self->h[i]);
    }
}

static void update(struct sha256* self) {
    uint32_t* sh = self->h;
    uint32_t* cd = (uint32_t*)self->chunk;
    uint32_t w[64];
#pragma unroll
    for (uint32_t i = 0; i < 16; i++) {
        w[i] = big32(cd[i]);
    }
#pragma unroll
    for (uint32_t i = 16; i < 64; i++) {
        uint32_t w16 = w[i - 16], w15 = w[i - 15], w7 = w[i - 7], w2 = w[i - 2];
        uint32_t s0 = rotr(w15, 7) ^ rotr(w15, 18) ^ (w15 >> 3);
        uint32_t s1 = rotr(w2, 17) ^ rotr(w2, 19) ^ (w2 >> 10);
        w[i] = w16 + s0 + w7 + s1;
    }
    uint32_t a = sh[0], b = sh[1], c = sh[2], d = sh[3], e = sh[4], f = sh[5], g = sh[6], h = sh[7];
#pragma unroll
    for (uint32_t i = 0; i < 64; i++) {
        uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t1 = h + s1 + ch + K[i] + w[i];
        uint32_t t2 = s0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    sh[0] += a;
    sh[1] += b;
    sh[2] += c;
    sh[3] += d;
    sh[4] += e;
    sh[5] += f;
    sh[6] += g;
    sh[7] += h;
}
static void tail(struct sha256* self) {
    uint64_t bits = big64(self->bytes * 8);
    self->chunk[self->chunk_size] = 0x80;
    memset(&self->chunk[self->chunk_size + 1], 0, 63 - self->chunk_size);
    if (self->chunk_size + 1 + sizeof bits > 64) {
        update(self);
        memset(self->chunk, 0, 64 - sizeof bits);
    }
    memcpy(&self->chunk[64 - sizeof bits], &bits, sizeof bits);
    update(self);
}
