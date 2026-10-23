#pragma once
// Minimal self-contained SHA-256 (FIPS 180-4), hand-rolled to avoid pulling
// an external crypto dependency into a project whose only use of the hash is
// a deterministic-replay fingerprint, not anything security-sensitive.
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace shmmw {

class Sha256 {
public:
    Sha256() { reset(); }

    void reset() {
        h_[0]=0x6a09e667; h_[1]=0xbb67ae85; h_[2]=0x3c6ef372; h_[3]=0xa54ff53a;
        h_[4]=0x510e527f; h_[5]=0x9b05688c; h_[6]=0x1f83d9ab; h_[7]=0x5be0cd19;
        buffer_.clear();
        total_len_ = 0;
    }

    void update(const void* data, size_t len) {
        // Processes full 64-byte blocks straight out of the caller's
        // buffer and only ever copies the sub-64-byte remainder into
        // buffer_; the previous version always appended the whole input to
        // buffer_ first and then called vector::erase(begin, begin+64) once
        // per block, which shifts everything after the erased block down
        // by 64 bytes every time, an O(len) operation repeated len/64
        // times. That is O(len^2) and was invisible at this repo's original
        // call sizes (one 64-byte Message at a time) but took several
        // minutes on an 8MB voxel-grid buffer in the perception extension's
        // replay digest (see that README's Findings) before being found
        // and fixed here, in the one shared utility, not worked around at
        // the call site.
        const uint8_t* p = static_cast<const uint8_t*>(data);
        total_len_ += len;

        if (!buffer_.empty()) {
            size_t need = 64 - buffer_.size();
            size_t take = need < len ? need : len;
            buffer_.insert(buffer_.end(), p, p + take);
            p += take;
            len -= take;
            if (buffer_.size() == 64) {
                process_(buffer_.data());
                buffer_.clear();
            }
        }
        while (len >= 64) {
            process_(p);
            p += 64;
            len -= 64;
        }
        if (len > 0) {
            buffer_.insert(buffer_.end(), p, p + len);
        }
    }

    std::string hex_digest() {
        // Pad and process the final block(s); operates on a copy so the
        // object can, in principle, keep accepting update() calls afterward
        // for a running digest if a caller ever needs that.
        std::vector<uint8_t> tail = buffer_;
        uint64_t bit_len = total_len_ * 8;
        tail.push_back(0x80);
        while (tail.size() % 64 != 56) tail.push_back(0x00);
        for (int i = 7; i >= 0; --i) tail.push_back(static_cast<uint8_t>((bit_len >> (i * 8)) & 0xff));

        uint32_t h[8];
        std::memcpy(h, h_, sizeof(h));
        for (size_t off = 0; off < tail.size(); off += 64) {
            process_into_(tail.data() + off, h);
        }
        std::ostringstream oss;
        for (int i = 0; i < 8; ++i) {
            char buf[9];
            std::snprintf(buf, sizeof(buf), "%08x", h[i]);
            oss << buf;
        }
        return oss.str();
    }

    static std::string hash_string(const std::string& s) {
        Sha256 sh;
        sh.update(s.data(), s.size());
        return sh.hex_digest();
    }

private:
    static uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

    void process_(const uint8_t* block) { process_into_(block, h_); }

    static void process_into_(const uint8_t* block, uint32_t h[8]) {
        static const uint32_t k[64] = {
            0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
            0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
            0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
            0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
            0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
            0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
            0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
            0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};

        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (uint32_t(block[i*4]) << 24) | (uint32_t(block[i*4+1]) << 16) |
                   (uint32_t(block[i*4+2]) << 8) | uint32_t(block[i*4+3]);
        }
        for (int i = 16; i < 64; ++i) {
            uint32_t s0 = rotr(w[i-15], 7) ^ rotr(w[i-15], 18) ^ (w[i-15] >> 3);
            uint32_t s1 = rotr(w[i-2], 17) ^ rotr(w[i-2], 19) ^ (w[i-2] >> 10);
            w[i] = w[i-16] + s0 + w[i-7] + s1;
        }

        uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
        for (int i = 0; i < 64; ++i) {
            uint32_t S1 = rotr(e,6) ^ rotr(e,11) ^ rotr(e,25);
            uint32_t ch = (e & f) ^ (~e & g);
            uint32_t temp1 = hh + S1 + ch + k[i] + w[i];
            uint32_t S0 = rotr(a,2) ^ rotr(a,13) ^ rotr(a,22);
            uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            uint32_t temp2 = S0 + maj;
            hh=g; g=f; f=e; e=d+temp1; d=c; c=b; b=a; a=temp1+temp2;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
    }

    uint32_t h_[8];
    std::vector<uint8_t> buffer_;
    uint64_t total_len_ = 0;
};

} // namespace shmmw
