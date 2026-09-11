// Hand-rolled check suite for the SHA-256 implementation against the
// standard FIPS 180-4 published test vectors. A homegrown hash
// implementation used for a correctness claim ("identical SHA-256 digest")
// is worthless if it has not itself been diffed against known-correct
// output.
#include <cstdio>
#include <cstdlib>
#include <string>

#include "shmmw/sha256.hpp"

using namespace shmmw;

static int g_checks = 0;
#define CHECK_EQ(a, b) do { \
    ++g_checks; \
    auto va = (a); auto vb = (b); \
    if (va != vb) { \
        std::fprintf(stderr, "FAIL %s:%d: got %s want %s\n", __FILE__, __LINE__, \
                     std::string(va).c_str(), std::string(vb).c_str()); \
        std::exit(1); \
    } \
} while (0)

int main() {
    // FIPS 180-4 published test vectors.
    CHECK_EQ(Sha256::hash_string(""),
             std::string("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
    CHECK_EQ(Sha256::hash_string("abc"),
             std::string("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    CHECK_EQ(Sha256::hash_string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
             std::string("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));

    ++g_checks;
    if (Sha256::hash_string("").size() != 64) {
        std::fprintf(stderr, "FAIL: hex digest length %zu != 64\n", Sha256::hash_string("").size());
        return 1;
    }

    // Determinism and avalanche: same input always digests the same way; a
    // one-character change must not leave the digest unchanged.
    CHECK_EQ(Sha256::hash_string("hello"), Sha256::hash_string("hello"));
    ++g_checks;
    if (Sha256::hash_string("hello") == Sha256::hash_string("hellO")) {
        std::fprintf(stderr, "FAIL: single-character change did not change digest\n");
        return 1;
    }

    // Multi-update() calls must produce the same digest as one big update(),
    // since block_runner streams log rows incrementally rather than
    // building one giant string first.
    Sha256 incremental;
    incremental.update("ab", 2);
    incremental.update("c", 1);
    CHECK_EQ(incremental.hex_digest(), Sha256::hash_string("abc"));

    std::printf("check_sha256: %d checks passed\n", g_checks);
    return 0;
}
