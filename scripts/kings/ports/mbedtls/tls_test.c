/* mbedTLS 4.2.0 + TF-PSA-Crypto self-tests on bare metal.
 * Runs the upstream AES (ECB/CBC x 128/192/256) and SHA-256 (NIST vectors)
 * self-tests with CYCCNT timing, prints OVERALL verdict. Upstream sources
 * untouched; declarations come from their own private headers. */
#include "board.h"
/* Same prelude the library sources use (pulls config + adjustment chain
 * exactly as aes.c/sha256.c see it). */
#include "tf_psa_crypto_common.h"
#include "mbedtls/private/aes.h"
#include "mbedtls/private/sha256.h"

/* Port glue: upstream platform_util.c drags in entropy/time requirements we
 * don't need for KAT self-tests. This mirrors its zeroize semantics
 * (volatile sweep so the wipe is never optimized out). */
void mbedtls_platform_zeroize(void *buf, size_t len) {
    volatile unsigned char *p = (volatile unsigned char *)buf;
    while (len--) *p++ = 0;
}

int main(void) {
    uint32_t t0, t1;
    int aes, sha, ok;
    board_init();
    uart_puts("MBEDTLS 4.2.0 SELFTEST\n");
    t0 = cycnt();
    aes = mbedtls_aes_self_test(1);
    t1 = cycnt();
    uart_puts("aes_cycles=");
    uart_putdec32(t1 - t0);
    uart_puts(aes == 0 ? " aes=PASSED\n" : " aes=FAILED\n");
    t0 = cycnt();
    sha = mbedtls_sha256_self_test(1);
    t1 = cycnt();
    uart_puts("sha_cycles=");
    uart_putdec32(t1 - t0);
    uart_puts(sha == 0 ? " sha=PASSED\n" : " sha=FAILED\n");
    ok = (aes == 0) && (sha == 0);
    uart_puts(ok ? "OVERALL: PASSED\n" : "OVERALL: FAILED\n");
    uart_puts("TLS_DONE\n");
    for (;;) {}
    return 0;
}
