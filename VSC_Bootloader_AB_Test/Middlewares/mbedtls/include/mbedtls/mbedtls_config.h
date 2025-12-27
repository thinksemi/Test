#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* Platform */
#define MBEDTLS_PLATFORM_C
// #define MBEDTLS_PLATFORM_ZEROIZE_ALT

/* Hash */
#define MBEDTLS_SHA256_C

/* Big number */
#define MBEDTLS_BIGNUM_C

/* ECC / ECDSA */
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECDSA_C

/* ASN.1 */
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C

/* No entropy / RNG */
#define MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES
#define MBEDTLS_NO_PLATFORM_ENTROPY

// #include "mbedtls/check_config.h"

#endif
