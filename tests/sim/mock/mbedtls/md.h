#pragma once
#include "../Arduino.h"
#include <openssl/hmac.h>
#include <openssl/evp.h>
typedef int mbedtls_md_type_t;
#define MBEDTLS_MD_SHA256 6
struct mbedtls_md_info_t { int dummy; };
inline const mbedtls_md_info_t* mbedtls_md_info_from_type(mbedtls_md_type_t) { static mbedtls_md_info_t i; return &i; }
inline int mbedtls_md_hmac(const mbedtls_md_info_t*, const unsigned char* key, size_t klen, const unsigned char* in, size_t ilen, unsigned char* out) {
  unsigned int n = 32;
  return HMAC(EVP_sha256(), key, (int)klen, in, ilen, out, &n) ? 0 : -1;
}
