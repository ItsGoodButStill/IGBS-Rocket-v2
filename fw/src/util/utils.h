/*
 *  Created: Jun 25, 2021
 *  Author:  Jakub Parez
 *  Company: ATEsystem s.r.o.
 *  Project: Varitest
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <stddef.h>

#define PI                            3.14159265359
#define J(a,b)                        a##b
#define WTF                           ASSERT(0)
#define ARRAYSIZE(a)                  (sizeof(a)/sizeof(a[0]))
#define ARGSIZE(...)                  (sizeof((uint8_t[]){__VA_ARGS__})/sizeof(uint8_t))
#define BITMAP_GET_BIT(m,b,s)         (*(m + (b / s)) &  (uint8_t)(1 << (b % s)))
#define BITMAP_SET_BIT(m,b,s)         (*(m + (b / s)) |= (uint8_t)(1 << (b % s)))
#define U8_TO_U16(h,l)                ((uint16_t)(h << 8u) | l)
#define U8_TO_U32(h,a,b,l)            ((uint32_t)(h << 24u) | (uint32_t)(a << 16u) | (uint32_t)(b << 8u) | l)
#define U16_TO_U8_L(x)                ((uint8_t)(((x) & 0x00FF)))
#define U16_TO_U8_H(x)                ((uint8_t)(((x) & 0xFF00) >> 8))
#define U24_TO_U8_L(x)                ((uint8_t)(((x) & 0x000000FF)))
#define U24_TO_U8_M(x)                ((uint8_t)(((x) & 0x0000FF00) >> 8))
#define U24_TO_U8_H(x)                ((uint8_t)(((x) & 0x00FF0000) >> 16))
#define U32_TO_U8_L(x)                ((uint8_t)(((x) & 0x000000FF)))
#define U32_TO_U8_B(x)                ((uint8_t)(((x) & 0x0000FF00) >> 8))
#define U32_TO_U8_A(x)                ((uint8_t)(((x) & 0x00FF0000) >> 16))
#define U32_TO_U8_H(x)                ((uint8_t)(((x) & 0xFF000000) >> 24))

#define Pu8 "hu"
#define Pd8 "hd"
#define Px8 "hx"
#define Pu16 "hu"
#define Pd16 "hd"
#define Px16 "hx"
#define Pu32 "u"
#define Pd32 "d"
#define Px32 "x"
#define Pu64 "llu" // or possibly "lu"
#define Pd64 "lld" // or possibly "ld"
#define Px64 "llx" // or possibly "lx"

#define ASSERT(expr) \
  do {                                          \
    if (!(expr)) {                              \
      assert2(__FILE__, __LINE__);              \
    }                                           \
  } while (0)

void assert2(const char *file, uint32_t line);

static inline int get_last_circ_idx(int pos, int len)
{
    int last_idx = pos - 1;
    if (last_idx < 0)
        last_idx = len - 1;
    return last_idx;
}

void split3(const char* input, size_t len, char delim, char* str1, char* str2, char* str3);

int ipv4_atoi(const char* str, size_t str_len, uint8_t* out);

uint8_t charHex2int(char hex);

double get_freq(int* prescaler, int* reload, int max_reload, int freq_osc, int freq_want);

void busy_wait(int x);
float fastlog2 (float x);
float fastlog(float x);
float fastexp(float p);
float fastpow2(float p);
int itoa_fast(char* s, int num, int radix);
long long lltoa_fast(char* s, long long num, int radix);

/* Author: Jakub Parez
 * Descr:  ultra fast float sprintf
 */
int sprint_fast(char* s, const char* format, double fVal, int prec);
char* ftoa2(double f, char * buf, int precision);

#endif /* INC_UTILS_H_ */
