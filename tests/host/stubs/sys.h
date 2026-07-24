#ifndef TEST_SYS_H
#define TEST_SYS_H

#include <stdint.h>

typedef uint8_t  u8;
typedef int8_t   s8;
typedef uint16_t u16;
typedef int16_t  s16;
typedef uint32_t u32;
typedef int32_t  s32;

static inline u32 __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

#endif
