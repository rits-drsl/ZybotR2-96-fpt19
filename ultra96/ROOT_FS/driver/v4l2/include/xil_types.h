#ifndef XIL_TYPES_H
#define XIL_TYPES_H

#include <linux/types.h>
#define UPPER_32_BITS(n) ((u32)(((n) >> 16) >> 16))
#define LOWER_32_BITS(n) ((u32)(n))
typedef uintptr_t UINTPTR;
typedef char char8;

#define XIL_COMPONENT_IS_READY     0x11111111U

#endif /* XIL_TYPES_H */
