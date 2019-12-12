#ifndef XIL_IO_H
#define XIL_IO_H

#define INLINE __inline__

static INLINE u32 Xil_In32(UINTPTR Addr)
{
    return *(volatile u32 *) Addr;
}

static INLINE void Xil_Out32(UINTPTR Addr, u32 Value)
{
    #ifndef ENABLE_SAFETY
    volatile u32 *LocalAddr = (volatile u32 *)Addr;
    *LocalAddr = Value;
    #else
    XStl_RegUpdate(Addr, Value);
    #endif
}

#endif /* XIL_IO_H */
