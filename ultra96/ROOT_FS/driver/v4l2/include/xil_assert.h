#ifndef XIL_ASSERT_H
#define XIL_ASSERT_H

extern u32 Xil_AssertStatus;

#define XIL_ASSERT_NONE     0U
#define XIL_ASSERT_OCCURRED 1U

#define Xil_AssertNonvoid(Expression)               \
    {                                               \
        if (Expression) {                           \
            Xil_AssertStatus = XIL_ASSERT_NONE;     \
        } else {                                    \
            Xil_AssertStatus = XIL_ASSERT_OCCURRED; \
            return 0;                               \
        }                                           \
    }

#define Xil_AssertVoid(Expression)                  \
    {                                               \
        if (Expression) {                           \
            Xil_AssertStatus = XIL_ASSERT_NONE;     \
        } else {                                    \
            Xil_AssertStatus = XIL_ASSERT_OCCURRED; \
            return;                                 \
        }                                           \
    }

#define Xil_AssertVoidAlways()                  \
    {                                           \
        Xil_AssertStatus = XIL_ASSERT_OCCURRED; \
        return;                                 \
    }

#endif /* XIL_ASSERT_H */
