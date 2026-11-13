#ifndef DCU_COMPILER_H
#define DCU_COMPILER_H

#ifdef __cplusplus
#define DCU_STATIC_ASSERT static_assert
#else
#define DCU_STATIC_ASSERT _Static_assert
#endif

#endif
