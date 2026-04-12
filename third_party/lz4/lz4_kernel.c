/*++
Copyright: LZ4 is BSD-licensed — see lz4.c. This file adapts it for kernel mode.
--*/
#include <ntddk.h>

#define LZ4_STATIC_LINKING_ONLY_DISABLE_MEMORY_ALLOCATION 1
#define LZ4_FREESTANDING 1

#ifndef LZ4_memcpy
#define LZ4_memcpy(d, s, l) RtlCopyMemory((d), (s), (l))
#endif
#ifndef LZ4_memmove
#define LZ4_memmove(d, s, l) RtlMoveMemory((d), (s), (l))
#endif
#ifndef LZ4_memset
#define LZ4_memset(p, v, l) RtlFillMemory((p), (l), (UCHAR)(v))
#endif

#include "lz4.c"
