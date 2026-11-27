#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# Makes FBoW (https://github.com/rmsalinas/fbow, MIT/BSD-2) build on non-x86 CPUs (Jetson, ARM).
#
# Upstream guards its x86 SIMD code (immintrin.h, __m128/__m256, cpuid) with `#ifndef __ANDROID__`
# only, so on aarch64 Linux the build fails. Defining __ANDROID__ globally would also change the
# behaviour of other headers (OpenCV), so instead we rename the guard to FBOW_NO_X86_SIMD and
# define it automatically on non-x86 targets. x86 keeps the fast SIMD paths; other CPUs use
# FBoW's existing portable (generic) code path.
#
# Usage: fbow_portable_simd.sh <fbow source dir>
set -eu
src="$1/src"
for f in "$src/fbow.h" "$src/cpu.h"; do
  sed -i 's/__ANDROID__/FBOW_NO_X86_SIMD/g' "$f"
  # Prepend the automatic definition right after the first line of the file.
  sed -i '1a\
#if !defined(__x86_64__) \&\& !defined(__i386__) \&\& !defined(FBOW_NO_X86_SIMD)\
#define FBOW_NO_X86_SIMD\
#endif' "$f"
done
grep -q "FBOW_NO_X86_SIMD" "$src/fbow.h" "$src/cpu.h"
echo "fbow: portable SIMD guard applied"
