// SPDX-License-Identifier: Apache-2.0
//
// kestrel/common/gtsam_compat.hpp — includes that moved between GTSAM releases.
//
// The fixed-lag smoothers live in gtsam_unstable up to GTSAM 4.2 and were promoted to the core
// library (gtsam/nonlinear) in 4.3. Including through this header keeps Kestrel building against
// either; CMake links gtsam_unstable when it exists.

#pragma once

#if __has_include(<gtsam/nonlinear/IncrementalFixedLagSmoother.h>)
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#else
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#endif
