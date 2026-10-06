// Copyright (C) 2026 - zsliu98
// This file is part of ZLCompressor
//
// ZLCompressor is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License Version 3 as published by the Free Software Foundation.
//
// ZLCompressor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License along with ZLCompressor. If not, see <https://www.gnu.org/licenses/>.

// Modified for mpc-vst-zlcomp (2026): Highway SIMD paths replaced by the file's own scalar loops
// (same arithmetic), so the code builds without the Highway dependency.

#pragma once

#include "../../chore/decibels.hpp"
#include "../tracker/tracker.hpp"
#include "../follower/follower.hpp"

namespace zldsp::compressor {
    template <typename FloatType>
    class CleanCompressor final {
    public:
        CleanCompressor() = default;

        template <typename F>
        static void reset(F& follower, const FloatType v = FloatType(0)) {
            follower.reset(v);
        }

        template <typename C, typename F, PPState pp_state = PPState::kOff, SState s_state = SState::kOff>
        static void process(C& computer, F& follower,
                            FloatType* __restrict buffer, const size_t num_samples) {
            static constexpr auto kLogMin = static_cast<FloatType>(chore::kLogMin);
            static constexpr auto kLogMul = static_cast<FloatType>(chore::kLogMul);
            {
                size_t i = 0;
                for (; i < num_samples; ++i) {
                    FloatType x = buffer[i];
                    x = std::max(std::abs(x), kLogMin);
                    x = std::log(x) * kLogMul;
                    buffer[i] = x;
                }
            }
            // pass through the computer and the follower
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = -follower.template processSample<pp_state, s_state>(
                    -computer.eval(buffer[i]));
            }
        }

        template <typename C, typename F, PPState pp_state = PPState::kOff, SState s_state = SState::kOff>
        static void process(C& computer, F& follower, RMSTracker<FloatType>& tracker,
                            FloatType* __restrict buffer, const size_t num_samples) {
            // pass through the tracker
            for (size_t i = 0; i < num_samples; ++i) {
                tracker.processSample(buffer[i]);
                buffer[i] = tracker.getMomentarySquare();
            }
            // transfer square sum to db
            {
                static constexpr auto kLogSqrMin = static_cast<FloatType>(chore::kLogSqrMin);
                static constexpr auto kLogSqrMul = static_cast<FloatType>(chore::kLogSqrMul);
                const auto mean_scale = FloatType(1) / static_cast<FloatType>(tracker.getCurrentBufferSize());
                size_t i = 0;
                for (; i < num_samples; ++i) {
                    FloatType x = buffer[i];
                    x = x * mean_scale;
                    x = std::max(x, kLogSqrMin);
                    x = std::log(x) * kLogSqrMul;
                    buffer[i] = x;
                }
            }
            // pass through the computer and the follower
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = -follower.template processSample<pp_state, s_state>(
                    -computer.eval(buffer[i]));
            }
        }
    };
}
