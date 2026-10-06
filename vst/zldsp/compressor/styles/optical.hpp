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
    class OpticalCompressor final {
    public:
        OpticalCompressor() =  default;

        template <typename F>
        static void reset(F& follower, const FloatType v = FloatType(0)) {
            follower.reset(v);
        }

        template <typename C, typename F, PPState pp_state = PPState::kOff, SState s_state = SState::kOff>
        static void process(C& computer, F& follower,
                            FloatType* __restrict buffer, const size_t num_samples) {
            // pass through the follower
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = follower.template processSample<pp_state, s_state>(std::abs(buffer[i]));
            }
            // transfer to db
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = static_cast<FloatType>(chore::kLogMul) * std::log(
                    std::max(buffer[i], static_cast<FloatType>(chore::kLogMin)));
            }
            // pass through the computer
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = computer.eval(buffer[i]);
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
            {
                const auto mean_scale = FloatType(1) / static_cast<FloatType>(tracker.getCurrentBufferSize());
                size_t i = 0;
                for (; i < num_samples; ++i) {
                    FloatType x = buffer[i];
                    x = x * mean_scale;
                    x = std::sqrt(x);
                    buffer[i] = x;
                }
            }
            // pass through the follower
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = follower.template processSample<pp_state, s_state>(buffer[i]);
            }
            // transfer to db
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = static_cast<FloatType>(chore::kLogMul) * std::log(
                    std::max(buffer[i], static_cast<FloatType>(chore::kLogMin)));
            }
            // pass through the computer
            for (size_t i = 0; i < num_samples; ++i) {
                buffer[i] = computer.eval(buffer[i]);
            }
        }
    };
}
