// zlcomp_core.h - the master-bus signal path of mpc-vst-zlcomp.
//
// Copyright (C) 2026 - lunarscraper. Built on the DSP of ZL Compressor, Copyright (C) 2026 - zsliu98
// (the unchanged or lightly changed files under zldsp/). The order of operations follows
// ZLCompressor's source/zlp/compress_controller.cpp, reduced to what a stereo master insert needs.
//
// This file is part of mpc-vst-zlcomp, free software under the GNU Affero General Public License
// Version 3 as published by the Free Software Foundation. It comes WITHOUT ANY WARRANTY; see ../LICENSE.
//
// Signal path, per channel pair:
//   side = input -> side-chain high-pass (optional) -> style (CLEAN / CLASSIC / OPTICAL: level
//   detection, gain computer, attack/release follower; all ZL Compressor) -> stereo link (both
//   channels get the same gain, so the image does not move) -> gain
//   main = input * gain -> MIX against the dry input -> tanh clipper (ZL Compressor) -> OUTPUT
// No look-ahead and no oversampling: zero latency, and cheap enough to sit on the master all night.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "zldsp/chore/decibels.hpp"
#include "zldsp/compressor/clipper/tanh_clipper.hpp"
#include "zldsp/compressor/computer/compression_computer.hpp"
#include "zldsp/compressor/follower/follower.hpp"
#include "zldsp/compressor/styles/classic.hpp"
#include "zldsp/compressor/styles/clean.hpp"
#include "zldsp/compressor/styles/optical.hpp"

class MasterComp {
public:
    enum Style { kClean = 0, kClassic = 1, kOptical = 2 };

    struct Params {
        int style = kClean;
        float thresholdDb = -12.0f;   // -60 .. 0
        float ratio = 2.0f;           // 1 .. 20
        float kneeDb = 6.0f;          // 0 .. 32 (ZL's knee width: the knee spans threshold +- this)
        float attackMs = 100.0f;      // ZL Compressor's scale (its time constant is this / 2 pi)
        float releaseMs = 600.0f;     // ditto
        float scHpHz = 90.0f;         // side-chain high-pass, 0 = off
        float mix = 1.0f;             // 0 = dry .. 1 = compressed
        float outputDb = 0.0f;
        float clip = 0.0f;            // 0 .. 100, ZL's clipper drive
    };

    void setSampleRate(float sr) {
        sr_ = sr > 1000.0f ? sr : 44100.0f;
        for (auto &f : follower_) f.prepare((double)sr_);
        smooth_ = 1.0f - std::exp(-1.0f / (0.02f * sr_));   // 20 ms for MIX and OUTPUT
        force_ = true;
        reset();
    }

    void reset() {
        for (auto &f : follower_) f.reset(0.0f);
        for (auto &c : classic_) c.reset(follower_[0], 0.0f);
        follower_[1].reset(0.0f);
        for (auto &h : hp_) h = Biquad{};
    }

    /** the gain reduction at the end of the last block, in dB (<= 0); for tests */
    float lastGainDb() const { return last_gain_db_; }

    void process(const Params &p, const float *inL, const float *inR, float *outL, float *outR, uint32_t n) {
        update(p);
        while (n > 0) {
            const uint32_t m = std::min<uint32_t>(n, kBlock);
            block(inL, inR, outL, outR, m);
            inL += m; inR += m; outL += m; outR += m; n -= m;
        }
    }

private:
    static constexpr uint32_t kBlock = 256;
    using Computer = zldsp::compressor::CompressionComputer<float, true>;
    using Follower = zldsp::compressor::PSFollower<float>;

    struct Biquad {
        float z1 = 0, z2 = 0;
        float run(float x, const float *c) {   // transposed direct form II
            const float y = c[0] * x + z1;
            z1 = c[1] * x - c[3] * y + z2;
            z2 = c[2] * x - c[4] * y;
            return y;
        }
    };

    float sr_ = 44100.0f, smooth_ = 0.001f;
    bool force_ = true;
    Params cur_{};
    Computer computer_{};
    std::array<Follower, 2> follower_{};
    std::array<zldsp::compressor::ClassicCompressor<float>, 2> classic_{};
    zldsp::compressor::TanhClipper<float> clipper_{};
    std::array<Biquad, 2> hp_{};
    float hp_c_[5] = {1, 0, 0, 0, 0};
    bool hp_on_ = false;
    float mix_ = 1.0f, out_ = 1.0f, mix_target_ = 1.0f, out_target_ = 1.0f;
    float last_gain_db_ = 0.0f;

    void update(const Params &p) {
        const bool f = force_;
        if (f || p.thresholdDb != cur_.thresholdDb) computer_.setThreshold(std::clamp(p.thresholdDb, -100.0f, 0.0f));
        if (f || p.ratio != cur_.ratio) computer_.setRatio(std::clamp(p.ratio, 1.0f, 100.0f));
        if (f || p.kneeDb != cur_.kneeDb) computer_.setKneeW(std::clamp(p.kneeDb, 0.0f, 32.0f));
        if (f) computer_.setCurve(0.0f);
        if (f || p.attackMs != cur_.attackMs) follower_[0].setAttack(p.attackMs);
        if (f || p.releaseMs != cur_.releaseMs) follower_[0].setRelease(p.releaseMs);
        if (f) { follower_[0].setPumpPunch(0.0f); follower_[0].setSmooth(0.0f); }
        if (f || p.clip != cur_.clip) clipper_.setWet(p.clip);
        if (f || p.style != cur_.style) {   // as in ZL: a style change starts from a clean state
            for (auto &fo : follower_) fo.reset(0.0f);
            for (auto &c : classic_) c.reset(follower_[0], 0.0f);
            follower_[1].reset(0.0f);
        }
        if (f || p.scHpHz != cur_.scHpHz) {
            hp_on_ = p.scHpHz >= 10.0f;
            if (hp_on_) {   // 2-pole Butterworth high-pass
                const float w = 2.0f * 3.14159265358979f * std::min(p.scHpHz, 0.45f * sr_) / sr_;
                const float cs = std::cos(w), al = std::sin(w) / (2.0f * 0.70710678f), a0 = 1.0f + al;
                hp_c_[0] = (1.0f + cs) * 0.5f / a0;
                hp_c_[1] = -(1.0f + cs) / a0;
                hp_c_[2] = hp_c_[0];
                hp_c_[3] = -2.0f * cs / a0;
                hp_c_[4] = (1.0f - al) / a0;
            } else {
                for (auto &h : hp_) h = Biquad{};
            }
        }
        mix_target_ = std::clamp(p.mix, 0.0f, 1.0f);
        out_target_ = zldsp::chore::decibelsToGain(std::clamp(p.outputDb, -60.0f, 24.0f));
        if (f) { mix_ = mix_target_; out_ = out_target_; }
        cur_ = p;
        force_ = false;
    }

    void block(const float *inL, const float *inR, float *outL, float *outR, const uint32_t n) {
        using zldsp::compressor::PPState;
        using zldsp::compressor::SState;
        float s0[kBlock], s1[kBlock];
        if (hp_on_) {
            for (uint32_t i = 0; i < n; i++) { s0[i] = hp_[0].run(inL[i], hp_c_); s1[i] = hp_[1].run(inR[i], hp_c_); }
        } else {
            std::copy(inL, inL + n, s0);
            std::copy(inR, inR + n, s1);
        }
        if (follower_[0].prepareBuffer()) follower_[1].copyFrom(follower_[0]);
        if (computer_.prepareBuffer()) clipper_.setReductionAtUnit(computer_.eval(0.0f));
        // side buffers: signal in, gain in dB (<= 0) out
        switch (cur_.style) {
        case kClassic:
            classic_[0].process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[0], s0, n);
            classic_[1].process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[1], s1, n);
            break;
        case kOptical:
            zldsp::compressor::OpticalCompressor<float>::process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[0], s0, n);
            zldsp::compressor::OpticalCompressor<float>::process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[1], s1, n);
            break;
        default:
            zldsp::compressor::CleanCompressor<float>::process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[0], s0, n);
            zldsp::compressor::CleanCompressor<float>::process<Computer, Follower, PPState::kOff, SState::kOff>(computer_, follower_[1], s1, n);
            break;
        }
        // stereo link (ZL's link at 100 %): both channels get the mean of the two gains
        static constexpr float kDbToLn = 0.11512925464970228f;   // ln(10) / 20
        float g_db = 0.0f;
        for (uint32_t i = 0; i < n; i++) {
            g_db = std::clamp(0.5f * (s0[i] + s1[i]), -240.0f, 40.0f);
            s0[i] = std::exp(g_db * kDbToLn);
        }
        last_gain_db_ = g_db;
        clipper_.prepareBuffer();
        const bool clip = clipper_.getIsON();
        for (uint32_t i = 0; i < n; i++) {
            mix_ += smooth_ * (mix_target_ - mix_);
            out_ += smooth_ * (out_target_ - out_);
            const float dl = inL[i], dr = inR[i], g = s0[i];
            float l = dl + mix_ * (dl * g - dl), r = dr + mix_ * (dr * g - dr);
            if (clip) { l = clipper_.processSample(l); r = clipper_.processSample(r); }
            outL[i] = l * out_;
            outR[i] = r * out_;
        }
    }
};
