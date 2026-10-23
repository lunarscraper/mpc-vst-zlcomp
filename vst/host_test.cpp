/* Offline x86 test of zlcomp_vst.cpp (test.sh builds it with ASan/UBSan): unity when RATIO is 1 or
 * MIX is 0, each style compressing by about what the curve says, RATIO, the attack letting a
 * transient through, the side-chain high-pass, linked stereo, OUTPUT, CLIP, the PRESET list, chunk
 * restore, NaN/denormal-free output, no UpdateDisplay on host writes, the presets as VST programs. With "bench" as the second argument it only measures CPU load.
 * Prints PASSED/FAILED. */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <dlfcn.h>

struct AEffect;
typedef intptr_t (*audioMasterCallback)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
struct AEffect {
    int32_t magic;
    intptr_t (*dispatcher)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
    void (*process)(AEffect *, float **, float **, int32_t);
    void (*setParameter)(AEffect *, int32_t, float);
    float (*getParameter)(AEffect *, int32_t);
    int32_t numPrograms, numParams, numInputs, numOutputs, flags;
    intptr_t resvd1, resvd2;
    int32_t initialDelay, realQualities, offQualities;
    float ioRatio;
    void *object, *user;
    int32_t uniqueID, version;
    void (*processReplacing)(AEffect *, float **, float **, int32_t);
    void (*processDoubleReplacing)(AEffect *, double **, double **, int32_t);
    char future[56];
};
static int automated = 0, updates = 0;
static intptr_t master(AEffect *, int32_t op, int32_t, intptr_t, void *, float) { if (op == 0) automated++; if (op == 42) updates++; return 0; }
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)
static const float SR = 44100;
static const int BS = 256;
static bool bad = false;

static int param(AEffect *e, const char *name) {
    char buf[64];
    for (int i = 0; i < e->numParams; i++) { buf[0] = 0; e->dispatcher(e, 8, i, 0, buf, 0); if (!std::strcmp(buf, name)) return i; }
    std::printf("FAIL: no parameter %s\n", name); fails++; return 0;
}
static std::string display(AEffect *e, const char *name) { char buf[64] = {0}; e->dispatcher(e, 7, param(e, name), 0, buf, 0); return buf; }
/* knobs by their real values (the same maps as in zlcomp_vst.cpp) */
static void setn(AEffect *e, const char *name, double n) { e->setParameter(e, param(e, name), (float)n); }
static void thresh(AEffect *e, double db) { setn(e, "Threshold", (db + 60) / 60); }
static void ratio(AEffect *e, double r) { setn(e, "Ratio", std::log(r) / std::log(20.0)); }
static void attack(AEffect *e, double ms) { setn(e, "Attack", std::log(ms) / std::log(1000.0)); }
static void release(AEffect *e, double ms) { setn(e, "Release", std::log(ms / 10) / std::log(500.0)); }
static void schp(AEffect *e, double hz) { setn(e, "SC HP", hz < 20 ? 0 : std::log(hz / 20) / std::log(15.0)); }
static void output(AEffect *e, double db) { setn(e, "Output", (db + 12) / 24); }
static void knee(AEffect *e, int db) { setn(e, "Knee", db / 32.0); }
static void mix(AEffect *e, int pc) { setn(e, "Mix", pc / 100.0); }
static void clip(AEffect *e, int pc) { setn(e, "Clip", pc / 100.0); }
static void style(AEffect *e, int s) { setn(e, "Style", s / 2.0); }
static void preset(AEffect *e, int k) { setn(e, "Preset", k / 7.0); }
/* a sine through the effect; returns the right channel's output (left gets silence unless both) */
static std::vector<float> run(AEffect *e, double f, double amp, double sec, std::vector<float> *left = nullptr, bool leftSilent = false) {
    std::vector<float> outR, il(BS), ir(BS), ol(BS), orr(BS);
    float *in[2] = {il.data(), ir.data()}, *out[2] = {ol.data(), orr.data()};
    static double ph = 0;
    for (int b = 0; b < (int)(sec * SR / BS); b++) {
        for (int i = 0; i < BS; i++) { float s = (float)(amp * std::sin(ph)); ph += 2 * M_PI * f / SR; il[i] = leftSilent ? 0 : s; ir[i] = s; }
        e->processReplacing(e, in, out, BS);
        for (int i = 0; i < BS; i++) {
            for (float s : {ol[i], orr[i]}) if (!std::isfinite(s) || std::fpclassify(s) == FP_SUBNORMAL) bad = true;
            outR.push_back(orr[i]);
            if (left) left->push_back(ol[i]);
        }
    }
    return outR;
}
static double goertzel(const std::vector<float> &a, double f, double t0) {
    size_t i0 = (size_t)(t0 * SR), i1 = a.size();
    double w = 2 * M_PI * f / SR, c = std::cos(w), s1 = 0, s2 = 0;
    for (size_t i = i0; i < i1; i++) { double s0 = a[i] + 2 * c * s1 - s2; s2 = s1; s1 = s0; }
    return 2 * std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - 2 * c * s1 * s2)) / (i1 - i0);
}
static double rms(const std::vector<float> &a, double t0 = 0.2) {
    double s = 0; size_t i0 = (size_t)(t0 * SR);
    for (size_t i = i0; i < a.size(); i++) s += (double)a[i] * a[i];
    return std::sqrt(s / (a.size() - i0));
}
/* total harmonic distortion of f0 (harmonics 2..10) */
static double thd(const std::vector<float> &a, double f0) {
    double h1 = goertzel(a, f0, 0.2), hs = 0;
    for (int k = 2; k <= 10; k++) { double h = goertzel(a, f0 * k, 0.2); hs += h * h; }
    return std::sqrt(hs) / std::max(h1, 1e-12);
}
static double db(double x) { return 20 * std::log10(x + 1e-12); }

/* no compression: RATIO 1, no side-chain filter, no clipper, unity */
static void neutral(AEffect *e) {
    style(e, 0); thresh(e, -20); ratio(e, 1); knee(e, 0); attack(e, 20); release(e, 200);
    schp(e, 0); mix(e, 100); clip(e, 0); output(e, 0);
}
static void silence(AEffect *e, double sec) { run(e, 100, 0.0, sec); }
static double gain_db(const std::vector<float> &a, double amp, double t0 = 0.5) { return db(rms(a, t0) / (amp / std::sqrt(2.0))); }

int main(int argc, char **argv) {
    void *h = dlopen(argc > 1 ? argv[1] : "./zlcomp.so", RTLD_NOW | RTLD_LOCAL);
    if (!h) { std::printf("FAILED: dlopen %s\n", dlerror()); return 1; }
    auto mainf = (AEffect * (*)(audioMasterCallback)) dlsym(h, "VSTPluginMain");
    AEffect *e = mainf(master);
    CHECK(e->numInputs == 2 && e->numOutputs == 2, "not a stereo effect");
    e->dispatcher(e, 0, 0, 0, nullptr, 0);
    e->dispatcher(e, 10, 0, 0, nullptr, SR);
    if (argc > 2 && !std::strcmp(argv[2], "bench")) {
        const char *sn[3] = {"CLEAN", "CLASSIC", "OPTICAL"};
        for (int s = 0; s < 3; s++) {
            style(e, s); thresh(e, -24); ratio(e, 4); schp(e, 120); clip(e, 50);
            auto t0 = std::chrono::steady_clock::now();
            run(e, 110, 0.5, 10.0);
            double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("bench (this CPU, one core): stereo, %s, side-chain filter and clipper on: %.2f %%\n", sn[s], 10.0 * sec);
        }
        return 0;
    }
    std::printf("defaults: %s, threshold %s, ratio %s, knee %s, attack %s, release %s, sc hp %s, mix %s, clip %s, output %s\n",
                display(e, "Style").c_str(), display(e, "Threshold").c_str(), display(e, "Ratio").c_str(), display(e, "Knee").c_str(),
                display(e, "Attack").c_str(), display(e, "Release").c_str(), display(e, "SC HP").c_str(), display(e, "Mix").c_str(),
                display(e, "Clip").c_str(), display(e, "Output").c_str());

    /* 1. RATIO 1: clean and at unity */
    neutral(e);
    {
        auto a = run(e, 220, 0.5, 1.0);
        double t = thd(a, 220), g = gain_db(a, 0.5);
        std::printf("  ratio 1: THD %.3f %%, gain %+.3f dB\n", 100 * t, g);
        CHECK(t < 0.005, "ratio 1 is not clean (%.3f %%)", 100 * t);   /* 0.2 % is this measurement's floor */
        CHECK(std::fabs(g) < 0.05, "ratio 1 is not at unity (%+.2f dB)", g);
    }

    /* 2. each style: a -6 dBFS tone, threshold -20 dB, 4:1, hard knee. The curve says 14 dB over
     *    the threshold come out as 3.5, so about -10.5 dB (CLASSIC is feedback: less). */
    const char *sn[3] = {"CLEAN", "CLASSIC", "OPTICAL"};
    ratio(e, 4);
    for (int s = 0; s < 3; s++) {
        style(e, s);
        silence(e, 0.3);
        auto a = run(e, 1000, 0.5, 1.5);
        double g = gain_db(a, 0.5, 1.0), t = thd(a, 1000);
        std::printf("  style %-7s: gain %+.2f dB, THD %.2f %%\n", sn[s], g, 100 * t);
        CHECK(display(e, "Style") == sn[s], "style readout %s", display(e, "Style").c_str());
        CHECK(g < -4.0 && g > -12.5, "style %s: gain %+.2f dB", sn[s], g);
        if (s != 1) CHECK(std::fabs(g + 10.5) < 1.5, "style %s is off the curve (%+.2f dB)", sn[s], g);
    }

    /* 3. RATIO: more ratio, more reduction; a soft knee starts earlier */
    style(e, 0);
    {
        double g[3];
        ratio(e, 2); g[0] = gain_db(run(e, 1000, 0.5, 1.0), 0.5);
        ratio(e, 8); g[1] = gain_db(run(e, 1000, 0.5, 1.0), 0.5);
        ratio(e, 2); thresh(e, -6); knee(e, 0); double k0 = gain_db(run(e, 1000, 0.25, 1.0), 0.25);
        knee(e, 16); g[2] = gain_db(run(e, 1000, 0.25, 1.0), 0.25);
        std::printf("  ratio: %+.2f dB (2:1), %+.2f dB (8:1); knee 16 dB below the threshold: %+.2f dB (hard: %+.2f)\n", g[0], g[1], g[2], k0);
        CHECK(g[1] < g[0] - 3, "RATIO does not compress harder");
        CHECK(std::fabs(k0) < 0.1 && g[2] < -0.3, "KNEE does not soften the curve");
        knee(e, 0); thresh(e, -20); ratio(e, 4);
    }

    /* 4. ATTACK: after silence the start of a tone gets through before the gain comes down */
    for (int k = 0; k < 2; k++) {
        attack(e, k ? 300 : 1); release(e, 200);
        silence(e, 1.0);
        auto a = run(e, 1000, 0.5, 1.0);
        std::vector<float> early(a.begin(), a.begin() + (size_t)(0.004 * SR));
        double ea = rms(early, 0), la = rms(a, 0.7);
        std::printf("  attack %s: first 4 ms %+.1f dB above the settled level\n", display(e, "Attack").c_str(), db(ea / la));
        if (k) CHECK(ea > la * 1.7, "a slow ATTACK does not let the transient through");
        else CHECK(ea < la * 1.6, "a fast ATTACK is not fast");
    }
    attack(e, 20);

    /* 5. SC HP: with the high-pass a 50 Hz tone no longer drives the compressor */
    {
        double g[2];
        for (int k = 0; k < 2; k++) { schp(e, k ? 300 : 0); silence(e, 0.3); g[k] = gain_db(run(e, 50, 0.5, 1.5), 0.5, 1.0); }
        std::printf("  sc hp: 50 Hz gets %+.2f dB (off), %+.2f dB (%s)\n", g[0], g[1], display(e, "SC HP").c_str());
        CHECK(g[0] < -6 && g[1] > -1.0, "SC HP does not keep the bass out of the detector");
        schp(e, 0);
        CHECK(display(e, "SC HP") == "OFF", "SC HP readout %s", display(e, "SC HP").c_str());
    }

    /* 6. stereo: linked (same gain on both channels), and silence in stays silence out */
    {
        std::vector<float> L;
        silence(e, 0.3);
        auto R = run(e, 1000, 0.5, 1.0, &L);
        double d = 0;
        for (size_t i = 0; i < R.size(); i++) d = std::max(d, (double)std::fabs(L[i] - R[i]));
        CHECK(d < 1e-6, "the channels get different gains (%.6f)", d);
        L.clear();
        silence(e, 0.3);
        R = run(e, 1000, 0.5, 1.0, &L, true);
        double gr = gain_db(R, 0.5);
        std::printf("  stereo: L-R difference %.7f; right only: left rms %.6f, right %+.2f dB (link: half the reduction)\n", d, rms(L, 0.3), gr);
        CHECK(rms(L, 0.3) < 1e-6, "a silent channel is not silent");
        CHECK(gr < -3.0 && gr > -7.5, "one-sided signal: gain %+.2f dB", gr);
    }

    /* 7. MIX 0 is the dry signal; OUTPUT is plain gain */
    {
        mix(e, 0);
        double g0 = gain_db(run(e, 1000, 0.5, 1.0), 0.5);
        mix(e, 50);
        double g50 = gain_db(run(e, 1000, 0.5, 1.0), 0.5);
        mix(e, 100); ratio(e, 1); output(e, 6);
        double g6 = gain_db(run(e, 1000, 0.1, 1.0), 0.1);
        std::printf("  mix 0: %+.3f dB, mix 50: %+.2f dB; output %s: %+.2f dB\n", g0, g50, display(e, "Output").c_str(), g6);
        CHECK(std::fabs(g0) < 0.05, "MIX 0 is not dry");
        CHECK(g50 < -1.5 && g50 > -8, "MIX 50 is not in between");
        CHECK(std::fabs(g6 - 6.0) < 0.1, "OUTPUT +6 dB gives %+.2f dB", g6);
        output(e, 0);
    }

    /* 8. CLIP: off is clean, on rounds the peaks (harmonics, lower peak) */
    {
        thresh(e, -12); ratio(e, 4);
        double t[2], pk[2];
        for (int k = 0; k < 2; k++) {
            clip(e, k ? 100 : 0);
            silence(e, 0.3);
            auto a = run(e, 220, 0.9, 1.5);
            t[k] = thd(a, 220); pk[k] = 0;
            for (size_t i = (size_t)(1.0 * SR); i < a.size(); i++) pk[k] = std::max(pk[k], (double)std::fabs(a[i]));
        }
        std::printf("  clip: THD %.2f %% / peak %.3f (0), THD %.2f %% / peak %.3f (100)\n", 100 * t[0], pk[0], 100 * t[1], pk[1]);
        CHECK(t[1] > t[0] + 0.005 && pk[1] < pk[0], "CLIP does not clip");
        clip(e, 0);
    }

    /* 9. PRESET: choosing an entry sets the knobs and tells the host */
    {
        const char *pn[8] = {"INIT", "GLUE SOFT", "GLUE TECHNO", "GLUE GOA", "PARALLEL", "PUNCH", "OPTO SMOOTH", "LOUD"};
        for (int k = 7; k >= 0; k--) {
            preset(e, k);
            automated = 0;
            auto a = run(e, 1000, 0.7, 0.5);
            std::printf("  preset %-11s: %s, %s, %s, knee %s, att %s, rel %s, hp %s, mix %s, out %s, clip %s -> %+.2f dB on a -3 dBFS tone\n",
                        display(e, "Preset").c_str(), display(e, "Style").c_str(), display(e, "Threshold").c_str(), display(e, "Ratio").c_str(),
                        display(e, "Knee").c_str(), display(e, "Attack").c_str(), display(e, "Release").c_str(), display(e, "SC HP").c_str(),
                        display(e, "Mix").c_str(), display(e, "Output").c_str(), display(e, "Clip").c_str(), gain_db(a, 0.7, 0.3));
            CHECK(display(e, "Preset") == pn[k], "preset readout %s", display(e, "Preset").c_str());
            CHECK(automated >= 10, "preset %s: the host was not told about the new knob positions", pn[k]);
        }
        preset(e, 3);
        CHECK(display(e, "Release") == "350 ms" && display(e, "SC HP") == "140 Hz" && display(e, "Style") == "CLEAN",
              "GLUE GOA: %s, %s, %s", display(e, "Release").c_str(), display(e, "SC HP").c_str(), display(e, "Style").c_str());
    }

    /* 10. chunk: the knobs come back as saved (a tweak after the preset survives) */
    thresh(e, -17.3); style(e, 2); mix(e, 80);
    void *chunk = nullptr;
    intptr_t len = e->dispatcher(e, 23, 0, 0, &chunk, 0);
    std::vector<uint8_t> copy((uint8_t *)chunk, (uint8_t *)chunk + len);
    AEffect *e2 = mainf(master);
    e2->dispatcher(e2, 10, 0, 0, nullptr, SR);
    e2->dispatcher(e2, 24, 0, (intptr_t)copy.size(), copy.data(), 0);
    for (const char *k : {"Style", "Threshold", "Ratio", "Knee", "Attack", "Release", "SC HP", "Mix", "Clip", "Output", "Preset"})
        CHECK(display(e2, k) == display(e, k), "chunk: %s %s vs %s", k, display(e2, k).c_str(), display(e, k).c_str());
    CHECK(display(e2, "Threshold") == "-17.3 dB", "chunk: threshold %s", display(e2, "Threshold").c_str());
    {
        std::vector<float> il(BS, 0.0f), ol(BS), orr(BS);
        float *in[2] = {il.data(), il.data()}, *out[2] = {ol.data(), orr.data()};
        e2->processReplacing(e2, in, out, BS);
        CHECK(display(e2, "Threshold") == "-17.3 dB" && display(e2, "Style") == "OPTICAL", "the restored knobs do not survive the first block");
    }
    std::printf("  chunk %ld bytes: %s, %s, %s, mix %s, preset %s\n", (long)len, display(e2, "Style").c_str(), display(e2, "Threshold").c_str(),
                display(e2, "Release").c_str(), display(e2, "Mix").c_str(), display(e2, "Preset").c_str());
    e2->dispatcher(e2, 1, 0, 0, nullptr, 0);

    /* 11. in-place buffers (out == in), odd block sizes, silence */
    {
        preset(e, 2);
        std::vector<float> l(1000), r(1000);
        for (int b = 0; b < 40; b++) {
            for (int i = 0; i < 1000; i++) { l[i] = 0.8f * (float)std::sin(0.05 * i); r[i] = b < 20 ? l[i] : 0.0f; }
            float *io[2] = {l.data(), r.data()};
            e->processReplacing(e, io, io, b % 2 ? 1000 : 37);
            for (float s : l) if (!std::isfinite(s) || std::fabs(s) > 2.0f) bad = true;
        }
        silence(e, 2.0);
    }

    /* 12. MPC rules (mpc-vst-plugins 2026-10-21): no UpdateDisplay on host writes; one, after the block, on a preset;
     *     the presets as VST programs (PRESET menu), a re-pick of the current one ignored */
    {
        preset(e, 0);
        silence(e, 0.2);
        updates = 0;
        for (int k = 0; k < 30; k++) { thresh(e, -10 - k); ratio(e, 2 + k * 0.1); style(e, k % 3); run(e, 1000, 0.3, 0.01); }
        CHECK(updates == 0, "host writes sent %d UpdateDisplay", updates);
        preset(e, 5);
        CHECK(updates == 0, "no UpdateDisplay from inside setParameter");
        silence(e, 0.1);
        CHECK(updates == 1, "one UpdateDisplay after a preset, got %d", updates);
        char name[64] = {0};
        CHECK(e->numPrograms == 8, "8 programs, got %d", e->numPrograms);
        e->dispatcher(e, 29, 3, 0, name, 0);
        CHECK(!std::strcmp(name, "GLUE GOA"), "program 3 name '%s'", name);
        e->dispatcher(e, 2, 0, 3, nullptr, 0);
        CHECK(e->dispatcher(e, 3, 0, 0, nullptr, 0) == 3 && display(e, "Preset") == "GLUE GOA" && display(e, "Release") == "350 ms",
              "program 3: %s, %s", display(e, "Preset").c_str(), display(e, "Release").c_str());
        release(e, 900);
        e->dispatcher(e, 2, 0, 3, nullptr, 0);   /* same program again: ignored */
        CHECK(display(e, "Release") == "900 ms", "a re-pick must not reload: %s", display(e, "Release").c_str());
        silence(e, 0.1);
    }

    CHECK(!bad, "NaN, Inf or denormals in the output");
    e->dispatcher(e, 1, 0, 0, nullptr, 0);
    std::printf(fails ? "FAILED (%d)\n" : "PASSED\n", fails);
    return fails ? 1 : 0;
}
