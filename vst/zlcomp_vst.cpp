/* =============================================================================
 * zlcomp_vst.cpp - ZL Comp: a master-bus compressor as a VST2 insert effect for the MPC OS plugin
 * host (Force, MPC Live/One/X/Key), armhf. The DSP is zlcomp_core.h on top of ZL Compressor's
 * compressor code (zldsp/, Copyright (C) 2026 zsliu98). This file is the plug-in around it,
 * hand-written like mpc-vst-rat's rat_vst.cpp (no wrapper, no SDK).
 * GNU Affero General Public License Version 3 (see ../LICENSE). Unofficial port, not affiliated
 * with ZL Audio.
 * ========================================================================== */
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "params.h"
#include "popup.h"    /* mpc-vst-plugins wrapper/popup.h, copied into build/ by build.sh */
#include "zlcomp_core.h"

/* ---- VST2 ABI (hand-written; no Steinberg SDK) ---------------------------- */
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
typedef struct { int32_t type, byteSize, deltaFrames, flags; char data[16]; } VstEvent;
typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    unsigned char midiData[4];
    char detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;
typedef struct { int32_t numEvents; intptr_t reserved; VstEvent *events[2]; } VstEvents;

enum {
    effOpen = 0, effClose = 1, effGetParamLabel = 6, effGetParamDisplay = 7, effGetParamName = 8,
    effSetSampleRate = 10, effSetBlockSize = 11, effMainsChanged = 12, effGetChunk = 23,
    effSetChunk = 24, effProcessEvents = 25, effCanBeAutomated = 26, effGetPlugCategory = 35,
    effGetEffectName = 45, effGetVendorString = 47, effGetProductString = 48,
    effGetVendorVersion = 49, effCanDo = 51, effGetVstVersion = 58,
};
enum { audioMasterAutomate = 0, audioMasterGetTime = 7, audioMasterUpdateDisplay = 42 };
enum { kVstTransportPlaying = 1 << 1, kVstPpqPosValid = 1 << 9, kVstTempoValid = 1 << 10, kVstTimeSigValid = 1 << 13 };
typedef struct {
    double samplePos, sampleRate, nanoSeconds, ppqPos, tempo, barStartPos, cycleStartPos, cycleEndPos;
    int32_t timeSigNumerator, timeSigDenominator, smpteOffset, smpteFrameRate, samplesToNextClock, flags;
} VstTimeInfo;
enum { effFlagsCanReplacing = 1 << 4, effFlagsProgramChunks = 1 << 5 };



struct Plugin {
    AEffect fx;
    audioMasterCallback master = nullptr;
    std::atomic<float> cache[NPARAMS];
    std::atomic<int> notify[NPARAMS];
    float open[NPARAMS] = {0};
    volatile int release[NPARAMS] = {0};
    std::atomic<bool> dirty{true};
    MasterComp dsp;
    MasterComp::Params p;
    std::atomic<int> preset{0};
    float sr = 44100;
    std::vector<uint8_t> chunk;
};

static int IDX_STYLE, IDX_THRESH, IDX_RATIO, IDX_KNEE, IDX_ATTACK, IDX_RELEASE, IDX_SCHP, IDX_MIX,
           IDX_CLIP, IDX_OUTPUT, IDX_PRESET;

static int param_index(const char *key) {
    for (int i = 0; i < NPARAMS; i++) if (!std::strcmp(PARAMS[i].key, key)) return i;
    return -1;
}
static float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static void copy_str(void *dst, const char *s, size_t max) {
    std::strncpy((char *)dst, s, max - 1);
    ((char *)dst)[max - 1] = 0;
}
static int norm_to_ui(const param_t *p, float n) {
    if (p->nopts) return (int)std::lround(clamp01(n) * (p->nopts - 1));
    return (int)std::lround(p->min + (p->max - p->min) * clamp01(n));
}
static float ui_to_norm(const param_t *p, double v) {
    if (p->nopts) return p->nopts > 1 ? clamp01((float)(v / (p->nopts - 1))) : 0.0f;
    return p->max > p->min ? clamp01((float)((v - p->min) / (p->max - p->min))) : 0.0f;
}
static int ui(Plugin *w, int i) { return i < 0 ? 0 : norm_to_ui(&PARAMS[i], w->cache[i].load()); }
static void set_ui(Plugin *w, int i, int v) {
    if (i < 0) return;
    w->cache[i].store(ui_to_norm(&PARAMS[i], v));
    w->notify[i].store(1);
}

struct NoDenormals {
#if defined(__arm__) && defined(__ARM_FP)
    uint32_t old = 0;
    NoDenormals() { asm volatile("vmrs %0, fpscr" : "=r"(old)); asm volatile("vmsr fpscr, %0" : : "r"(old | (1u << 24))); }
    ~NoDenormals() { asm volatile("vmsr fpscr, %0" : : "r"(old)); }
#elif defined(__x86_64__) || defined(__i386__)
    unsigned old = __builtin_ia32_stmxcsr();
    NoDenormals() { __builtin_ia32_ldmxcsr(old | 0x8040); }
    ~NoDenormals() { __builtin_ia32_ldmxcsr(old); }
#endif
};

/* knob positions -> real values. THRESHOLD, RATIO, ATTACK, RELEASE, SC HP and OUTPUT read the
 * knob's full resolution (the number range the host shows is only its readout); ATTACK and RELEASE
 * are on ZL Compressor's scale, so settings carry over from the desktop plug-in. */
static float nrm(Plugin *w, int i) { return i < 0 ? 0.0f : w->cache[i].load(); }
static float thresh_db(float n) { return -60.0f + 60.0f * n; }                  /* -60 .. 0 dB */
static float ratio_of(float n) { return std::pow(20.0f, n); }                   /* 1 .. 20 */
static float attack_ms(float n) { return std::pow(1000.0f, n); }                /* 1 ms .. 1 s */
static float release_ms(float n) { return 10.0f * std::pow(500.0f, n); }        /* 10 ms .. 5 s */
static float schp_hz(float n) { return n < 0.005f ? 0.0f : 20.0f * std::pow(15.0f, n); }   /* off, 20 .. 300 Hz */
static float output_db(float n) { return -12.0f + 24.0f * n; }                  /* -12 .. +12 dB */
/* ... and back, for the presets */
static float thresh_n(float db) { return (db + 60.0f) / 60.0f; }
static float ratio_n(float r) { return std::log(r) / std::log(20.0f); }
static float attack_n(float ms) { return std::log(ms) / std::log(1000.0f); }
static float release_n(float ms) { return std::log(ms / 10.0f) / std::log(500.0f); }
static float schp_n(float hz) { return hz < 20.0f ? 0.0f : std::log(hz / 20.0f) / std::log(15.0f); }
static float output_n(float db) { return (db + 12.0f) / 24.0f; }

static void configure(Plugin *w) {
    MasterComp::Params &p = w->p;
    p.style = clampi(ui(w, IDX_STYLE), 0, 2);
    p.thresholdDb = thresh_db(nrm(w, IDX_THRESH));
    p.ratio = ratio_of(nrm(w, IDX_RATIO));
    p.kneeDb = (float)ui(w, IDX_KNEE);
    p.attackMs = attack_ms(nrm(w, IDX_ATTACK));
    p.releaseMs = release_ms(nrm(w, IDX_RELEASE));
    p.scHpHz = schp_hz(nrm(w, IDX_SCHP));
    p.mix = ui(w, IDX_MIX) / 100.0f;
    p.clip = (float)ui(w, IDX_CLIP);
    p.outputDb = output_db(nrm(w, IDX_OUTPUT));
}

static void set_norm(Plugin *w, int i, float n) { if (i >= 0) { w->cache[i].store(clamp01(n)); w->notify[i].store(1); } }

/* Starting points for a techno / Goa master, to be finished by ear: THRESHOLD depends on how hot
 * the mix arrives. Choosing one in the PRESET list sets the other knobs; the list entry itself is
 * only a selector and is not changed back when a knob is turned afterwards. */
struct Preset { int style; float thresh, ratio; int knee; float attack, release, schp; int mix; float output; int clip; };
static const Preset PRESETS[] = {
    /* INIT        */ {0, -12, 2.0f, 6, 100, 600, 90, 100, 0.0f, 0},
    /* GLUE SOFT   */ {0, -14, 1.5f, 10, 150, 800, 100, 100, 1.0f, 0},
    /* GLUE TECHNO */ {1, -12, 2.0f, 6, 180, 500, 120, 100, 1.5f, 10},
    /* GLUE GOA    */ {0, -12, 2.0f, 8, 120, 350, 140, 100, 1.5f, 10},
    /* PARALLEL    */ {0, -24, 6.0f, 4, 30, 400, 100, 40, 0.0f, 0},
    /* PUNCH       */ {1, -10, 4.0f, 2, 250, 300, 150, 100, 2.0f, 0},
    /* OPTO SMOOTH */ {2, -14, 2.5f, 12, 100, 1500, 80, 100, 1.0f, 0},
    /* LOUD        */ {0, -10, 3.0f, 6, 60, 300, 120, 100, 3.0f, 40},
};
static const int NPRESETS = (int)(sizeof PRESETS / sizeof PRESETS[0]);
static void apply_preset(Plugin *w, int k) {
    const Preset &q = PRESETS[clampi(k, 0, NPRESETS - 1)];
    set_ui(w, IDX_STYLE, q.style);
    set_norm(w, IDX_THRESH, thresh_n(q.thresh));
    set_norm(w, IDX_RATIO, ratio_n(q.ratio));
    set_ui(w, IDX_KNEE, q.knee);
    set_norm(w, IDX_ATTACK, attack_n(q.attack));
    set_norm(w, IDX_RELEASE, release_n(q.release));
    set_norm(w, IDX_SCHP, schp_n(q.schp));
    set_ui(w, IDX_MIX, q.mix);
    set_norm(w, IDX_OUTPUT, output_n(q.output));
    set_ui(w, IDX_CLIP, q.clip);
    w->dirty.store(true);
}

static void processReplacing(AEffect *e, float **in, float **out, int32_t n) {
    Plugin *w = (Plugin *)e->object;
    NoDenormals nd;
    if (w->dirty.exchange(false)) configure(w);
    if (n > 0) w->dsp.process(w->p, in[0], in[1], out[0], out[1], (uint32_t)n);
    bool any = false;
    for (int i = 0; i < NPARAMS; i++) {
        if (w->release[i]) { w->release[i] = 0; any = true; w->master(&w->fx, audioMasterAutomate, i, 0, 0, 0.0f); }
        if (!w->notify[i].exchange(0)) continue;
        any = true;
        w->master(&w->fx, audioMasterAutomate, i, 0, 0, w->cache[i].load());
    }
    if (any) w->master(&w->fx, audioMasterUpdateDisplay, 0, 0, 0, 0.0f);
}

static void setParameter(AEffect *e, int32_t i, float n) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return;
    const param_t *p = &PARAMS[i];
    if (popup_set(w->open, i, n)) return;
    bool nudge = false;
    if (p->nopts > 1) {
        float pos = clamp01(n) * (p->nopts - 1);
        if (std::fabs(pos - std::round(pos)) > 0.001f) {
            float cur = w->cache[i].load() * (p->nopts - 1);
            n = (float)clampi((int)std::lround(cur) + (pos > cur ? 1 : -1), 0, p->nopts - 1) / (p->nopts - 1);
            nudge = true;
        }
    }
    w->cache[i].store(clamp01(n));
    w->dirty.store(true);
    if (i == IDX_PRESET) {   /* a new entry in the PRESET list sets the knobs */
        const int k = clampi(norm_to_ui(p, w->cache[i].load()), 0, NPRESETS - 1);
        if (w->preset.exchange(k) != k) apply_preset(w, k);
    }
    if (!nudge) popup_picked(w->open, w->release, i);
}
static float getParameter(AEffect *e, int32_t i) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return 0.0f;
    if (popup_is(i)) return w->open[i];
    return w->cache[i].load();
}

/* The chunk keeps each knob's full position (x 100000, as an integer: no decimal point, so no
 * locale trouble), not the rounded readout, so THRESHOLD comes back exactly where it was. Loading
 * a chunk restores the knobs as saved; it does not re-apply the PRESET entry. */
static intptr_t get_chunk(Plugin *w, void **ptr) {
    std::string t = "ZLC1;";
    char buf[64];
    for (int i = 0; i < NPARAMS; i++) {
        std::snprintf(buf, sizeof buf, "%s=%ld;", PARAMS[i].key, std::lround(w->cache[i].load() * 100000.0f));
        t += buf;
    }
    w->chunk.assign(t.begin(), t.end());
    *ptr = w->chunk.data();
    return (intptr_t)w->chunk.size();
}
static intptr_t set_chunk(Plugin *w, const void *data, intptr_t len) {
    std::string t((const char *)data, (size_t)len);
    if (t.compare(0, 5, "ZLC1;")) return 0;
    for (size_t pos = 5; pos < t.size();) {
        size_t semi = t.find(';', pos);
        if (semi == std::string::npos) break;
        std::string kv = t.substr(pos, semi - pos);
        pos = semi + 1;
        size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        int i = param_index(kv.substr(0, eq).c_str());
        if (i < 0) continue;
        w->cache[i].store(clamp01(std::atol(kv.c_str() + eq + 1) / 100000.0f));
        w->notify[i].store(1);
    }
    if (IDX_PRESET >= 0) w->preset.store(clampi(ui(w, IDX_PRESET), 0, NPRESETS - 1));
    w->dirty.store(true);
    return 1;
}

static void format_value(Plugin *w, int idx, int u, float n, char *buf, size_t size) {
    const param_t *pp = &PARAMS[idx];
    if (pp->nopts) std::snprintf(buf, size, "%s", pp->opts[u]);
    else if (idx == IDX_THRESH) std::snprintf(buf, size, "%.1f dB", thresh_db(n));
    else if (idx == IDX_RATIO) std::snprintf(buf, size, "%.2f:1", ratio_of(n));
    else if (idx == IDX_KNEE) std::snprintf(buf, size, "%d dB", u);
    else if (idx == IDX_ATTACK) {
        const float ms = attack_ms(n);
        std::snprintf(buf, size, ms < 10 ? "%.1f ms" : "%.0f ms", ms);
    }
    else if (idx == IDX_RELEASE) std::snprintf(buf, size, "%.0f ms", release_ms(n));
    else if (idx == IDX_SCHP) {
        const float hz = schp_hz(n);
        if (hz <= 0) std::snprintf(buf, size, "OFF");
        else std::snprintf(buf, size, "%.0f Hz", hz);
    }
    else if (idx == IDX_OUTPUT) std::snprintf(buf, size, "%+.1f dB", output_db(n));
    else if (idx == IDX_MIX || idx == IDX_CLIP) std::snprintf(buf, size, "%d %%", u);
    else std::snprintf(buf, size, "%d", u);
    (void)w;
}

static intptr_t dispatcher(AEffect *e, int32_t op, int32_t idx, intptr_t v, void *p, float o) {
    Plugin *w = (Plugin *)e->object;
    switch (op) {
    case effOpen: return 1;
    case effClose: delete w; return 1;
    case effGetPlugCategory: return 1;   /* kPlugCategEffect */
    case effGetEffectName:
    case effGetProductString: copy_str(p, PLUG_NAME, 32); return 1;
    case effGetVendorString: copy_str(p, PLUG_VENDOR, 32); return 1;
    case effGetVendorVersion: return PLUG_VERSION;
    case effGetVstVersion: return 2400;
    case effCanBeAutomated: return idx >= 0 && idx < NPARAMS;
    case effGetParamName: if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].name, 32); return 1;
    case effGetParamLabel: if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].unit, 8); return 1;
    case effGetParamDisplay: {
        if (idx < 0 || idx >= NPARAMS) return 0;
        const bool pop = popup_is(idx);
        const float n = pop ? w->open[idx] : w->cache[idx].load();
        char buf[32];
        format_value(w, idx, norm_to_ui(&PARAMS[idx], n), n, buf, sizeof buf);
        copy_str(p, buf, 24);
        return 1;
    }
    case effSetSampleRate:
        if (o > 0) { w->sr = o; w->dsp.setSampleRate(o); w->dirty.store(true); }
        return 1;
    case effSetBlockSize: case effMainsChanged: return 1;
    case effCanDo: return -1;
    case effGetChunk: return get_chunk(w, (void **)p);
    case effSetChunk: return set_chunk(w, p, v);
    default: return 0;
    }
}

static void start_values(Plugin *w) {
    set_ui(w, IDX_PRESET, 0);
    w->preset.store(0);
    apply_preset(w, 0);   /* INIT */
}

extern "C" __attribute__((visibility("default"))) AEffect *VSTPluginMain(audioMasterCallback master) {
    static std::once_flag once;
    std::call_once(once, [] {
        IDX_STYLE = param_index("style"); IDX_THRESH = param_index("thresh"); IDX_RATIO = param_index("ratio");
        IDX_KNEE = param_index("knee"); IDX_ATTACK = param_index("attack"); IDX_RELEASE = param_index("release");
        IDX_SCHP = param_index("schp"); IDX_MIX = param_index("mix"); IDX_CLIP = param_index("clip");
        IDX_OUTPUT = param_index("output"); IDX_PRESET = param_index("preset");
    });
    Plugin *w = new Plugin();
    w->master = master;
    for (int i = 0; i < NPARAMS; i++) { w->cache[i].store(PARAMS[i].def); w->notify[i].store(0); }
    start_values(w);
    w->dsp.setSampleRate(w->sr);
    AEffect *e = &w->fx;
    std::memset(e, 0, sizeof *e);
    e->magic = 0x56737450;
    e->dispatcher = dispatcher;
    e->setParameter = setParameter;
    e->getParameter = getParameter;
    e->processReplacing = processReplacing;
    e->numParams = NPARAMS;
    e->numInputs = 2;
    e->numOutputs = 2;
    e->flags = effFlagsCanReplacing | effFlagsProgramChunks;
    e->uniqueID = PLUG_UID;
    e->version = PLUG_VERSION;
    e->object = w;
    return e;
}
