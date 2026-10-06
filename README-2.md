# mpc-vst-zlcomp

ZL Comp: a **master-bus compressor** as a VST2 **insert effect** for the Akai MPC OS plugin host
(Force, MPC Live/One/X/Key). An unofficial, reduced port of
[ZL Compressor](https://github.com/ZL-Audio/ZLCompressor) by zsliu98: the compressor itself (gain
computer, attack/release follower, the CLEAN / CLASSIC / OPTICAL styles, the tanh clipper) is
ZL Compressor's code; the plug-in shell, the side-chain high-pass, the parallel mix and the presets
are new. Made to glue a live techno / Goa mix together, set by ear.

Signal path (`vst/zlcomp_core.h`): side-chain high-pass -> style (detector, curve, follower) ->
stereo link -> gain -> MIX against the dry signal -> clipper -> OUTPUT. No look-ahead, no
oversampling: zero latency.

Tab **COMP**
- **THRESHOLD** (-60..0 dB), **RATIO** (1:1..20:1), **KNEE** (0..32 dB, ZL's knee width)
- **ATTACK** (1 ms..1 s), **RELEASE** (10 ms..5 s): on ZL Compressor's scale, so settings carry
  over from the desktop plug-in. They read long compared to other compressors (the time constant
  is about a sixth of the number): 100 ms here is a medium attack, not a slow one.
- **STYLE**: CLEAN (feed-forward, transparent), CLASSIC (feedback, rounder, grabs less),
  OPTICAL (the follower runs on the level, not on the gain: slower, smoother)

Tab **OUT**
- **SC HP** (OFF, 20..300 Hz): high-pass in front of the detector only, so the kick and the bass
  do not make the whole mix pump. The audio itself is not filtered.
- **MIX**: 0 = dry, 100 = compressed; in between is parallel compression
- **CLIP**: ZL's tanh clipper behind the compressor, 0 = off
- **OUTPUT** (-12..+12 dB): make-up gain. There is no hidden limiter behind it.
- **PRESET**: INIT, GLUE SOFT, GLUE TECHNO, GLUE GOA, PARALLEL, PUNCH, OPTO SMOOTH, LOUD.
  Choosing an entry sets the knobs; they are starting points, THRESHOLD depends on how hot the mix
  arrives. The project saves the knobs as they are.

Stereo is always linked (both channels get the same gain). There is no gain-reduction meter: the
MPC skin has no meters.

Left out of ZL Compressor: the editor and analyzers, external side-chain and side-chain EQ,
mid/side, expansion / inflation / shaping, VOCAL style, RMS detection, pump/punch/smooth, hold,
range, look-ahead, oversampling, LUFS matching.

## Build

`vst/build.sh` builds `vst/build/zlcomp.so` (armhf) and the skin, `vst/test.sh` runs the offline
test (unity at ratio 1 and mix 0, each style against the curve, attack, side-chain filter, linked
stereo, clipper, presets, project restore) and a benchmark (0.3..0.6 % of an x86 core). The
workflow under `.github/workflows` does both. Build/deploy: see `sd88me/mpc-vst-plugins`'
`docs/PORTING.md`. The compiler runs with `-std=c++20` (ZL's code needs it).

## License and changes

GNU Affero General Public License Version 3, see [LICENSE](LICENSE), as ZL Compressor.
ZL Compressor is Copyright (C) 2025-2026 zsliu98. The files under `vst/zldsp/` are taken from its
`source/dsp/`; changed are `compressor/styles/clean.hpp` and `optical.hpp` (the Highway SIMD loops
replaced by the files' own scalar loops) and `compressor/tracker/rms_tracker.hpp` (an unused
include removed); `tracker.hpp` and `follower.hpp` are one-line includes. Each changed file says so
at its top. The ZL Audio and ZL Compressor logos are not part of this repository. Not affiliated
with ZL Audio. The font is Titillium Web (SIL Open Font License, `vst/fonts/OFL.txt`).

## Hinweis

Entwickelt mit Unterstützung von Claude (Anthropic)
