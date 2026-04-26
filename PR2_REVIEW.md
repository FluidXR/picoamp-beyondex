# PR #2 (Windows Underrun Fix) — Review

Branch: `test-pr2-windows-underrun`
Reviewed against: `main`
PR: https://github.com/FluidXR/picoamp-beyondex/pull/2

## High-priority issues

### 1. Integer overflow in the new feedback PI loop
`main.c` (in `tud_sof_cb`):
```c
int32_t i_q16 = (fb_i_us << 16) >> I_SHIFT;   // I_SHIFT = 18
```
With `fb_i_us` clamped to ±50000, `fb_i_us << 16` overflows `int32_t` (max safe value is 32767 before `<< 16` wraps). E.g. `fb_i_us = 50000`: the intent is `50000/4 = 12500`, but the actual computation produces `~ -3884` after the wrap + arithmetic shift. **Sign-flips and magnitude-collapses the integral term whenever the integrator crosses ±32767.**

The integrator is *designed* to saturate at ±50000 ("large enough to overcome host smoothing"), so this region is reachable in normal operation. Audio quality on a bad day will be unpredictable around heavy buffer drift.

Fix:
```c
int32_t i_q16 = (int32_t)(((int64_t)fb_i_us << 16) >> I_SHIFT);
```

**Status:** fixed (2026-04-26)

---

### 2. UAC1-style 9-byte endpoint descriptor on a UAC2 device
`usb_descriptors.h` adds:
```c
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_SYNC(_ep, _attr, ...) \
    TUD_STD_EP_DESC_LEN_9, TUSB_DESC_ENDPOINT, ..., 0x00, (_sync_epaddr)
```
…and replaces `TUD_AUDIO_DESC_STD_AS_ISO_EP` (7 bytes, UAC2) with the 9-byte version (`bRefresh + bSynchAddress`) at all three alt settings.

The 9-byte form is **UAC1**. UAC2 (which this device speaks) uses the standard 7-byte USB endpoint descriptor and puts sync info in the class-specific descriptor instead. Some hosts tolerate the larger descriptor; others don't. macOS is historically the strictest UAC2 implementation, so this could be an additional reason mac was flaky beyond the feedback-format bug. Worth verifying audio still works on mac after the format-correction fix lands; if it doesn't, this is the next thing to revert.

**Status:** verified working on Windows + macOS as-is (2026-04-26). Leaving the 9-byte form in place; revisit only if a future host rejects it.

---

### 3. `CFG_TUD_AUDIO_FUNC_1_N_AS_INT` changed 2 → 1
`tusb_config.h:158`. The descriptor only ever defined one AS interface (speaker, with 3 alt settings), so 1 is technically more correct than 2. But it's a behavior change in the audio class state machine. Low risk, just calling it out — TinyUSB allocates per-AS-interface state from this number.

**Status:** accepted as-is (2026-04-26). Note in merge commit.

---

### 4. Submodule bump not reviewed
`lib/pico-i2s-pio` jumped from main to `beyondex-buffer-improvements-1` (commit b05a6d3). The companion PR (FluidXR/pico-i2s-pio#2) likely changes the I2S ring buffer behavior the new feedback loop relies on. **I have not read that diff.** If we're going to merge this branch, that submodule needs its own review.

**Status:** API-surface review done (2026-04-26). New constants `I2S_TARGET_LEVEL_MIN_US` (12000) / `I2S_TARGET_LEVEL_MAX_US` (20000) are *required* by the new `tud_sof_cb` PI loop in main.c — bump is not optional. Buffer depth: 8 → 32. Levels: start 2 → 12, target 6 → 16. Empirically working on Win + Mac. Deeper review of 1900-line `i2s.c` rewrite deferred to a separate sweep.

---

## Medium-priority issues

### 5. Channel-swap default is now semantically inverted
We flipped `g_channel_swap = 0 → 1` to fix L/R. That works, but now:
- The flag's name ("swap") is the opposite of its physical meaning
- WebUSB UI's "Swap channels" toggle defaults to ON for correct output
- New unit boots with toggle on; toggling off plays backwards

Cleaner fix: leave the default at 0 and **invert the swap logic in `dsp/eq.h`** instead — so `g_channel_swap=0` means "no swap = correct" and the bit's name matches reality. That's a 1-line change in `eq.h:162` (`if (g_channel_swap)` → `if (!g_channel_swap)`).

**Status:** fixed (2026-04-26). Reverted `g_channel_swap` default to 0 and inverted `eq.h` logic. Bit semantics now: 0 = compensate for hardware-reversed L/R (the default), 1 = expose raw hardware ordering. Comment in `eq.h` documents the rationale.

---

### 6. `audio_diag_plot.py` removed, replaced with Windows-targeted C# tool
The Python tool was cross-platform via pyusb. The new `tools/diagnostics/Program.cs` is .NET — runnable on mac/Linux in theory, but the project file and the HID API usage will be Windows-flavored. You lose the macOS/Linux plotter you had.

If you actually use the diag stream from mac, this is a regression. If you only debug on Windows, fine.

**Status:** fixed (2026-04-26). Recreated `tools/audio_diag_plot.py` to read HID input reports via `hidapi` instead of vendor control. Also flipped `BEYONDEX_HID_DEBUG` from 0 to 1 in `usb_descriptors.h` since it was disabled — neither the C# tool nor the Python tool could have worked with it off. New tool plots: buf_us, counters (buf_len/underrun/overflow/sof_miss), feedback (frames/ms). Cross-platform: macOS/Linux/Windows, no driver setup. Install: `pip install hidapi matplotlib`.

---

### 7. Globals not `static`, not `volatile`
```c
uint32_t feedback;     // main.c
uint32_t sof_calls;
uint32_t sof_updates;
```
- Not `static` → leak into global namespace. `feedback` in particular is a generic name and could collide with anything in TinyUSB or the SDK.
- Written in `tud_sof_cb` (USB ISR context on rp2040), read in `hid_diag_task` (main loop) — should be `volatile` for correctness.

Fix: `static volatile uint32_t ...`. (`spk_streaming_active` and `host_is_windows` already follow this pattern, so it's just sloppiness.)

**Status:** fixed (2026-04-26). All three are now `static volatile uint32_t`.

---

## Low-priority / nits

### 8. IIR sentinel uses zero
```c
if (fb_avg_us == 0) fb_avg_us = us;
fb_avg_us = (fb_avg_us * 63 + us) >> 6;
```
If `i2s_get_buf_us()` legitimately returns 0 (empty buffer mid-stream), the smoother re-initializes every call instead of converging. Probably benign in practice but it's a bug-shaped pattern. A `static bool first` sentinel would be cleaner.

**Status:** fixed (2026-04-26). Replaced the `fb_avg_us == 0` check with a `fb_avg_primed` boolean. Reset alongside `fb_avg_us`/`fb_i_us` when streaming stops.

---

### 9. Dead store
```c
fb_last_q16 = fb_q16;
```
Set, never read. Drop the variable.

**Status:** fixed (2026-04-26). Variable removed.

---

### 10. `tud_sof_cb_enable(false); tud_sof_cb_enable(true);` in `tud_mount_cb`
Looks like a workaround for some TinyUSB state quirk, undocumented. Harmless. Worth a one-line comment so a future reader doesn't "clean it up."

**Status:** fixed (2026-04-26). Added a comment in `tud_mount_cb` explaining the bus-reset re-arm.

---

### 11. `host_is_windows` is sticky until unmount
If a Windows host enumerates, suspends, resumes — the flag stays set across the suspend/resume cycle (which is correct). If somehow the host changed without unmount, it'd be stale, but that's not a real scenario over USB. No action needed; just noting it's the intended behavior.

**Status:** intended behavior

---

## Summary

| # | Issue | Action |
|---|---|---|
| 1 | Integer overflow in PI loop | ✅ fixed |
| 2 | UAC1-style 9-byte EP descriptor on UAC2 | ✅ verified working on both OSes — leave as-is |
| 3 | N_AS_INT 2→1 | ✅ accepted |
| 4 | Submodule diff unreviewed | ✅ API-surface reviewed; deeper sweep deferred |
| 5 | Channel-swap inversion semantics | ✅ fixed (eq.h logic inverted, default reset to 0) |
| 6 | Lost Python diag tool | ✅ fixed (HID-based replacement) |
| 7 | Globals not static/volatile | ✅ fixed |
| 8 | IIR sentinel uses zero | ✅ fixed |
| 9 | Dead store `fb_last_q16` | ✅ fixed |
| 10 | Undocumented SOF toggle | ✅ fixed (commented) |
| 11 | `host_is_windows` sticky | ✅ intended |
