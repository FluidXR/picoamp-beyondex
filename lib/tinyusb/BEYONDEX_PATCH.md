# Vendored TinyUSB for Beyondex firmware

This directory is a vendored copy of [hathach/tinyusb](https://github.com/hathach/tinyusb)
**v0.17.0** plus one local patch.

It is selected via `PICO_TINYUSB_PATH` in the root `CMakeLists.txt` so the
pico-sdk's TinyUSB build picks up this tree instead of the one bundled with the
SDK at `${PICO_SDK_PATH}/lib/tinyusb`.

## Why vendor instead of using the SDK's copy

Two stacked problems forced this:

1. **SDK 2.1.1 ships TinyUSB 0.18.0**, which breaks Windows USB Audio playback
   (see project commit `4b6f971`). We pin to 0.17.0.

2. **TinyUSB 0.17.0's RP2040 DCD has a DPRAM-leak bug** (upstream issues
   [#628](https://github.com/hathach/tinyusb/issues/628) and
   [#1232](https://github.com/hathach/tinyusb/issues/1232); fix in
   [PR #1802](https://github.com/hathach/tinyusb/pull/1802)). `_hw_endpoint_alloc`
   is a bump allocator and `_hw_endpoint_close` only reclaims when **all**
   non-control endpoints are closed. Beyondex has an always-open WinUSB vendor
   interface (MI_02) plus isochronous audio endpoints that cycle on every
   Windows audio session open/close. Each cycle leaks DPRAM; after a few cycles
   `hard_assert(...)` panics the CPU, freezing the main loop. Symptom to the
   user: "device visible in PnP but plays no sound, replug-only fix" or
   eventually "device disappears from audio device list" once Windows gives up
   on USBAUDIO.

PR #1802 hasn't shipped in a tagged release we can use without losing the
0.18.0 audio fix's regression. So we backport it into our pinned 0.17.0 here.

## The patch

Only `src/portable/raspberrypi/rp2040/dcd_rp2040.c` is modified. Search for
`PR #1802` in that file to find the touched sections. Summary:

- `next_buffer_ptr` (uint8_t* bump pointer) → `dpram_state` (uint64_t bitmap, 64
  blocks × 64 bytes).
- `_hw_endpoint_alloc` does first-fit search through the bitmap instead of bumping.
- `_hw_endpoint_close` clears its own bits instead of waiting for an all-EPs-closed
  reclaim.
- One off-by-one bug from the upstream PR (`hard_assert(start_block > 0)` would
  false-positive if a free range began at block 0) is replaced with an explicit
  `found` flag.

The CMakeLists in the project root verifies the patch is present at configure
time and refuses to build if not — without it the firmware compiles fine but
wedges on Windows audio session churn.

## Regression test

`tools/stress_audio_cycle.py --cycles 20` reproduces the wedge on unpatched
firmware within 3 cycles. On the patched firmware it runs all 20 cycles
without a wedge. Any change to this directory should be re-validated against
that script.

## Future cleanup

When upstream TinyUSB ships a tagged release that contains both PR #1802 and a
fix for the Windows USB Audio regression that drove us to pin 0.17.0, this
vendored copy can be removed and the SDK's bundled TinyUSB used directly.
