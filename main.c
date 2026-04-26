/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2025 BambooMaster
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/**
 * @file main.c
 * @author BambooMaster (https://misskey.hakoniwa-project.com/@BambooMaster)
 * @brief pico_usb_i2s_speaker
 * @version 0.8
 * @date 2025-05-29
 * 
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "beyondex_diag.h"

#include "i2s.h"
#include "dsp/eq.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

// List of supported sample rates
const uint32_t sample_rates[] = {48000};

uint32_t current_sample_rate  = 48000;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

#define DEFAULT_VOLUME  -6 * 256 // -6 dB ~ 67%

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_STREAMING = 25,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

enum
{
  VOLUME_CTRL_0_DB = 0,
  VOLUME_CTRL_10_DB = 2560,
  VOLUME_CTRL_20_DB = 5120,
  VOLUME_CTRL_30_DB = 7680,
  VOLUME_CTRL_40_DB = 10240,
  VOLUME_CTRL_50_DB = 12800,
  VOLUME_CTRL_60_DB = 15360,
  VOLUME_CTRL_70_DB = 17920,
  VOLUME_CTRL_80_DB = 20480,
  VOLUME_CTRL_90_DB = 23040,
  VOLUME_CTRL_100_DB = 25600,
  VOLUME_CTRL_SILENCE = 0x8000,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Buffer for speaker data
uint8_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ];
// Speaker data size received in the last frame
int spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_3_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

static volatile bool spk_streaming_active;

// Set when the host requests the MS OS 2.0 descriptor set, which only Windows
// does. Used to pick the USB Audio feedback format: Windows' UAC2 driver has a
// known bug requiring 16.16 on full-speed (vs the spec'd 10.14), while macOS
// and Linux follow the spec.
static volatile bool host_is_windows;

static volatile uint32_t feedback;
static volatile uint32_t sof_calls;
static volatile uint32_t sof_updates;

void led_blinking_task(void);
void audio_task(void);
void bootsel_task(void);

static void reboot_to_bootsel(void);

#if BEYONDEX_HID_DEBUG
static void hid_diag_task(void);
#endif

// Vendor control request: reboot into BOOTSEL (ROM USB boot)
// bmRequestType: 0x40 (Host-to-device | Vendor | Device)
#define BEYONDEX_USB_REQ_BOOTSEL   0x42
#define BEYONDEX_USB_BOOTSEL_MAGIC 0xB007

// Vendor control request: read audio debug stats
// bmRequestType: 0xC0 (Device-to-host | Vendor | Device)
#define BEYONDEX_USB_REQ_AUDIO_DIAG 0x43

// Vendor control request: get/set channel swap
#define BEYONDEX_USB_REQ_CHANNEL_SWAP 0x44

//--------------------------------------------------------------------+
// PERSISTENT SETTINGS (last sector of flash)
//--------------------------------------------------------------------+
#define SETTINGS_MAGIC 0x42455844u  // 'BEXD'
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint8_t  channel_swap;   // 0 = normal, 1 = L/R swapped
  uint8_t  _reserved[251]; // pad to FLASH_PAGE_SIZE (256)
} beyondex_settings_t;

_Static_assert(sizeof(beyondex_settings_t) == FLASH_PAGE_SIZE,
               "settings struct must be exactly one flash page");

volatile uint8_t g_channel_swap = 0;

static void settings_load(void) {
  const beyondex_settings_t *s =
      (const beyondex_settings_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
  if (s->magic == SETTINGS_MAGIC) {
    g_channel_swap = s->channel_swap ? 1 : 0;
  }
}

static void settings_save(void) {
  beyondex_settings_t page;
  memset(&page, 0xFF, sizeof(page));
  page.magic        = SETTINGS_MAGIC;
  page.channel_swap = g_channel_swap ? 1 : 0;

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(SETTINGS_FLASH_OFFSET, (const uint8_t *)&page, FLASH_PAGE_SIZE);
  restore_interrupts(ints);
}

static volatile uint8_t g_bootsel_reboot_countdown = 0;

// WebUSB landing page URL descriptor
#define WEBUSB_URL  "updatebeyondex.fluid.so"

static const tusb_desc_webusb_url_t desc_url = {
  .bLength         = 3 + sizeof(WEBUSB_URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = WEBUSB_URL
};

/*------------- MAIN -------------*/
int main(void)
{
  //uartの設定よりも前に呼び出す
  i2s_mclk_set_config(pio0, 0, dma_claim_unused_channel(true), false, CLOCK_MODE_LOW_JITTER_OC, MODE_I2S);
  board_init();

  //i2s init
  i2s_mclk_set_pin(18, 16, 22);
  i2s_mclk_init(current_sample_rate);

  settings_load();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);
  tud_sof_cb_enable(true);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  TU_LOG1("Headset running\r\n");

  // set default mute and volume
  for (int i = 0; i < CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1; i++)
  {
    mute[i] = false;
    volume[i] = DEFAULT_VOLUME;
    audio_set_mute(mute[i], i);
    audio_set_volume(volume[i], i);
  }

  while (1)
  {
    tud_task(); // TinyUSB device task
    audio_task();
    led_blinking_task();
    bootsel_task();
#if BEYONDEX_HID_DEBUG
    hid_diag_task();
#endif
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  // Re-arm the SOF callback on every mount: TinyUSB clears its enable state on
  // bus reset, so toggling off→on guarantees the feedback loop runs after a
  // host-side disconnect/reconnect even though tud_init() already enabled it.
  tud_sof_cb_enable(false);
  tud_sof_cb_enable(true);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
  host_is_windows = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

      audio_control_cur_4_t curf = { (int32_t) tu_htole32(current_sample_rate) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
      {
        .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
      };
      TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
      for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
      {
        rangef.subrange[i].bMin = (int32_t) sample_rates[i];
        rangef.subrange[i].bMax = (int32_t) sample_rates[i];
        rangef.subrange[i].bRes = 0;
        TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
      }

      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

    current_sample_rate = (uint32_t) ((audio_control_cur_4_t const *)buf)->bCur;
    
    //サンプリングレート変更
    i2s_mclk_change_clock(current_sample_rate);

    TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

    return true;
  }
  else
  {
    TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
    TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
  }
  else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_2_n_t(1) range_vol = {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_100_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(1) }
      };
      TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
              range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

    mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

    // macOS typically uses master channel 0; apply to both channels.
    if (request->bChannelNumber == 0)
    {
      mute[1] = mute[0];
      mute[2] = mute[0];
      audio_set_mute(mute[0], 0);
    }
    else if (request->bChannelNumber == 1)
    {
      audio_set_mute(mute[1], 1);
    }
    else if (request->bChannelNumber == 2)
    {
      audio_set_mute(mute[2], 2);
    }
  
    TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

    return true;
  }
  else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

    volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

    //音量変更
    // macOS typically uses master channel 0; apply to both channels.
    if (request->bChannelNumber == 0)
    {
      volume[1] = volume[0];
      volume[2] = volume[0];
      audio_set_volume(volume[0], 0);
    }
    // Windows often uses channels 1/2 directly.
    else if (request->bChannelNumber == 1)
    {
      audio_set_volume(volume[1], 1);
    }
    else if (request->bChannelNumber == 2)
    {
      audio_set_volume(volume[2], 2);
    }

    TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

    return true;
  }
  else
  {
    TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_get_request(rhport, request);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_get_request(rhport, request);
  else
  {
    TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
  }
  return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_set_request(rhport, request, buf);
  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_set_request(rhport, request, buf);
  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;

  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
      blink_interval_ms = BLINK_MOUNTED;

  return true;
}

static int32_t fb_avg_us = 0;
static int32_t fb_i_us = 0;
static bool fb_avg_primed = false;

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : (v > hi) ? hi
                                  : v;
}

void tud_sof_cb(uint32_t frame_count)
{
  (void)frame_count;

  sof_calls++;

  // If stream not active, keep nominal and reset state.
  if (!spk_streaming_active)
  {
    fb_avg_us = 0;
    fb_i_us = 0;
    fb_avg_primed = false;
    return;
  }

  const int32_t min_us = I2S_TARGET_LEVEL_MIN_US;
  const int32_t max_us = I2S_TARGET_LEVEL_MAX_US;
  const int32_t mid_us = (min_us + max_us) / 2;
  const int32_t range_us = (max_us - min_us) > 1 ? (max_us - min_us) : 1;

  int32_t us = i2s_get_buf_us();

  // 1st-order IIR LPF to suppress packet jitter
  if (!fb_avg_primed) {
    fb_avg_us = us;
    fb_avg_primed = true;
  }
  fb_avg_us = (fb_avg_us * 63 + us) >> 6;

  // Positive == buffer below target == host must speed up (increase feedback)
  int32_t err_us = (mid_us - fb_avg_us);

  // Map full range_us error to +/- P_MAX_DELTA (in 16.16)
  const int32_t P_MAX_DELTA_Q16 = (1 << 14);
  int32_t p_q16 = (int32_t)(((int64_t)err_us * (int64_t)P_MAX_DELTA_Q16) / (int64_t)(range_us / 2));
  p_q16 = clamp_i32(p_q16, -P_MAX_DELTA_Q16, P_MAX_DELTA_Q16);

  // Integrate error (us per ms)
  // I_LIM_US sets the max accumulated error; large enough to overcome host smoothing.
  const int32_t I_LIM_US = 50000;
  fb_i_us = clamp_i32(fb_i_us + err_us, -I_LIM_US, I_LIM_US);

  // Convert integrated error to 16.16 delta with a small gain.
  // SHIFT controls integral gain: smaller == stronger.
  const int I_SHIFT = 18;
  // 64-bit intermediate: fb_i_us is clamped to ±50000 and `<< 16` would
  // overflow int32 above ±32767, sign-flipping the integral term.
  int32_t i_q16 = (int32_t)(((int64_t)fb_i_us << 16) >> I_SHIFT);
  // Clamp integral contribution tighter than full-scale to prevent hunting
  const int32_t I_MAX_DELTA_Q16 = (1 << 15); // 0.5 sample/ms
  i_q16 = clamp_i32(i_q16, -I_MAX_DELTA_Q16, I_MAX_DELTA_Q16);

  // Nominal 16.16 frames per ms
  const uint32_t nominal_q16 = (uint32_t)((((uint64_t)current_sample_rate) << 16) + 500) / 1000;

  const int32_t TOTAL_MAX_DELTA_Q16 = (1 << 16);
  int32_t delta_q16 = clamp_i32(p_q16 + i_q16, -TOTAL_MAX_DELTA_Q16, TOTAL_MAX_DELTA_Q16);

  int32_t fb_q16 = nominal_q16 + delta_q16;

  int32_t min_fb_q16 = nominal_q16 - (1 << 16);
  int32_t max_fb_q16 = nominal_q16 + (1 << 16);
  fb_q16 = clamp_i32(fb_q16, min_fb_q16, max_fb_q16);

  tud_audio_fb_set((uint32_t)fb_q16);
  sof_updates++;

  feedback = (uint32_t)fb_q16;
}

bool tud_audio_feedback_format_correction_cb(uint8_t func_id)
{
  (void)func_id;
  // On full-speed, USB 2.0 spec says feedback is 10.14. Windows' UAC2 driver
  // ignores that and wants 16.16 — so for Windows we send raw 16.16. Every
  // other host gets TinyUSB's automatic 16.16→10.14 conversion.
  return !host_is_windows;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  spk_streaming_active = ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0;

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (spk_streaming_active)
  {
    blink_interval_ms = BLINK_STREAMING;
    uint32_t nominal_fb = (current_sample_rate / 1000) << 16; // 16.16
    tud_audio_fb_set(nominal_fb);                             // prime feedback EP
  }

  // Clear buffer when streaming format is changed
  spk_data_size = 0;
  if(alt != 0)
  {
    current_resolution = resolutions_per_format[alt-1];
  }

  return true;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)func_id;
  (void)ep_out;
  (void)cur_alt_setting;

  spk_data_size = tud_audio_read(spk_buf, n_bytes_received);
  return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)itf;
  (void)ep_in;
  (void)cur_alt_setting;

  // This callback could be used to fill microphone data separately
  return true;
}

void tud_audio_fb_done_cb(uint8_t rhport)
{
  return;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
  if (spk_data_size)
  {
    eq_process(spk_buf, spk_data_size, current_resolution);
    spk_data_size = 0;
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return;
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state;
}

// Invoked when received a control request with VENDOR type.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  if (stage != CONTROL_STAGE_SETUP) return true;

  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) return false;

  switch (request->bRequest)
  {
    // ---- WebUSB landing page URL ----
    case VENDOR_REQUEST_WEBUSB:
      return tud_control_xfer(rhport, request, (void *)(uintptr_t)&desc_url, desc_url.bLength);

    // ---- MS OS 2.0 descriptor set (auto-installs WinUSB on Windows) ----
    case VENDOR_REQUEST_MICROSOFT:
      if (request->wIndex == 7)
      {
        host_is_windows = true;
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20 + 8, 2);
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20, total_len);
      }
      return false;

    // ---- Reboot into BOOTSEL (firmware update) ----
    case BEYONDEX_USB_REQ_BOOTSEL:
      if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
          request->wValue == BEYONDEX_USB_BOOTSEL_MAGIC &&
          request->wLength == 0)
      {
        g_bootsel_reboot_countdown = 2;
        return tud_control_status(rhport, request);
      }
      return false;

    // ---- Channel swap (get / set, persisted to flash) ----
    case BEYONDEX_USB_REQ_CHANNEL_SWAP:
      if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE)
      {
        if (request->bmRequestType_bit.direction == TUSB_DIR_IN)
        {
          static uint8_t swap_val;
          swap_val = g_channel_swap ? 1 : 0;
          return tud_control_xfer(rhport, request, &swap_val, 1);
        }
        else
        {
          g_channel_swap = request->wValue ? 1 : 0;
          settings_save();
          return tud_control_status(rhport, request);
        }
      }
      return false;

    // ---- Audio diagnostics ----
    case BEYONDEX_USB_REQ_AUDIO_DIAG:
      if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
          request->bmRequestType_bit.direction == TUSB_DIR_IN &&
          request->wLength == sizeof(beyondex_audio_diag_t))
      {
        static beyondex_audio_diag_t diag;
        diag.magic    = 0x44584542u; // 'BEXD'
        diag.underrun = i2s_dbg_get_underrun_count();
        diag.overflow = i2s_dbg_get_overflow_count();
        diag.buf_len  = i2s_get_buf_length();
        diag.buf_us   = i2s_get_buf_us();
        return tud_control_xfer(rhport, request, &diag, sizeof(diag));
      }
      return false;

    default:
      break;
  }

  return false;
}

//--------------------------------------------------------------------+
// BOOTSEL TASK
//--------------------------------------------------------------------+

void bootsel_task(void)
{
  // Software-triggered BOOTSEL (USB vendor request)
  if (g_bootsel_reboot_countdown)
  {
    g_bootsel_reboot_countdown--;
    if (g_bootsel_reboot_countdown == 0)
    {
      reboot_to_bootsel();
    }
  }

  // Hardware fallback: hold BOOTSEL/USER button for ~1.5s to enter BOOTSEL
  static uint32_t pressed_since_ms = 0;
  bool const pressed = board_button_read();
  if (pressed)
  {
    if (pressed_since_ms == 0)
    {
      pressed_since_ms = board_millis();
    }
    else if ((board_millis() - pressed_since_ms) > 1500)
    {
      reboot_to_bootsel();
    }
  }
  else
  {
    pressed_since_ms = 0;
  }
}

static void reboot_to_bootsel(void)
{
  // reset_usb_boot() is a ROM function (noreturn) that jumps directly into the USB bootloader.
  // Arguments:
  // - gpio_pin_mask: which GPIOs to monitor for USB activity (0 = default behaviour)
  // - disable_interface_mask: which boot interfaces to disable (0 = none)
  reset_usb_boot(0u, 0u);
  while (1) { tight_loop_contents(); }
}

#if BEYONDEX_HID_DEBUG
//--------------------------------------------------------------------+
// HID DEBUG TASK
//--------------------------------------------------------------------+

static void hid_diag_task(void)
{
  static uint32_t last_ms = 0;
  uint32_t now = board_millis();
  if (now - last_ms < 200)
    return;
  last_ms += 200;

  if (!tud_hid_ready())
    return;

  uint8_t p[sizeof(beyondex_audio_diag_t)];
  beyondex_get_audio_diag_payload(p, feedback, sof_calls, sof_updates, spk_streaming_active);

  tud_hid_report(0, p, sizeof(p));
}
#endif
