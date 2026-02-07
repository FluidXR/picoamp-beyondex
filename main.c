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
#include "hardware/uart.h"

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

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

//--------------------------------------------------------------------+
// ADAPTIVE CLOCK TRIM
// Auto-detect when USB feedback is not working (e.g. Windows) and
// enable device-side clock trimming only when needed.
//--------------------------------------------------------------------+
#if I2S_ENABLE_CLOCK_TRIM
static bool adaptive_trim_enabled = false;
static uint32_t adaptive_trim_startup_ms = 0;
static int32_t adaptive_trim_baseline_us = 0;
static uint32_t adaptive_trim_last_check_ms = 0;

// Detection thresholds:
// If buffer loses > DRAIN_THRESHOLD_US over DETECT_WINDOW_MS, enable trim
// If buffer is stable for STABLE_WINDOW_MS, disable trim
#define ADAPTIVE_STARTUP_DELAY_MS    5000   // Wait 5s after audio starts
#define ADAPTIVE_DETECT_WINDOW_MS    15000  // Check for drain over 15s
#define ADAPTIVE_STABLE_WINDOW_MS    30000  // Require 30s stability to disable
#define ADAPTIVE_DRAIN_THRESHOLD_US  1500   // > 1.5ms loss over 15s = feedback broken
#define ADAPTIVE_STABLE_THRESHOLD_US 1000   // < 1ms drift = feedback working
#endif

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

void led_blinking_task(void);
void audio_task(void);
void bootsel_task(void);

static void reboot_to_bootsel(void);

//-----------------------------------------------------------------------------
// Optional debug UART (does not use stdio; safe for USB audio timing).
//
// Connect a USB-UART dongle:
// - Pico GP0 (UART0 TX) -> dongle RX
// - Pico GND -> dongle GND
// 115200 8N1
//-----------------------------------------------------------------------------
#ifndef BEYONDEX_DEBUG_UART
#define BEYONDEX_DEBUG_UART 0
#endif

#if BEYONDEX_DEBUG_UART
#define DBG_UART_ID uart0
#define DBG_UART_BAUD 115200
#define DBG_UART_TX_PIN 0
#define DBG_UART_RX_PIN 1

static inline void dbg_uart_puts(const char *s) {
  while (*s) uart_putc_raw(DBG_UART_ID, *s++);
}

static void dbg_uart_put_u32(uint32_t v) {
  char buf[11];
  int i = 0;
  if (v == 0) {
    uart_putc_raw(DBG_UART_ID, '0');
    return;
  }
  while (v && i < (int)sizeof(buf)) {
    buf[i++] = (char)('0' + (v % 10));
    v /= 10;
  }
  while (i--) uart_putc_raw(DBG_UART_ID, buf[i]);
}

static void dbg_uart_put_i32(int32_t v) {
  if (v < 0) {
    uart_putc_raw(DBG_UART_ID, '-');
    // careful with INT32_MIN
    uint32_t uv = (uint32_t)(-(v + 1)) + 1u;
    dbg_uart_put_u32(uv);
  } else {
    dbg_uart_put_u32((uint32_t)v);
  }
}

static void debug_uart_init(void) {
  uart_init(DBG_UART_ID, DBG_UART_BAUD);
  gpio_set_function(DBG_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(DBG_UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(DBG_UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(DBG_UART_ID, false);
}

static void debug_uart_task(void) {
  static uint32_t last_ms = 0;
  uint32_t now = board_millis();
  if (now - last_ms < 1000) return;
  last_ms += 1000;

  dbg_uart_puts("buf_len=");
  dbg_uart_put_i32(i2s_get_buf_length());
  dbg_uart_puts(" buf_us=");
  dbg_uart_put_i32(i2s_get_buf_us());
  dbg_uart_puts(" underrun=");
  dbg_uart_put_u32(i2s_dbg_get_underrun_count());
  dbg_uart_puts(" overflow=");
  dbg_uart_put_u32(i2s_dbg_get_overflow_count());
  dbg_uart_puts("\r\n");
}
#endif

// Vendor control request: reboot into BOOTSEL (ROM USB boot)
// bmRequestType: 0x40 (Host-to-device | Vendor | Device)
#define BEYONDEX_USB_REQ_BOOTSEL   0x42
#define BEYONDEX_USB_BOOTSEL_MAGIC 0xB007

// Vendor control request: read audio debug stats
// bmRequestType: 0xC0 (Device-to-host | Vendor | Device)
#define BEYONDEX_USB_REQ_AUDIO_DIAG 0x43

typedef struct __attribute__((packed)) {
  uint32_t magic;     // 'BEXD' = 0x44584542
  uint32_t underrun;  // i2s_dbg_get_underrun_count()
  uint32_t overflow;  // i2s_dbg_get_overflow_count()
  int32_t  buf_len;   // i2s_get_buf_length()
  int32_t  buf_us;    // i2s_get_buf_us()
} beyondex_audio_diag_t;

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

#if BEYONDEX_DEBUG_UART
  debug_uart_init();
#endif

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

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

#if I2S_ENABLE_CLOCK_TRIM
    // Adaptive clock trim: Auto-detect when USB feedback isn't working
    // (e.g. Windows) and enable device-side clock trim only when needed.
    // This prevents competing control loops on hosts where feedback works (macOS).

    uint32_t curr_ms = board_millis();
    int32_t curr_buf_us = i2s_get_buf_us();

    // Wait for startup stabilization
    if (adaptive_trim_startup_ms == 0 && curr_buf_us > 0) {
      adaptive_trim_startup_ms = curr_ms;
      adaptive_trim_baseline_us = curr_buf_us;
      adaptive_trim_last_check_ms = curr_ms;
    }

    if (adaptive_trim_startup_ms > 0 &&
        (curr_ms - adaptive_trim_startup_ms) > ADAPTIVE_STARTUP_DELAY_MS) {

      uint32_t elapsed_ms = curr_ms - adaptive_trim_last_check_ms;

      if (!adaptive_trim_enabled && elapsed_ms >= ADAPTIVE_DETECT_WINDOW_MS) {
        // Check if buffer is draining (feedback not working)
        int32_t buffer_change = curr_buf_us - adaptive_trim_baseline_us;
        if (buffer_change < -ADAPTIVE_DRAIN_THRESHOLD_US) {
          // Buffer is draining! Enable clock trim
          adaptive_trim_enabled = true;
          adaptive_trim_baseline_us = curr_buf_us;
          adaptive_trim_last_check_ms = curr_ms;
        } else {
          // Reset baseline for next check
          adaptive_trim_baseline_us = curr_buf_us;
          adaptive_trim_last_check_ms = curr_ms;
        }
      } else if (adaptive_trim_enabled && elapsed_ms >= ADAPTIVE_STABLE_WINDOW_MS) {
        // Check if buffer is stable (feedback working again?)
        int32_t buffer_change = curr_buf_us - adaptive_trim_baseline_us;
        if (buffer_change > -ADAPTIVE_STABLE_THRESHOLD_US &&
            buffer_change < ADAPTIVE_STABLE_THRESHOLD_US) {
          // Buffer is stable! Disable clock trim
          adaptive_trim_enabled = false;
          adaptive_trim_baseline_us = curr_buf_us;
          adaptive_trim_last_check_ms = curr_ms;
        } else {
          // Reset baseline for next check
          adaptive_trim_baseline_us = curr_buf_us;
          adaptive_trim_last_check_ms = curr_ms;
        }
      }
    }

    // Only call clock trim if adaptively enabled
    if (adaptive_trim_enabled) {
      i2s_clock_trim();
    }
#endif
    led_blinking_task();
    bootsel_task();
#if BEYONDEX_DEBUG_UART
    debug_uart_task();
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
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
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

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
      blink_interval_ms = BLINK_STREAMING;

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

    //1ms間隔でフィードバック
    static uint32_t start_ms = 0;
    uint32_t curr_ms = board_millis();
    if (curr_ms - start_ms >= 1)
    {
      int32_t length_us = i2s_get_buf_us();
      static int32_t avg_length_us = 0;

      if (avg_length_us == 0) avg_length_us = length_us;
      
      // Low pass filter to smooth out jitter from USB packet arrival timing
      avg_length_us = (avg_length_us * 63 + length_us) >> 6; 

      // IMPORTANT: TinyUSB expects feedback in 16.16 when
      // CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION=1 (it converts to 10.14 on FS).
      // See tinyusb `tud_audio_n_fb_set()` docs in `audio_device.h`.
      //
      // Keep feedback within UAC2 FMT-2.0 section 2.3.1.1 limits:
      // deviation from nominal packet size must not exceed +/- one audio slot.
      // In practice, sending out-of-range feedback can cause some hosts to
      // ignore feedback and revert to nominal pacing (leading to slow drift).
      uint32_t min_feedback = (current_sample_rate / 1000 - 1) << 16;
      uint32_t max_feedback = (current_sample_rate / 1000 + 1) << 16;
      uint32_t feedback_range = 2 << 16;

      // remap max-min target to min-max feedback
      int32_t feedback = I2S_TARGET_LEVEL_MAX_US - avg_length_us;
      feedback *= feedback_range;
      feedback /= (I2S_TARGET_LEVEL_MAX_US - I2S_TARGET_LEVEL_MIN_US);
      feedback += min_feedback;

      if (feedback < min_feedback)
        feedback = min_feedback;
      else if (feedback > max_feedback)
        feedback = max_feedback;

      tud_audio_fb_set(feedback);
      start_ms += 1;
    }

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
