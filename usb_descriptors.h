// Beyondex USB descriptors — selects the endpoint layout at compile time.
// BEYONDEX_SYNC_MODE is a -D compile definition (CMake option), so this switch is
// valid wherever this header is included (tusb_config.h includes it before
// CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP is defined, which is why the switch must not
// depend on that macro).
#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_
#ifdef BEYONDEX_SYNC_MODE
#include "usb_descriptors_sync.h"
#else
#include "usb_descriptors_async.h"
#endif
#endif
