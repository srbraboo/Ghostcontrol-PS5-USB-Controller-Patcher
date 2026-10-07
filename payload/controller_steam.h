#pragma once

#include <stdint.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>

#include "gc_types.h"

/* Valve Steam Controller 2 (Triton) wired USB device. */
#define STEAM_VID    0x28deu
#define STEAM_PID    0x1302u
#define STEAM_PUCK_PID 0x1304u
#define STEAM_PUCK_ALT_PID 0x1305u
#define STEAM_EP_IN  0x81
#define STEAM_PUCK_EP_IN 0x82

/* Report 0x42 is the wired controller-state report. */
#define STEAM_REPORT_STATE 0x42

void steam_parse_state(const uint8_t *buf, uint32_t len, ScePadData *out);
int steam_handle_packet(const uint8_t *buf, uint32_t len, ScePadData *out);
