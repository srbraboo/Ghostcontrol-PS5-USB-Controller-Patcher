#include "controller_steam.h"

#include <string.h>

enum {
    STEAM_A            = 1u << 0,
    STEAM_B            = 1u << 1,
    STEAM_X            = 1u << 2,
    STEAM_Y            = 1u << 3,
    STEAM_R3           = 1u << 5,
    STEAM_VIEW         = 1u << 6,
    STEAM_R            = 1u << 9,
    STEAM_DPAD_DOWN    = 1u << 10,
    STEAM_DPAD_RIGHT   = 1u << 11,
    STEAM_DPAD_LEFT    = 1u << 12,
    STEAM_DPAD_UP      = 1u << 13,
    STEAM_MENU         = 1u << 14,
    STEAM_L3           = 1u << 15,
    STEAM_STEAM        = 1u << 16,
    STEAM_L            = 1u << 19,
    STEAM_R2_CLICK     = 1u << 23,
    STEAM_L2_CLICK     = 1u << 27,
};

static uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_i16(const uint8_t *p) {
    return (int16_t)read_u16(p);
}

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t stick_axis(int16_t value) {
    return (uint8_t)(((int32_t)value + 32768) >> 8);
}

static uint8_t trigger_axis(int16_t value) {
    /* Triton reports trigger travel as an unsigned 0..32767 value carried
     * in the signed field. Idle is zero; it must not become half-pressed. */
    if (value <= 0) return 0;
    if (value >= 32767) return 255;
    return (uint8_t)(((uint32_t)value * 255u) / 32767u);
}

static uint16_t touch_axis(int16_t value, uint16_t maximum) {
    uint32_t normalized = (uint32_t)((int32_t)value + 32768);
    return (uint16_t)((normalized * maximum) / 65535u);
}

/*
 * USB report 0x42, excluding the report ID:
 *   0: sequence, 1: buttons (uint32), 5: triggers (int16 pair),
 *   9: sticks (four int16), 17: left pad, 23: right pad.
 * The pad coordinates are signed 16-bit values and pressure is unsigned.
 */
void steam_parse_state(const uint8_t *buf, uint32_t len, ScePadData *out) {
    static uint64_t timestamp;
    uint32_t buttons = read_u32(buf + 2);
    uint8_t l2 = trigger_axis(read_i16(buf + 6));
    uint8_t r2 = trigger_axis(read_i16(buf + 8));
    int16_t lx = read_i16(buf + 10);
    int16_t ly = read_i16(buf + 12);
    int16_t rx = read_i16(buf + 14);
    int16_t ry = read_i16(buf + 16);
    int16_t left_pad_x = read_i16(buf + 18);
    int16_t left_pad_y = read_i16(buf + 20);
    uint16_t left_pressure = read_u16(buf + 22);
    int16_t right_pad_x = read_i16(buf + 24);
    int16_t right_pad_y = read_i16(buf + 26);
    uint16_t right_pressure = read_u16(buf + 28);
    int left_touch = (buttons & (1u << 25)) != 0;
    int right_touch = (buttons & (1u << 21)) != 0;
    int left_click = (buttons & (1u << 26)) != 0;
    int right_click = (buttons & (1u << 22)) != 0;

    memset(out, 0, sizeof(*out));
    if (buttons & STEAM_A)         out->buttons |= SCE_PAD_BUTTON_CROSS;
    if (buttons & STEAM_B)         out->buttons |= SCE_PAD_BUTTON_CIRCLE;
    if (buttons & STEAM_X)         out->buttons |= SCE_PAD_BUTTON_SQUARE;
    if (buttons & STEAM_Y)         out->buttons |= SCE_PAD_BUTTON_TRIANGLE;
    if (buttons & STEAM_L)         out->buttons |= SCE_PAD_BUTTON_L1;
    if (buttons & STEAM_R)         out->buttons |= SCE_PAD_BUTTON_R1;
    if (buttons & STEAM_L3)        out->buttons |= SCE_PAD_BUTTON_L3;
    if (buttons & STEAM_R3)        out->buttons |= SCE_PAD_BUTTON_R3;
    if (buttons & STEAM_MENU)      out->buttons |= SCE_PAD_BUTTON_SHARE;
    if (buttons & STEAM_VIEW)      out->buttons |= SCE_PAD_BUTTON_OPTIONS;
    if (buttons & STEAM_STEAM)     out->buttons |= SCE_PAD_BUTTON_PS;
    if (buttons & STEAM_DPAD_UP)   out->buttons |= SCE_PAD_BUTTON_UP;
    if (buttons & STEAM_DPAD_DOWN) out->buttons |= SCE_PAD_BUTTON_DOWN;
    if (buttons & STEAM_DPAD_LEFT) out->buttons |= SCE_PAD_BUTTON_LEFT;
    if (buttons & STEAM_DPAD_RIGHT)out->buttons |= SCE_PAD_BUTTON_RIGHT;
    if ((buttons & STEAM_L2_CLICK) || l2 > 16) out->buttons |= SCE_PAD_BUTTON_L2;
    if ((buttons & STEAM_R2_CLICK) || r2 > 16) out->buttons |= SCE_PAD_BUTTON_R2;
    if (left_touch || right_touch || left_click || right_click)
        out->buttons |= SCE_PAD_BUTTON_TOUCH_PAD;

    out->leftStick.x = stick_axis(lx);
    out->leftStick.y = (uint8_t)(255u - stick_axis(ly));
    out->rightStick.x = stick_axis(rx);
    out->rightStick.y = (uint8_t)(255u - stick_axis(ry));
    out->analogButtons.l2 = l2;
    out->analogButtons.r2 = r2;
    out->touchData.fingers = (uint8_t)(left_touch + right_touch);
    if (left_touch) {
        out->touchData.touch[0].x = touch_axis(left_pad_x, 1919);
        out->touchData.touch[0].y = touch_axis((int16_t)-left_pad_y, 1079);
        out->touchData.touch[0].finger = 1;
    }
    if (right_touch) {
        uint32_t index = (uint32_t)left_touch;
        out->touchData.touch[index].x = touch_axis(right_pad_x, 1919);
        out->touchData.touch[index].y = touch_axis((int16_t)-right_pad_y, 1079);
        out->touchData.touch[index].finger = 1;
    }
    (void)left_pressure;
    (void)right_pressure;
    out->connected = 1;
    out->quat.w = 1.0f;
    out->timestamp = ++timestamp;
    out->count = 1;
    (void)len;
}

int steam_handle_packet(const uint8_t *buf, uint32_t len, ScePadData *out) {
    if (len < 29 || buf[0] != STEAM_REPORT_STATE) return 0;
    steam_parse_state(buf, len, out);
    return 1;
}
