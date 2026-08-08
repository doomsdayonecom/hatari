/*
  Hatari — RRDC (Retro Remote Debug Controller) backend, contract 0.5.0.

  The portable core (rrdc/retro_control.c) owns the TCP/HTTP server, the /step
  budget, and PPM/WAV marshalling; this file is the Atari-ST-specific backend it
  calls back into, plus the handful of hooks Hatari's main loop drives. Modelled
  on the Mednafen fork's src/control/pcfx_control.cpp, retargeted to Hatari's own
  APIs (STRam, the WinUAE register file, the SDL screen surface, the IKBD, the
  YM2149 mix buffer).

  Wiring (see the calls added to main.c and video.c):
    - st_control_init()  once at startup — reads HATARI_CONTROLPORT, binds server
    - st_control_vbl()   once per video frame at the VBL boundary — services one
                         control request, captures the frame + audio, ticks the
                         /step budget, and BLOCKS while paused (the machine stops
                         at a frame boundary, exactly the RRDC step model)
    - st_control_uninit() at shutdown

  Everything here runs on the emulator thread, so reads are consistent and the
  audio/frame taps need no locking against the emulator (only against the server
  accept thread, which the core already marshals through retro_control_service).
*/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* A 1 ms host sleep for the pause-gate — portable without pulling in SDL (the
 * Hatari Core sources don't get SDL's include path; only src/sdl does). Mirrors
 * the portability split the RRDC core itself uses. */
#if defined(_WIN32) || defined(__EMSCRIPTEN__)
#  include <windows.h>
static void st_sleep1ms(void) { Sleep(1); }
#else
#  include <unistd.h>
static void st_sleep1ms(void) { usleep(1000); }
#endif

#include "retro_control.h"

/* Hatari core headers (src/includes is on the include path). */
#include "main.h"
#include "stMemory.h"
#include "m68000.h"
#include "reset.h"
#include "screen.h"
#include "conv_st.h"        /* ConvertPalette[] — referenced by pixel_convert.h */
#include "pixel_convert.h"
#include "ikbd.h"
#include "keymap.h"
#include "sound.h"

#include "st_control.h"

/* ---- framebuffer capture --------------------------------------------------
 * A generous ceiling: ST/STE low-res is 320x200, but Hatari's surface can carry
 * borders and a display zoom, so size for the largest plausible ST-family frame
 * and report the true dimensions each capture. */
#define FB_MAX_W 1024
#define FB_MAX_H 768
static uint8_t  g_fb[FB_MAX_W * FB_MAX_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* ---- audio tap ------------------------------------------------------------
 * A non-destructive read of Hatari's mixed output ring: we keep our OWN read
 * index and drain from it up to the ring's write position, never touching the
 * SDL consumer's AudioMixBuffer_pos_read. */
static int g_aud_tap = -1;   /* -1 => uninitialised (start at the write head) */

/* ---- memory --------------------------------------------------------------- */

static uint32_t st_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                            uint8_t *out, uint32_t out_cap)
{
	uint32_t n, i;
	(void)bank;
	if (!STRam || !STRamEnd) return 0;
	n = (len < out_cap) ? len : out_cap;
	for (i = 0; i < n; i++)
		out[i] = STRam[(addr + i) % STRamEnd];
	return n;
}

static uint32_t st_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                             const uint8_t *in)
{
	uint32_t i;
	(void)bank;
	if (!STRam || !STRamEnd) return 0;
	for (i = 0; i < len; i++)
		STRam[(addr + i) % STRamEnd] = in[i];
	return len;
}

/* ---- registers ------------------------------------------------------------ */

static void st_get_regs_json(char *buf, size_t cap)
{
	int n, i;
	n = snprintf(buf, cap, "{\"pc\":%u", (unsigned)M68000_GetPC());
	for (i = 0; i < 8 && n > 0 && (size_t)n < cap; i++)
		n += snprintf(buf + n, cap - (size_t)n, ",\"d%d\":%u",
		              i, (unsigned)Regs[REG_D0 + i]);
	for (i = 0; i < 8 && n > 0 && (size_t)n < cap; i++)
		n += snprintf(buf + n, cap - (size_t)n, ",\"a%d\":%u",
		              i, (unsigned)Regs[REG_A0 + i]);
	if (n > 0 && (size_t)n < cap)
		snprintf(buf + n, cap - (size_t)n, "}");
}

/* ---- framebuffer / frame count -------------------------------------------- */

static void st_get_framebuffer(retro_framebuffer_t *out)
{
	out->pixels = g_fb;
	out->width  = g_fb_w;
	out->height = g_fb_h;
	out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t st_get_frame_count(void) { return g_frames; }

/* ---- reset ---------------------------------------------------------------- */

static void st_reset(void) { Reset_Warm(); }

/* ---- keyboard injection ---------------------------------------------------
 * is_text=1 => value is an ASCII codepoint routed through the host-layout
 * keymap; is_text=0 => value is a raw ST scancode. action per retro_key_action_t
 * (TAP presses then releases; DOWN/UP hold/release). */
static int st_press(int is_text, uint32_t value, int down)
{
	if (is_text) {
		if (value > 0x7f) return 0;
		Keymap_SimulateCharacter((char)value, down ? true : false);
	} else {
		IKBD_PressSTKey((uint8_t)value, down ? true : false);
	}
	return 1;
}

static int st_inject_key(int is_text, uint32_t value, int action)
{
	switch (action) {
	case RETRO_KEY_DOWN: return st_press(is_text, value, 1);
	case RETRO_KEY_UP:   return st_press(is_text, value, 0);
	case RETRO_KEY_TAP:
	default:
		if (!st_press(is_text, value, 1)) return 0;
		return st_press(is_text, value, 0);
	}
}

/* ---- audio ---------------------------------------------------------------- */

static uint32_t st_capture_audio(int16_t *out, uint32_t cap,
                                 int *rate, int *channels, uint32_t *dropped)
{
	uint32_t n = 0;
	int write_pos = AudioMixBuffer_pos_write;
	if (rate)     *rate     = nAudioFrequency ? nAudioFrequency : 44100;
	if (channels) *channels = 2;
	if (dropped)  *dropped  = 0;
	if (g_aud_tap < 0) g_aud_tap = write_pos;   /* start at the head */
	/* Drain interleaved L,R int16 from our tap up to the write head. */
	while (n + 1 < cap && g_aud_tap != write_pos) {
		out[n++] = AudioMixBuffer[g_aud_tap][0];
		out[n++] = AudioMixBuffer[g_aud_tap][1];
		g_aud_tap = (g_aud_tap + 1) & AUDIOMIXBUFFER_SIZE_MASK;
	}
	return n;
}

/* ---- pointer / pad --------------------------------------------------------
 * Honest stubs for now so 0.5.0 still advertises cumulatively (a 0 return makes
 * the endpoint answer 400, not a link error). The ST has a mouse + joystick, so
 * these become real in a follow-up (mouse via Keyboard.*, joystick via Joy_*). */
static int st_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{ (void)absolute; (void)x; (void)y; (void)buttons; return 0; }
static int st_get_pointer(int32_t *x, int32_t *y, int *buttons)
{ (void)x; (void)y; (void)buttons; return 0; }
static int st_set_pad(int index, int buttons, int connected)
{ (void)index; (void)buttons; (void)connected; return 0; }
static int st_get_pad(int index, int *buttons, int *connected)
{ (void)index; (void)buttons; (void)connected; return 0; }

/* Field order MUST match retro_control_backend_t exactly. */
static const retro_control_backend_t st_backend = {
	/* platform        */ "atarist",
	/* emulator        */ "hatari",
	/* read_mem        */ st_read_mem,
	/* get_regs_json   */ st_get_regs_json,
	/* get_framebuffer */ st_get_framebuffer,
	/* get_frame_count */ st_get_frame_count,
	/* inject_key      */ st_inject_key,
	/* reset           */ st_reset,
	/* write_mem       */ st_write_mem,
	/* capture_audio   */ st_capture_audio,
	/* set_pointer     */ st_set_pointer,
	/* get_pointer     */ st_get_pointer,
	/* set_pad         */ st_set_pad,
	/* get_pad         */ st_get_pad,
};

/* ---- frame + audio capture at the VBL boundary ---------------------------- */

static void st_capture_frame(void)
{
	uint32_t *pixels = NULL;
	int w = 0, h = 0, pitch = 0, y, W, H;

	Screen_GetDimension(&pixels, &w, &h, &pitch);
	if (!pixels || w <= 0 || h <= 0) return;

	W = (w > FB_MAX_W) ? FB_MAX_W : w;
	H = (h > FB_MAX_H) ? FB_MAX_H : h;

	if (!Screen_Lock()) return;
	for (y = 0; y < H; y++) {
		uint32_t *row = (uint32_t *)((uint8_t *)pixels + (size_t)y * pitch);
		PixelConvert_32to24Bits(&g_fb[(size_t)y * W * 3], row, W, W);
	}
	Screen_UnLock();
	g_fb_w = W;
	g_fb_h = H;
}

/* ---- frontend hooks Hatari's main loop calls ------------------------------ */

void st_control_init(void)
{
	static int started = 0;
	const char *p;
	if (started) return;
	started = 1;
	p = getenv("HATARI_CONTROLPORT");
	if (p && *p)
		retro_control_start(atoi(p), &st_backend);
}

void st_control_uninit(void)
{
	retro_control_stop();
}

/* One call per completed video frame, at the VBL after Video_DrawScreen() +
 * Sound_Update_VBL(). Captures the fresh frame/audio, ticks the frame + /step
 * counters, then blocks (servicing control requests) while the machine is paused
 * — so /pause and /step stop the emulator cleanly at a frame boundary. */
void st_control_vbl(void)
{
	retro_control_service();
	st_capture_frame();
	g_frames++;
	retro_control_on_frame();

	while (!retro_control_running()) {
		retro_control_service();
		st_sleep1ms();
	}
}
