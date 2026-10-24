// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Original work Copyright 2023 Carlos Vicente Ariño Latorre, modified by DIY Embedded Systems 2024.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ABS(x) (((x) > 0) ? (x) : (-(x)))

#define rt_m 0.027 // mechanical response time (s)
#define rt_e 0.003 // electrical response time (s)
#define w_0 300.0 // no load rpm
#define w_l 180.0 // load rpm
#define L 0.001 // inductance (H)
#define R 3.0 // resistance (Ohms)
#define T 0.00001 // discretization time (s)
#define Vn 12.0 // nominal voltage (V)
#define Tn 0.1 // nominal torque (Nm)
#define ppr 100 // pulses per revolution
#define A_V 4 // Voltage gain: motor Voltage / 3.3 V

float ad1, ad0, bd0, bd0_T, bd1_T; //discrete motor parameters 
float w2 = 0, w1 = 0, w0 = 0;
float alpha = 0;
long int alphaA1 = 0, alphaA2 = 0;
long int alphaB1 = 0, alphaB2 = 0;
int ENC_A = 0, ENC_B = 0;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
} rgba_t;

typedef struct {
  buffer_t framebuffer;
  uint32_t width;
  uint32_t height;
} screen_t;


typedef struct {
  pin_t pin_out;
  pin_t pin_vp;
  pin_t pin_vn;
  pin_t pin_A;
  pin_t pin_B;
  uint32_t torque_attr; 

  screen_t screen;
  uint32_t next_update_ns;
} chip_state_t;



static void chip_timer_event(void *user_data);
static void chip_draw_shaft(void *user_data);
static void screen_clear(screen_t *screen, rgba_t color);
static void screen_putpixel(screen_t *screen, int x, int y, rgba_t color);
static void screen_draw_line(screen_t *screen, int x0, int y0, int x1, int y1, rgba_t color);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->pin_out = pin_init("vel", ANALOG);
  chip->pin_vp = pin_init("V+", ANALOG);
  chip->pin_vn = pin_init("V-", ANALOG);
  chip->pin_A = pin_init("ENC_A",OUTPUT_LOW);
  chip->pin_B = pin_init("ENC_B",OUTPUT_LOW);
  chip->torque_attr = attr_init_float("torque", 0.1);
  
  float a2 = rt_m*rt_e;
  float a1 = rt_m+rt_e;
  float a0 = 1;
  float k = w_0/Vn;
  float k_T = (w_0-w_l)/Tn;
  float T2=T*T;
  ad1=(2*a2/T2+a1/T)/(a2/T2+a1/T+a0);
  ad0=-a2/T2/(a2/T2+a1/T+a0);
  bd0 = k/(a2/T2+a1/T+a0);
  bd0_T = k_T/(a2/T2+a1/T+a0);
  // for constant mechanical torque bd1_T is unnecessary

  chip->screen.framebuffer = framebuffer_init(&chip->screen.width, &chip->screen.height);
  
  chip->next_update_ns = get_sim_nanos();

  timer_t timer_id = timer_init(&(const timer_config_t){.callback = chip_timer_event, .user_data = chip});
  timer_start(timer_id, T*1e6, true);

  timer_t timer_screen_id = timer_init(&(const timer_config_t){.callback = chip_draw_shaft, .user_data = chip});
  timer_start(timer_screen_id, 10*1000, true);

}

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  float torque = attr_read_float(chip->torque_attr);
  float vp = pin_adc_read(chip->pin_vp);
  float vn = pin_adc_read(chip->pin_vn);

  w2 = ad1*w1+ad0*w0+bd0*A_V*(vp-vn)-bd0_T*torque;
  w0 = w1;
  w1 = w2;
  pin_dac_write(chip->pin_out, w2*5.0/300.0);
  alpha = alpha + T*w2/60.0*ppr;

  alphaA2 = round(alpha);
  if (alphaA2 != alphaA1) {
    ENC_A = ! ENC_A;   
  }
  pin_write(chip->pin_A,ENC_A);
  alphaA1 = alphaA2;

  alphaB2 = floor(alpha);
  if (alphaB2 != alphaB1) {
    ENC_B = ! ENC_B;   
  }
  pin_write(chip->pin_B,ENC_B);
  alphaB1 = alphaB2;

}

static void chip_draw_shaft(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  screen_t *screen = &chip->screen;
  
  int cx = screen->width / 2;
  int cy = screen->height / 2;
  int r = (int)screen->width * .3;
  
  int px = cx + r * cos(alpha * 3.14/180 *360/100);
  int py = cy + r * sin(alpha * 3.14/180*360/100);

  rgba_t color = (rgba_t){.r = 0xff, .g = 0xff, .b = 0, .a = 0xff};

  screen_clear(screen, (rgba_t){.r = 8, .g = 127, .b = 69, .a = 0xff});
  screen_draw_line(screen, cx, cy, px, py, (rgba_t){.r = 0xff, .g = 0xff, .b = 0xff, .a = 0xff});

  
}

void screen_clear(screen_t *screen, rgba_t color) {
  for (int y = 0; y < screen->height; y++) {
    for (int x = 0; x < screen->width; x++) {
      screen_putpixel(screen, x, y, color);
    }
  }
}

void screen_putpixel(screen_t *screen, int x, int y, rgba_t color) {
  if (screen && x >= 0 && x < screen->width && y >= 0 && y < screen->height) 
  {
    buffer_write(screen->framebuffer, (x + y * screen->width) * 4, &color, sizeof(color));
  }
}

void screen_draw_line(screen_t *screen, int x0, int y0, int x1, int y1, rgba_t color) {
  int dx = ABS(x1 - x0);
  int dy = ABS(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = ((dx > dy) ? dx : -dy) / 2;
  int e2;

  for (;;) {
    screen_putpixel(screen, x0, y0, color);
    if (x0 == x1 && y0 == y1) return;
    e2 = err;
    if (e2 > -dx) {err -= dy; x0 += sx; }
    if (e2 < dy)  {err += dx; y0 += sy; }
  }
}