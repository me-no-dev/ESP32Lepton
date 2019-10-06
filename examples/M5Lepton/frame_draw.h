// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _FRAME_DRAW_H_
#define _FRAME_DRAW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>

/*
** Color Transform
*/

typedef enum {
  COLOR_TRANSFORM_NONE,
  COLOR_TRANSFORM_INVERT,
  COLOR_TRANSFORM_DARKEN,
  COLOR_TRANSFORM_LIGHTEN,
  COLOR_TRANSFORM_INVERT_LIGHT,
  COLOR_TRANSFORM_MAX
} color_transform_t;

#define COLOR_TRANSFORM(a,p) (-(COLOR_TRANSFORM_ ## a | (p) << 8))

/*
** Frame Draw
**
** Colors can be transforms instead, in order to modify the current pixel color
*/

void ir_frame_clear(void * data_in, int color);
void ir_frame_pixel(void * data_in, int16_t x, int16_t y, int color);
void ir_frame_hline(void * data_in, int16_t x, int16_t y, uint16_t w, int color);
void ir_frame_vline(void * data_in, int16_t x, int16_t y, uint16_t h, int color);
void ir_frame_fill_rect(void * data_in, int16_t x, int16_t y, uint16_t w, uint16_t h, int color);
uint8_t ir_frame_putc(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, unsigned char c);
uint32_t ir_frame_print(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, const char * str);
uint32_t ir_frame_printf(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* _FRAME_DRAW_H_ */
