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

#include "frame_draw.h"
#include "glcdfont.h"
#include "esp_attr.h"
#include <stdio.h>

/*
** Color Transform
*/

static void IRAM_ATTR color_transform_rgb888(uint8_t r_in, uint8_t g_in, uint8_t b_in, color_transform_t action, uint8_t percentage, uint8_t * r_out, uint8_t * g_out, uint8_t * b_out){
  uint16_t r, g, b, c;
  if(action == COLOR_TRANSFORM_INVERT_LIGHT){
    c = (r_in + g_in + b_in) / 3;
    if(c > 127){
      action = COLOR_TRANSFORM_DARKEN;
    } else {
      action = COLOR_TRANSFORM_LIGHTEN;
    }
  }

  switch(action){
    case COLOR_TRANSFORM_DARKEN:
      percentage = 100 - percentage;
      r = (r_in * percentage) / 100;
      g = (g_in * percentage) / 100;
      b = (b_in * percentage) / 100;
    break;
    case COLOR_TRANSFORM_LIGHTEN:
      r = r_in + (((255 - r_in) * percentage) / 100);
      g = g_in + (((255 - g_in) * percentage) / 100);
      b = b_in + (((255 - b_in) * percentage) / 100);
      if(r > 255) r = 255;
      if(g > 255) g = 255;
      if(b > 255) b = 255;
    break;
    case COLOR_TRANSFORM_INVERT:
      r = 255 - r_in;
      g = 255 - g_in;
      b = 255 - b_in;
    break;
    default:
      r = r_in;
      g = g_in;
      b = b_in;
    break;
  }

  *r_out = r;
  *g_out = g;
  *b_out = b;
}

static uint16_t IRAM_ATTR color_transform_rgb565(uint16_t color, color_transform_t action, uint8_t percentage){
  uint8_t r, g, b;
  if(action <= COLOR_TRANSFORM_NONE || action >= COLOR_TRANSFORM_MAX){
    return color;
  }
  r = (color & 0xF800) >> 8;
  g = (color & 0x07E0) >> 3;
  b = (color & 0x001F) << 3;
  color_transform_rgb888(r, g, b, action, percentage, &r, &g, &b);
  color = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;
  return color;
}

static uint16_t IRAM_ATTR color_transform_16(uint16_t color, color_transform_t action, uint8_t percentage){
  color = (color >> 8) | (color << 8);
  color = color_transform_rgb565(color, action, percentage);
  color = (color >> 8) | (color << 8);
  return color;
}

/*
** Frame Draw
*/

void ir_frame_clear(void * data_in, int color){
    ir_frame_fill_rect(data_in, 0, 0, 160, 120, color);
}

void ir_frame_pixel(void * data_in, int16_t x, int16_t y, int color){
  if(x < 0 || x >= 160 || y < 0 || y >= 120){
    return;
  }
  uint16_t *data = (uint16_t*)data_in;
  size_t start_index = (y * 160) + x;
  if(color < 0){
    color *= -1;
    data[start_index] = color_transform_16(data[start_index], (color_transform_t)(color & 0xFF), (color >> 8) & 0x7F);
  } else {
    color = ((color & 0xFF00) >> 8) | ((color & 0xFF) << 8);
    data[start_index] = color;
  }
}

void ir_frame_hline(void * data_in, int16_t x, int16_t y, uint16_t w, int color){
  if(x < 0 || x >= 160 || y < 0 || y >= 120){
    return;
  }
  if((x + w) > 160){
    w = 160 - x;
  }
  uint16_t *data = (uint16_t*)data_in, action = 0, percentage = 100;
  size_t start_index = (y * 160) + x;
  if(color >= 0){
    color = ((color & 0xFF00) >> 8) | ((color & 0xFF) << 8);
  } else {
    action = -color;
    percentage = (action >> 8) & 0xFF;
    action &= 0xFF;
  }
  data = &data[start_index];
  for(uint16_t i=0; i<w; i++){
    if(color < 0){
      data[i] = color_transform_16(data[i], (color_transform_t)(action), percentage);
    } else {
      data[i] = color;
    }
  }
}

void ir_frame_vline(void * data_in, int16_t x, int16_t y, uint16_t h, int color){
  if(x < 0 || x >= 160 || y < 0 || y >= 120){
    return;
  }
  if((y + h) > 120){
    h = 120 - y;
  }
  uint16_t *data = (uint16_t*)data_in, i, action = 0, percentage = 100;
  size_t start_index = (y * 160) + x;
  if(color >= 0){
    color = ((color & 0xFF00) >> 8) | ((color & 0xFF) << 8);
  } else {
    action = -color;
    percentage = (action >> 8) & 0xFF;
    action &= 0xFF;
  }
  for(uint16_t a=0; a<h; a++){
    i = start_index + (a * 160);
    if(color < 0){
      data[i] = color_transform_16(data[i], (color_transform_t)(action), percentage);
    } else {
      data[i] = color;
    }
  }
}

void ir_frame_fill_rect(void * data_in, int16_t x, int16_t y, uint16_t w, uint16_t h, int color){
  if(x < 0 || x >= 160 || y < 0 || y >= 120){
    return;
  }
  if((x + w) > 160){
    w = 160 - x;
  }
  if((y + h) > 120){
    h = 120 - y;
  }
  uint16_t *data = (uint16_t*)data_in, *line, action = 0, percentage = 100;
  size_t start_index = (y * 160) + x;
  if(color >= 0){
    color = ((color & 0xFF00) >> 8) | ((color & 0xFF) << 8);
  } else {
    action = -color;
    percentage = (action >> 8) & 0xFF;
    action &= 0xFF;
  }
  for(uint16_t i=0; i<h; i++){
    line = &data[start_index + (i * 160)];
    for(uint16_t a=0; a<w; a++){
      if(color < 0){
        line[a] = color_transform_16(line[a], (color_transform_t)(action), percentage);
      } else {
        line[a] = color;
      }
    }
  }
}

uint8_t ir_frame_putc(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, unsigned char c) {
  if(color != bg_color){
    ir_frame_fill_rect(data_in, x, y, 6 * size, 8 * size, bg_color);
  }
  for (int8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5)
      line = 0x0;
    else
      line = font[(c*5)+i];
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
          if(size == 1){
              ir_frame_pixel(data_in, x+i, y+j, color);
          } else {
              ir_frame_fill_rect(data_in, x+(i*size), y+(j*size), size, size, color);
          }
      }
      line >>= 1;
    }
  }
  return 6*size;
}

uint32_t ir_frame_print(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, const char * str)
{
    uint32_t l = 0;
    int xc = x, yc = y, lc = 160 - (6 * size);
    uint8_t  fh = 8 * size;
    char c = *str++;
    while(c){
        if(c != '\r'){
            if(c == '\n'){
                yc += fh;
                xc = x;
            } else {
                if(xc > lc){
                    yc += fh;
                    xc = x;
                }
                xc += ir_frame_putc(data_in, xc, yc, color, bg_color, size, c);
            }
        }
        l++;
        c = *str++;
    }
    return l;
}

uint32_t ir_frame_printf(void * data_in, int16_t x, int16_t y, int color, int bg_color, uint8_t size, const char *format, ...)
{
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, 64, format, arg);
    va_end(copy);
    if(len >= 64){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    ir_frame_print(data_in, x, y, color, bg_color, size, temp);
    if(len >= 64){
        free(temp);
    }
    return len;
}
