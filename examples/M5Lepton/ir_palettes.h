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

#ifndef _IR_PALETTES_H_
#define _IR_PALETTES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>

typedef enum {
  IR_PALETTE_TYPE_RGB, //3 bytes per pixel (RGB888)
  IR_PALETTE_TYPE_16,  //2 bytes per pixel (RGB565)
  IR_PALETTE_TYPE_32   //2 bytes per pixel (RGB565), byte-swapped and duplicated (deprecated)
} ir_palette_type_t;

typedef struct {
  const void * colors;
  size_t len;
  ir_palette_type_t type;
  bool reverse;
} ir_palette_t;

enum {
  PALETTE_ICEFIRE,
  PALETTE_GRAYSCALE,
  PALETTE_IRONBLACK,
  PALETTE_FUSION,
  PALETTE_CONTRAST_RAINBOW,
  PALETTE_RAINBOW,
  PALETTE_168_COLOR,
  PALETTE_440_COLOR,
  PALETTE_TEST,
  PALETTE_MAX_LEN
};

struct LEP_VID_LUT_BUFFER_T_TAG;//Declared in LEPTON_VID.h

extern const ir_palette_t palettes[PALETTE_MAX_LEN];

//turn FLIR RGB888 (AGC On) frame to SPI LCD RGB565
void ir_rgb_to_16(void * data_in, void * data_out);

//turn FLIR RAW14 (AGC Off) frame to SPI LCD RGB565
void ir_raw_to_16(void * data_in, void * data_out, const ir_palette_t * palette, uint16_t min_value, uint16_t max_value);

//turn FLIR RAW14 (AGC On) frame to SPI LCD RGB565
void ir_agc_to_16(void * data_in, void * data_out, struct LEP_VID_LUT_BUFFER_T_TAG * lut);

//turn palette into FLIR User LUT to load into the sensor (RGB888+AGC mode)
void ir_palette_to_lut(const ir_palette_t * palette, struct LEP_VID_LUT_BUFFER_T_TAG * lut);

#ifdef __cplusplus
}
#endif

#endif /* _IR_PALETTES_H_ */
