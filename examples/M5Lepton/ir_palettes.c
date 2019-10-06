#include "ir_palettes.h"


//from LEPTON_VID.h
typedef struct LEP_VID_LUT_PIXEL_T_TAG {
   uint8_t reserved;
   uint8_t red;
   uint8_t green;
   uint8_t blue;
} LEP_VID_LUT_PIXEL_T, *LEP_VID_LUT_PIXEL_T_PTR;

typedef struct LEP_VID_LUT_BUFFER_T_TAG {
   LEP_VID_LUT_PIXEL_T bin[256];
} LEP_VID_LUT_BUFFER_T, *LEP_VID_LUT_BUFFER_T_PTR;


/*
**  Convert FLIR Data to LCD SPI Colors
*/
//from RGB888
void ir_rgb_to_16(void * data_in, void * data_out){
  uint16_t * out = (uint16_t *)data_out, color;
  uint8_t * in = (uint8_t *)data_in, r, g, b;
  for (int x=0; x < (160 * 120); x++) {
    r = *in++;
    g = *in++;
    b = *in++;
    color = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;
    out[x] = color >> 8 | color << 8;
  }
}
//from IR values through palette
void ir_raw_to_16(void * data_in, void * data_out, const ir_palette_t * palette, uint16_t min_value, uint16_t max_value){
    uint16_t * in = (uint16_t *)data_in;
    uint16_t * out = (uint16_t *)data_out;
    uint32_t * palette32 = (uint32_t *)palette->colors;
    uint16_t * palette16 = (uint16_t *)palette32, color;
    uint8_t * palette8 = (uint8_t *)palette32, r, g, b;
    int palette_index = 0;

    int diff = (max_value - min_value);
    float scale = (float)diff / (float)(palette->len);

    for (int x=0; x < (160 * 120); x++) {
        palette_index = (in[x] - min_value) / scale;
        if (palette_index < 0) {
            palette_index = 0;
        } else if (palette_index >= palette->len) {
            palette_index = (palette->len - 1);
        }
        if(palette->reverse) {
            palette_index = palette->len - palette_index - 1;
        }

        if(palette->type == IR_PALETTE_TYPE_32) {
            out[x] = palette32[palette_index] & 0xFFFF;
        } else if(palette->type == IR_PALETTE_TYPE_16) {
            color = palette16[palette_index];
            out[x] = color >> 8 | color << 8;
        } else if(palette->type == IR_PALETTE_TYPE_RGB) {
            palette_index = palette_index * 3;
            r = palette8[palette_index];
            g = palette8[palette_index+1];
            b = palette8[palette_index+2];
            color = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;
            out[x] = color >> 8 | color << 8;
        } else {
            out[x] = 0;
        }
    }
}
//from AGC values (0-255) through LUT
void ir_agc_to_16(void * data_in, void * data_out, LEP_VID_LUT_BUFFER_T_PTR lut){
    uint16_t * in = (uint16_t *)data_in;
    uint16_t * out = (uint16_t *)data_out;
    uint16_t color = 0;
    uint8_t index = 0, r, g, b;
    for (int x=0; x < (160 * 120); x++) {
        index = in[x];
        r = lut->bin[index].red;
        g = lut->bin[index].green;
        b = lut->bin[index].blue;
        color = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;
        out[x] = color >> 8 | color << 8;
    }
}

/*
** Convert Palette to FLIR User LUT
*/
void ir_palette_to_lut(const ir_palette_t * palette, LEP_VID_LUT_BUFFER_T_PTR lut){
  uint32_t * palette32 = (uint32_t *)palette->colors;
  uint16_t * palette16 = (uint16_t *)palette32, color = 0;
  uint8_t * palette8 = (uint8_t *)palette32, r, g, b;
  int palette_index = 0;
  float scale = (float)256 / (float)(palette->len);

  for(uint16_t i=0; i<256; i++){
    palette_index = (float)i / scale;
    if (palette_index < 0) {
      palette_index = 0;
    } else if (palette_index >= palette->len) {
      palette_index = (palette->len - 1);
    }
    if(palette->reverse) {
      palette_index = palette->len - palette_index - 1;
    }

    if(palette->type == IR_PALETTE_TYPE_RGB) {
      palette_index = palette_index * 3;
      r = palette8[palette_index];
      g = palette8[palette_index+1];
      b = palette8[palette_index+2];
    } else {
      if(palette->type == IR_PALETTE_TYPE_32) {
        color = palette32[palette_index] & 0xFFFF;
        color = color >> 8 | color << 8;
      } else if(palette->type == IR_PALETTE_TYPE_16) {
        color = palette16[palette_index];
      }
      r = (color & 0xF800) >> 8;
      g = (color & 0x07E0) >> 3;
      b = (color & 0x001F) << 3;
    }
    lut->bin[i].reserved = 0;
    lut->bin[i].red = r;
    lut->bin[i].green = g;
    lut->bin[i].blue = b;
  }
}











/*
 * 24-bit color maps designed to be indexed by a 8-bit value. Format: R-G-B
 * 
 * Thanks to:
 * Damien Walsh for colormap_golden (from his leptonic demo)
 * Pure Engineering for rainbow, grayscale and ironblack (from the rasperrypi_video demo)
 */

const unsigned char colormap_golden[255][3] = {{0,2,36}, {1,2,37}, {3,3,38}, {3,3,39}, {5,3,41}, {6,3,42}, {8,4,44}, {8,4,46}, {10,4,47}, {12,5,49}, {14,5,51}, {15,5,53}, {17,6,56}, {18,6,58}, {20,7,61}, {22,6,62}, {24,7,66}, {26,7,68}, {28,8,70}, {30,8,73}, {32,9,75}, {35,9,78}, {36,9,81}, {39,10,84}, {41,10,86}, {43,11,89}, {46,11,91}, {47,11,95}, {50,13,97}, {52,13,101}, {55,13,103}, {57,13,106}, {59,14,109}, {61,14,111}, {64,16,114}, {66,16,116}, {68,16,119}, {71,16,121}, {74,18,123}, {76,18,127}, {79,18,129}, {81,19,131}, {83,20,134}, {86,21,135}, {88,21,137}, {90,22,139}, {93,22,141}, {95,23,143}, {97,24,145}, {100,24,147}, {102,25,147}, {104,26,149}, {107,26,151}, {109,27,151}, {111,28,152}, {114,29,153}, {116,29,154}, {117,30,155}, {120,31,155}, {122,32,155}, {124,33,155}, {126,34,155}, {128,34,155}, {130,36,155}, {133,36,154}, {134,37,153}, {137,38,152}, {139,39,151}, {141,40,150}, {144,41,148}, {145,42,147}, {147,42,145}, {150,43,142}, {152,44,141}, {154,45,139}, {156,46,136}, {158,47,134}, {161,48,131}, {163,49,129}, {166,51,126}, {168,51,124}, {170,52,121}, {172,53,118}, {175,54,115}, {177,55,111}, {179,56,108}, {182,58,105}, {184,59,102}, {186,60,99}, {188,61,95}, {191,62,92}, {192,63,89}, {195,64,86}, {197,66,82}, {200,67,79}, {202,68,75}, {203,69,72}, {206,70,69}, {207,71,66}, {210,72,62}, {211,74,59}, {214,75,56}, {216,76,52}, {218,77,49}, {219,79,47}, {222,80,44}, {223,82,41}, {225,82,37}, {227,85,34}, {229,86,32}, {231,87,29}, {232,89,27}, {234,90,25}, {236,92,22}, {236,93,20}, {239,94,18}, {240,95,16}, {241,98,14}, {243,99,12}, {244,100,10}, {245,102,9}, {246,103,8}, {247,105,6}, {248,107,6}, {249,107,6}, {250,110,6}, {251,111,6}, {251,112,6}, {252,114,6}, {253,115,6}, {253,117,6}, {253,119,6}, {253,120,6}, {253,122,6}, {253,124,6}, {253,125,6}, {253,127,6}, {253,129,6}, {253,130,6}, {253,133,6}, {253,134,6}, {253,136,6}, {253,138,6}, {253,140,6}, {253,141,6}, {253,144,6}, {253,146,6}, {253,147,6}, {253,149,6}, {253,151,6}, {253,154,6}, {253,156,6}, {253,158,6}, {253,160,6}, {253,162,6}, {253,164,6}, {253,166,6}, {253,168,6}, {253,170,6}, {253,171,6}, {253,174,6}, {253,175,6}, {253,178,6}, {253,180,6}, {253,181,6}, {253,184,7}, {253,186,7}, {253,187,8}, {253,189,10}, {253,191,10}, {253,193,11}, {253,195,12}, {253,196,13}, {253,199,14}, {253,200,15}, {253,202,16}, {253,204,18}, {253,205,19}, {253,207,20}, {253,209,22}, {253,210,22}, {253,211,24}, {253,214,25}, {253,215,26}, {253,216,28}, {253,218,29}, {253,219,31}, {253,221,31}, {253,223,33}, {253,223,35}, {253,225,36}, {253,225,38}, {253,227,39}, {253,229,42}, {253,230,43}, {253,230,44}, {253,232,47}, {253,233,49}, {253,233,52}, {253,235,54}, {253,236,57}, {254,236,60}, {253,237,62}, {253,239,65}, {253,239,68}, {254,240,72}, {253,241,75}, {254,242,78}, {254,242,82}, {253,243,86}, {253,244,89}, {253,244,93}, {253,245,96}, {254,245,100}, {253,246,104}, {254,247,108}, {254,247,112}, {254,248,115}, {254,248,119}, {254,248,124}, {254,248,128}, {254,249,132}, {254,249,136}, {253,250,141}, {253,250,144}, {254,250,149}, {254,250,153}, {254,251,157}, {254,251,161}, {254,251,165}, {254,251,169}, {253,252,173}, {254,252,177}, {254,252,181}, {254,252,185}, {254,252,189}, {254,252,192}, {254,252,196}, {254,253,200}, {254,252,204}, {254,253,207}, {254,253,211}, {254,253,215}, {254,253,218}, {254,253,221}, {254,253,224}, {254,254,227}, {254,254,230}, {254,254,233}, {254,254,236}, {254,254,238}, {254,254,240}, {254,255,243}, {254,255,245}, {254,254,248}};
const unsigned char colormap_rainbow[768] = {1, 3, 74, 0, 3, 74, 0, 3, 75, 0, 3, 75, 0, 3, 76, 0, 3, 76, 0, 3, 77, 0, 3, 79, 0, 3, 82, 0, 5, 85, 0, 7, 88, 0, 10, 91, 0, 14, 94, 0, 19, 98, 0, 22, 100, 0, 25, 103, 0, 28, 106, 0, 32, 109, 0, 35, 112, 0, 38, 116, 0, 40, 119, 0, 42, 123, 0, 45, 128, 0, 49, 133, 0, 50, 134, 0, 51, 136, 0, 52, 137, 0, 53, 139, 0, 54, 142, 0, 55, 144, 0, 56, 145, 0, 58, 149, 0, 61, 154, 0, 63, 156, 0, 65, 159, 0, 66, 161, 0, 68, 164, 0, 69, 167, 0, 71, 170, 0, 73, 174, 0, 75, 179, 0, 76, 181, 0, 78, 184, 0, 79, 187, 0, 80, 188, 0, 81, 190, 0, 84, 194, 0, 87, 198, 0, 88, 200, 0, 90, 203, 0, 92, 205, 0, 94, 207, 0, 94, 208, 0, 95, 209, 0, 96, 210, 0, 97, 211, 0, 99, 214, 0, 102, 217, 0, 103, 218, 0, 104, 219, 0, 105, 220, 0, 107, 221, 0, 109, 223, 0, 111, 223, 0, 113, 223, 0, 115, 222, 0, 117, 221, 0, 118, 220, 1, 120, 219, 1, 122, 217, 2, 124, 216, 2, 126, 214, 3, 129, 212, 3, 131, 207, 4, 132, 205, 4, 133, 202, 4, 134, 197, 5, 136, 192, 6, 138, 185, 7, 141, 178, 8, 142, 172, 10, 144, 166, 10, 144, 162, 11, 145, 158, 12, 146, 153, 13, 147, 149, 15, 149, 140, 17, 151, 132, 22, 153, 120, 25, 154, 115, 28, 156, 109, 34, 158, 101, 40, 160, 94, 45, 162, 86, 51, 164, 79, 59, 167, 69, 67, 171, 60, 72, 173, 54, 78, 175, 48, 83, 177, 43, 89, 179, 39, 93, 181, 35, 98, 183, 31, 105, 185, 26, 109, 187, 23, 113, 188, 21, 118, 189, 19, 123, 191, 17, 128, 193, 14, 134, 195, 12, 138, 196, 10, 142, 197, 8, 146, 198, 6, 151, 200, 5, 155, 201, 4, 160, 203, 3, 164, 204, 2, 169, 205, 2, 173, 206, 1, 175, 207, 1, 178, 207, 1, 184, 208, 0, 190, 210, 0, 193, 211, 0, 196, 212, 0, 199, 212, 0, 202, 213, 1, 207, 214, 2, 212, 215, 3, 215, 214, 3, 218, 214, 3, 220, 213, 3, 222, 213, 4, 224, 212, 4, 225, 212, 5, 226, 212, 5, 229, 211, 5, 232, 211, 6, 232, 211, 6, 233, 211, 6, 234, 210, 6, 235, 210, 7, 236, 209, 7, 237, 208, 8, 239, 206, 8, 241, 204, 9, 242, 203, 9, 244, 202, 10, 244, 201, 10, 245, 200, 10, 245, 199, 11, 246, 198, 11, 247, 197, 12, 248, 194, 13, 249, 191, 14, 250, 189, 14, 251, 187, 15, 251, 185, 16, 252, 183, 17, 252, 178, 18, 253, 174, 19, 253, 171, 19, 254, 168, 20, 254, 165, 21, 254, 164, 21, 255, 163, 22, 255, 161, 22, 255, 159, 23, 255, 157, 23, 255, 155, 24, 255, 149, 25, 255, 143, 27, 255, 139, 28, 255, 135, 30, 255, 131, 31, 255, 127, 32, 255, 118, 34, 255, 110, 36, 255, 104, 37, 255, 101, 38, 255, 99, 39, 255, 93, 40, 255, 88, 42, 254, 82, 43, 254, 77, 45, 254, 69, 47, 254, 62, 49, 253, 57, 50, 253, 53, 52, 252, 49, 53, 252, 45, 55, 251, 39, 57, 251, 33, 59, 251, 32, 60, 251, 31, 60, 251, 30, 61, 251, 29, 61, 251, 28, 62, 250, 27, 63, 250, 27, 65, 249, 26, 66, 249, 26, 68, 248, 25, 70, 248, 24, 73, 247, 24, 75, 247, 25, 77, 247, 25, 79, 247, 26, 81, 247, 32, 83, 247, 35, 85, 247, 38, 86, 247, 42, 88, 247, 46, 90, 247, 50, 92, 248, 55, 94, 248, 59, 96, 248, 64, 98, 248, 72, 101, 249, 81, 104, 249, 87, 106, 250, 93, 108, 250, 95, 109, 250, 98, 110, 250, 100, 111, 251, 101, 112, 251, 102, 113, 251, 109, 117, 252, 116, 121, 252, 121, 123, 253, 126, 126, 253, 130, 128, 254, 135, 131, 254, 139, 133, 254, 144, 136, 254, 151, 140, 255, 158, 144, 255, 163, 146, 255, 168, 149, 255, 173, 152, 255, 176, 153, 255, 178, 155, 255, 184, 160, 255, 191, 165, 255, 195, 168, 255, 199, 172, 255, 203, 175, 255, 207, 179, 255, 211, 182, 255, 216, 185, 255, 218, 190, 255, 220, 196, 255, 222, 200, 255, 225, 202, 255, 227, 204, 255, 230, 206, 255, 233, 208};
const unsigned char colormap_grayscale[768] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53, 53, 54, 54, 54, 55, 55, 55, 56, 56, 56, 57, 57, 57, 58, 58, 58, 59, 59, 59, 60, 60, 60, 61, 61, 61, 62, 62, 62, 63, 63, 63, 64, 64, 64, 65, 65, 65, 66, 66, 66, 67, 67, 67, 68, 68, 68, 69, 69, 69, 70, 70, 70, 71, 71, 71, 72, 72, 72, 73, 73, 73, 74, 74, 74, 75, 75, 75, 76, 76, 76, 77, 77, 77, 78, 78, 78, 79, 79, 79, 80, 80, 80, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 85, 86, 86, 86, 87, 87, 87, 88, 88, 88, 89, 89, 89, 90, 90, 90, 91, 91, 91, 92, 92, 92, 93, 93, 93, 94, 94, 94, 95, 95, 95, 96, 96, 96, 97, 97, 97, 98, 98, 98, 99, 99, 99, 100, 100, 100, 101, 101, 101, 102, 102, 102, 103, 103, 103, 104, 104, 104, 105, 105, 105, 106, 106, 106, 107, 107, 107, 108, 108, 108, 109, 109, 109, 110, 110, 110, 111, 111, 111, 112, 112, 112, 113, 113, 113, 114, 114, 114, 115, 115, 115, 116, 116, 116, 117, 117, 117, 118, 118, 118, 119, 119, 119, 120, 120, 120, 121, 121, 121, 122, 122, 122, 123, 123, 123, 124, 124, 124, 125, 125, 125, 126, 126, 126, 127, 127, 127, 128, 128, 128, 129, 129, 129, 130, 130, 130, 131, 131, 131, 132, 132, 132, 133, 133, 133, 134, 134, 134, 135, 135, 135, 136, 136, 136, 137, 137, 137, 138, 138, 138, 139, 139, 139, 140, 140, 140, 141, 141, 141, 142, 142, 142, 143, 143, 143, 144, 144, 144, 145, 145, 145, 146, 146, 146, 147, 147, 147, 148, 148, 148, 149, 149, 149, 150, 150, 150, 151, 151, 151, 152, 152, 152, 153, 153, 153, 154, 154, 154, 155, 155, 155, 156, 156, 156, 157, 157, 157, 158, 158, 158, 159, 159, 159, 160, 160, 160, 161, 161, 161, 162, 162, 162, 163, 163, 163, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 170, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 177, 178, 178, 178, 179, 179, 179, 180, 180, 180, 181, 181, 181, 182, 182, 182, 183, 183, 183, 184, 184, 184, 185, 185, 185, 186, 186, 186, 187, 187, 187, 188, 188, 188, 189, 189, 189, 190, 190, 190, 191, 191, 191, 192, 192, 192, 193, 193, 193, 194, 194, 194, 195, 195, 195, 196, 196, 196, 197, 197, 197, 198, 198, 198, 199, 199, 199, 200, 200, 200, 201, 201, 201, 202, 202, 202, 203, 203, 203, 204, 204, 204, 205, 205, 205, 206, 206, 206, 207, 207, 207, 208, 208, 208, 209, 209, 209, 210, 210, 210, 211, 211, 211, 212, 212, 212, 213, 213, 213, 214, 214, 214, 215, 215, 215, 216, 216, 216, 217, 217, 217, 218, 218, 218, 219, 219, 219, 220, 220, 220, 221, 221, 221, 222, 222, 222, 223, 223, 223, 224, 224, 224, 225, 225, 225, 226, 226, 226, 227, 227, 227, 228, 228, 228, 229, 229, 229, 230, 230, 230, 231, 231, 231, 232, 232, 232, 233, 233, 233, 234, 234, 234, 235, 235, 235, 236, 236, 236, 237, 237, 237, 238, 238, 238, 239, 239, 239, 240, 240, 240, 241, 241, 241, 242, 242, 242, 243, 243, 243, 244, 244, 244, 245, 245, 245, 246, 246, 246, 247, 247, 247, 248, 248, 248, 249, 249, 249, 250, 250, 250, 251, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255};
const unsigned char colormap_ironblack[768] = {255, 255, 255, 253, 253, 253, 251, 251, 251, 249, 249, 249, 247, 247, 247, 245, 245, 245, 243, 243, 243, 241, 241, 241, 239, 239, 239, 237, 237, 237, 235, 235, 235, 233, 233, 233, 231, 231, 231, 229, 229, 229, 227, 227, 227, 225, 225, 225, 223, 223, 223, 221, 221, 221, 219, 219, 219, 217, 217, 217, 215, 215, 215, 213, 213, 213, 211, 211, 211, 209, 209, 209, 207, 207, 207, 205, 205, 205, 203, 203, 203, 201, 201, 201, 199, 199, 199, 197, 197, 197, 195, 195, 195, 193, 193, 193, 191, 191, 191, 189, 189, 189, 187, 187, 187, 185, 185, 185, 183, 183, 183, 181, 181, 181, 179, 179, 179, 177, 177, 177, 175, 175, 175, 173, 173, 173, 171, 171, 171, 169, 169, 169, 167, 167, 167, 165, 165, 165, 163, 163, 163, 161, 161, 161, 159, 159, 159, 157, 157, 157, 155, 155, 155, 153, 153, 153, 151, 151, 151, 149, 149, 149, 147, 147, 147, 145, 145, 145, 143, 143, 143, 141, 141, 141, 139, 139, 139, 137, 137, 137, 135, 135, 135, 133, 133, 133, 131, 131, 131, 129, 129, 129, 126, 126, 126, 124, 124, 124, 122, 122, 122, 120, 120, 120, 118, 118, 118, 116, 116, 116, 114, 114, 114, 112, 112, 112, 110, 110, 110, 108, 108, 108, 106, 106, 106, 104, 104, 104, 102, 102, 102, 100, 100, 100, 98, 98, 98, 96, 96, 96, 94, 94, 94, 92, 92, 92, 90, 90, 90, 88, 88, 88, 86, 86, 86, 84, 84, 84, 82, 82, 82, 80, 80, 80, 78, 78, 78, 76, 76, 76, 74, 74, 74, 72, 72, 72, 70, 70, 70, 68, 68, 68, 66, 66, 66, 64, 64, 64, 62, 62, 62, 60, 60, 60, 58, 58, 58, 56, 56, 56, 54, 54, 54, 52, 52, 52, 50, 50, 50, 48, 48, 48, 46, 46, 46, 44, 44, 44, 42, 42, 42, 40, 40, 40, 38, 38, 38, 36, 36, 36, 34, 34, 34, 32, 32, 32, 30, 30, 30, 28, 28, 28, 26, 26, 26, 24, 24, 24, 22, 22, 22, 20, 20, 20, 18, 18, 18, 16, 16, 16, 14, 14, 14, 12, 12, 12, 10, 10, 10, 8, 8, 8, 6, 6, 6, 4, 4, 4, 2, 2, 2, 0, 0, 0, 0, 0, 9, 2, 0, 16, 4, 0, 24, 6, 0, 31, 8, 0, 38, 10, 0, 45, 12, 0, 53, 14, 0, 60, 17, 0, 67, 19, 0, 74, 21, 0, 82, 23, 0, 89, 25, 0, 96, 27, 0, 103, 29, 0, 111, 31, 0, 118, 36, 0, 120, 41, 0, 121, 46, 0, 122, 51, 0, 123, 56, 0, 124, 61, 0, 125, 66, 0, 126, 71, 0, 127, 76, 1, 128, 81, 1, 129, 86, 1, 130, 91, 1, 131, 96, 1, 132, 101, 1, 133, 106, 1, 134, 111, 1, 135, 116, 1, 136, 121, 1, 136, 125, 2, 137, 130, 2, 137, 135, 3, 137, 139, 3, 138, 144, 3, 138, 149, 4, 138, 153, 4, 139, 158, 5, 139, 163, 5, 139, 167, 5, 140, 172, 6, 140, 177, 6, 140, 181, 7, 141, 186, 7, 141, 189, 10, 137, 191, 13, 132, 194, 16, 127, 196, 19, 121, 198, 22, 116, 200, 25, 111, 203, 28, 106, 205, 31, 101, 207, 34, 95, 209, 37, 90, 212, 40, 85, 214, 43, 80, 216, 46, 75, 218, 49, 69, 221, 52, 64, 223, 55, 59, 224, 57, 49, 225, 60, 47, 226, 64, 44, 227, 67, 42, 228, 71, 39, 229, 74, 37, 230, 78, 34, 231, 81, 32, 231, 85, 29, 232, 88, 27, 233, 92, 24, 234, 95, 22, 235, 99, 19, 236, 102, 17, 237, 106, 14, 238, 109, 12, 239, 112, 12, 240, 116, 12, 240, 119, 12, 241, 123, 12, 241, 127, 12, 242, 130, 12, 242, 134, 12, 243, 138, 12, 243, 141, 13, 244, 145, 13, 244, 149, 13, 245, 152, 13, 245, 156, 13, 246, 160, 13, 246, 163, 13, 247, 167, 13, 247, 171, 13, 248, 175, 14, 248, 178, 15, 249, 182, 16, 249, 185, 18, 250, 189, 19, 250, 192, 20, 251, 196, 21, 251, 199, 22, 252, 203, 23, 252, 206, 24, 253, 210, 25, 253, 213, 27, 254, 217, 28, 254, 220, 29, 255, 224, 30, 255, 227, 39, 255, 229, 53, 255, 231, 67, 255, 233, 81, 255, 234, 95, 255, 236, 109, 255, 238, 123, 255, 240, 137, 255, 242, 151, 255, 244, 165, 255, 246, 179, 255, 248, 193, 255, 249, 207, 255, 251, 221, 255, 253, 235, 255, 255, 24};

const unsigned char colormap_test[360] = {0, 0, 64, 10, 0, 77, 19, 0, 90, 29, 0, 102, 39, 0, 115, 48, 0, 128, 58, 0, 141, 68, 0, 154, 77, 0, 166, 87, 0, 179, 95, 0, 192, 86, 0, 198, 76, 0, 204, 66, 0, 211, 56, 0, 217, 47, 0, 224, 37, 0, 230, 27, 0, 237, 18, 0, 243, 8, 0, 250, 0, 1, 253, 0, 8, 240, 0, 14, 227, 0, 20, 214, 0, 27, 202, 0, 33, 189, 0, 40, 176, 0, 46, 163, 0, 53, 150, 0, 59, 138, 0, 69, 131, 0, 88, 144, 0, 107, 157, 0, 127, 170, 0, 146, 182, 0, 165, 195, 0, 184, 208, 0, 204, 221, 0, 223, 234, 0, 242, 246, 0, 251, 249, 0, 238, 229, 0, 225, 210, 0, 212, 191, 0, 200, 172, 0, 187, 152, 0, 174, 133, 0, 161, 114, 0, 148, 94, 0, 135, 75, 0, 133, 61, 0, 146, 55, 0, 159, 48, 0, 172, 42, 0, 185, 35, 0, 197, 29, 0, 210, 23, 0, 223, 16, 0, 236, 10, 0, 249, 3, 3, 249, 0, 10, 236, 0, 16, 223, 0, 23, 210, 0, 29, 197, 0, 35, 185, 0, 42, 172, 0, 48, 159, 0, 55, 146, 0, 61, 133, 0, 75, 135, 0, 94, 148, 0, 114, 161, 0, 133, 174, 0, 152, 187, 0, 172, 200, 0, 191, 212, 0, 210, 225, 0, 229, 238, 0, 249, 251, 0, 246, 242, 0, 234, 223, 0, 221, 204, 0, 208, 184, 0, 195, 165, 0, 182, 146, 0, 170, 127, 0, 157, 107, 0, 144, 88, 0, 131, 69, 0, 138, 59, 0, 150, 53, 0, 163, 46, 0, 176, 40, 0, 189, 33, 0, 202, 27, 0, 214, 20, 0, 227, 14, 0, 240, 8, 0, 253, 1, 0, 250, 0, 8, 243, 0, 18, 237, 0, 27, 230, 0, 37, 224, 0, 47, 217, 0, 56, 211, 0, 66, 204, 0, 76, 198, 0, 86, 192, 0, 95, 197, 24, 111, 203, 49, 127, 210, 75, 143, 216, 101, 159, 223, 126, 175, 229, 152, 191, 236, 178, 207, 242, 204, 223, 249, 229, 239, 255, 255, 255};
const unsigned short ir_bw_palette[34] = {0x001F, 0x0020, 0x0861, 0x10A2, 0x18E3, 0x2124, 0x2965, 0x31A6, 0x39E7, 0x4228, 0x4A69, 0x52AA, 0x5AEB, 0x632C, 0x6B6D, 0x73AE, 0x7BEF, 0x8430, 0x8C71, 0x94B2, 0x9CF3, 0xA534, 0xAD75, 0xB5B6, 0xBDF7, 0xC638, 0xCE79, 0xD6BA, 0xDEFB, 0xE73C, 0xEF7D, 0xF7BE, 0xFFFF, 0xF800};
const unsigned int ir_bw_palette_32[34] = {0x1F001F00, 0x20002000, 0x61086108, 0xA210A210, 0xE318E318, 0x24212421, 0x65296529, 0xA631A631, 0xE739E739, 0x28422842, 0x694A694A, 0xAA52AA52, 0xEB5AEB5A, 0x2C632C63, 0x6D6B6D6B, 0xAE73AE73, 0xEF7BEF7B, 0x30843084, 0x718C718C, 0xB294B294, 0xF39CF39C, 0x34A534A5, 0x75AD75AD, 0xB6B5B6B5, 0xF7BDF7BD, 0x38C638C6, 0x79CE79CE, 0xBAD6BAD6, 0xFBDEFBDE, 0x3CE73CE7, 0x7DEF7DEF, 0xBEF7BEF7, 0xFFFFFFFF, 0x00F800F8};
const unsigned short camColors[168] = { 0x480F, 0x400F, 0x4010, 0x3810, 0x3010, 0x2810, 0x2010, 0x1810, 0x1811, 0x1011, 0x0811, 0x0011, 0x0031, 0x0051, 0x0072, 0x0092, 0x00B2, 0x00D2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0172, 0x0192, 0x01B2, 0x01D2, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0273, 0x0293, 0x02B3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0353, 0x0373, 0x0394, 0x03B4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0554, 0x0574, 0x0573, 0x0572, 0x0571, 0x0591, 0x0590, 0x058F, 0x058E, 0x05AE, 0x05AD, 0x05AC, 0x05AB, 0x05CB, 0x05CA, 0x05C9, 0x05C8, 0x05E8, 0x05E7, 0x05E6, 0x05E5, 0x0604, 0x0603, 0x0602, 0x0601, 0x0621, 0x0620, 0x0E20, 0x0E40, 0x1640, 0x1E40, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3E60, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5E80, 0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x7EA0, 0x86A0, 0x8EA0, 0x8EC0, 0x96C0, 0x9EC0, 0xA6C0, 0xAEC0, 0xB6E0, 0xBEE0, 0xC6E0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0, 0xDEC0, 0xDEA0, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0, 0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480, 0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40, 0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200, 0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0, 0xF080, 0xF060, 0xF040, 0xF020, 0xF800};
const unsigned short ir_palette[440] = {0xFDDB, 0xFC38, 0xFB56, 0xFB55, 0xFB35, 0xFB56, 0xFB35, 0xFB15, 0xFB14, 0xFB15, 0xFB14, 0xFAF4, 0xFB14, 0xFAF4, 0xFAF3, 0xFAF4, 0xFAF3, 0xFAD3, 0xFAF3, 0xFAD3, 0xFAD2, 0xFAD3, 0xFAD2, 0xFAB2, 0xFAD2, 0xFAB2, 0xFAB1, 0xFAB2, 0xFAB1, 0xFA91, 0xFAB1, 0xFA91, 0xFA71, 0xFA91, 0xFA71, 0xFA70, 0xFA71, 0xFA70, 0xFA50, 0xFA4F, 0xFA50, 0xFA4F, 0xFA2F, 0xFA4F, 0xFA2F, 0xFA2E, 0xFA2F, 0xFA2E, 0xFA0E, 0xFA2E, 0xFA0E, 0xFA0D, 0xFA0E, 0xFA0D, 0xF9ED, 0xFA0D, 0xF9ED, 0xF9EC, 0xF9ED, 0xF9EC, 0xF9CC, 0xF9EC, 0xF9CC, 0xF9AB, 0xF9CC, 0xF9AC, 0xF9AB, 0xF98B, 0xF98A, 0xF98B, 0xF98A, 0xF96A, 0xF98A, 0xF96A, 0xF969, 0xF96A, 0xF969, 0xF949, 0xF948, 0xF949, 0xF948, 0xF928, 0xF948, 0xF928, 0xF908, 0xF928, 0xF908, 0xF907, 0xF908, 0xF907, 0xF8E7, 0xF907, 0xF8E7, 0xF8E6, 0xF8E7, 0xF8E6, 0xF8C6, 0xF8E6, 0xF8C6, 0xF8C5, 0xF8C6, 0xF8C5, 0xF8A5, 0xF8C5, 0xF8A5, 0xF8A4, 0xF884, 0xF8A5, 0xF884, 0xF864, 0xF884, 0xF864, 0xF844, 0xF843, 0xF844, 0xF843, 0xF823, 0xF843, 0xF823, 0xF843, 0xF863, 0xF843, 0xF863, 0xF883, 0xF863, 0xF883, 0xF8A3, 0xF883, 0xF8A3, 0xF8C3, 0xF8A3, 0xF8C3, 0xF8E3, 0xF903, 0xF8E3, 0xF903, 0xF923, 0xF903, 0xF923, 0xF924, 0xF944, 0xF923, 0xF944, 0xF964, 0xF984, 0xF964, 0xF984, 0xF9A4, 0xF984, 0xF9A4, 0xF9C4, 0xF9A4, 0xF9C4, 0xF9E4, 0xFA04, 0xF9E4, 0xFA04, 0xFA24, 0xFA04, 0xFA24, 0xFA44, 0xFA24, 0xFA44, 0xFA64, 0xFA44, 0xFA64, 0xFA84, 0xFA64, 0xFA84, 0xFAA4, 0xFA84, 0xFAA4, 0xFAA5, 0xFAA4, 0xFAA5, 0xFAC5, 0xFAA5, 0xFAC5, 0xFAE5, 0xFAC5, 0xFAE5, 0xFB05, 0xFAE5, 0xFB05, 0xFB25, 0xFB45, 0xFB65, 0xFB85, 0xFBA5, 0xFBC5, 0xFBE5, 0xFC05, 0xFC25, 0xFC45, 0xFC46, 0xFC45, 0xFC46, 0xFC66, 0xFC86, 0xFCA6, 0xFCC6, 0xFCE6, 0xFD06, 0xFD26, 0xFD46, 0xFD66, 0xFD86, 0xFDA6, 0xFDC6, 0xFDA6, 0xFDC6, 0xFDC7, 0xFDC6, 0xFDC7, 0xFDE7, 0xFE07, 0xFE27, 0xFE47, 0xFE27, 0xFE47, 0xFE67, 0xFE47, 0xFE67, 0xFE87, 0xFEA7, 0xFEC7, 0xFEE7, 0xFEE8, 0xFEE7, 0xFEE8, 0xFF08, 0xFF28, 0xFF48, 0xFF68, 0xFF48, 0xFF68, 0xFF88, 0xFFA8, 0xFFC8, 0xFFE8, 0xFFC8, 0xFFE8, 0xFFE9, 0xFFE8, 0xFFE9, 0xFFE8, 0xFFE9, 0xF7E8, 0xEFE8, 0xE7E8, 0xDFE8, 0xD7E8, 0xCFE8, 0xC7E8, 0xBFE8, 0xB7E8, 0xAFE8, 0xA7E8, 0x9FE8, 0x97E8, 0x8FE8, 0x87E8, 0x7FE8, 0x77E8, 0x6FE8, 0x67E8, 0x5FE8, 0x57E8, 0x4FE8, 0x47E8, 0x3FE8, 0x37E8, 0x2FE8, 0x2FC8, 0x2FA8, 0x2F88, 0x2F68, 0x2F48, 0x2F28, 0x2F08, 0x2708, 0x26E8, 0x26C8, 0x26A8, 0x2688, 0x2668, 0x2648, 0x2649, 0x2629, 0x2609, 0x25E9, 0x1DEA, 0x25CA, 0x1DCA, 0x1DAA, 0x1D8A, 0x1D6B, 0x1D4B, 0x1D2B, 0x1D2C, 0x1D0C, 0x1CEC, 0x1CCC, 0x1CCD, 0x1CAD, 0x1C8D, 0x1C8E, 0x148E, 0x146E, 0x1C6E, 0x146E, 0x144E, 0x142F, 0x140F, 0x13EF, 0x13F0, 0x13D0, 0x13B0, 0x13B1, 0x1391, 0x1371, 0x1372, 0x1352, 0x1332, 0x1333, 0x1313, 0x12F3, 0x12D3, 0x12D4, 0x12B4, 0x1294, 0x1295, 0x1275, 0x1255, 0x1256, 0x1236, 0x1216, 0x1217, 0x11F7, 0x11D7, 0x11D8, 0x11B8, 0x1198, 0x1179, 0x1159, 0x1139, 0x113A, 0x111A, 0x10FA, 0x10FB, 0x10DB, 0x10BB, 0x109B, 0x109C, 0x107C, 0x105C, 0x103C, 0x101D, 0x101E, 0x181E, 0x101E, 0x181F, 0x181E, 0x181F, 0x181E, 0x201E, 0x203E, 0x205E, 0x207E, 0x287E, 0x289D, 0x289E, 0x28BE, 0x28BD, 0x28DD, 0x30DD, 0x30FD, 0x311D, 0x391D, 0x393D, 0x395D, 0x397D, 0x417D, 0x417C, 0x417D, 0x417C, 0x419C, 0x49BC, 0x49DC, 0x51DC, 0x51FC, 0x521C, 0x5A3C, 0x5A5C, 0x625B, 0x627B, 0x629B, 0x6A9B, 0x6ABB, 0x6ADB, 0x72DB, 0x72FB, 0x7AFB, 0x7B1B, 0x7B3B, 0x833B, 0x833A, 0x833B, 0x835A, 0x837A, 0x8B7A, 0x8B9A, 0x939A, 0x93BA, 0x93DA, 0x9BDA, 0x9BFA, 0x9BF9, 0xA41A, 0xA419, 0xA439, 0xAC59, 0xAC79, 0xB479, 0xB499, 0xB4B9, 0xBCB9, 0xBCD9, 0xBCF9, 0xC4F9, 0xC4F8, 0xC519, 0xC518, 0xB496, 0x72EE, 0x3987, 0x1082, 0x0861, 0x31A6, 0x73AE};

const unsigned short colormap_contrast_rainbow[289];

const ir_palette_t palettes[PALETTE_MAX_LEN] = {
 {ir_bw_palette, 34, IR_PALETTE_TYPE_16, false},//IceFire
 {colormap_grayscale, 256, IR_PALETTE_TYPE_RGB, false},//Gray
 {colormap_ironblack, 256, IR_PALETTE_TYPE_RGB, false},//IronBlck
 {colormap_golden, 255, IR_PALETTE_TYPE_RGB, false},//Fusion
 {colormap_contrast_rainbow, 289, IR_PALETTE_TYPE_16, false},//Contrast
 {colormap_rainbow, 256, IR_PALETTE_TYPE_RGB, false},//Rainbow
 {camColors, 168, IR_PALETTE_TYPE_16, false},//168color
 {ir_palette, 440, IR_PALETTE_TYPE_16, true},//440color
 {colormap_test, 120, IR_PALETTE_TYPE_RGB, false},//TEST
};

const unsigned short colormap_contrast_rainbow[289] = {
0x0000,//2, 0, 6
0x0001,//4, 0, 12
0x0002,//6, 0, 18
0x0803,//8, 0, 24
0x0804,//12, 0, 36
0x0805,//14, 0, 42
0x1006,//16, 0, 48
0x1007,//20, 0, 60
0x1008,//22, 0, 66
0x1809,//24, 0, 72
0x180A,//28, 0, 84
0x180B,//30, 0, 90
0x200C,//32, 0, 96
0x200D,//36, 0, 108
0x200E,//38, 0, 114
0x280F,//40, 0, 120
0x2810,//44, 0, 132
0x2811,//46, 0, 138
0x3012,//48, 0, 144
0x3013,//52, 0, 156
0x3014,//54, 0, 162
0x3815,//56, 0, 168
0x3816,//60, 0, 180
0x3817,//62, 0, 186
0x4018,//64, 0, 192
0x3818,//62, 0, 194
0x3819,//56, 0, 200
0x3019,//54, 0, 202
0x301A,//48, 0, 208
0x281A,//46, 0, 210
0x281B,//40, 0, 216
0x201B,//38, 0, 218
0x201C,//32, 0, 224
0x181C,//30, 0, 226
0x181D,//24, 0, 232
0x101D,//22, 0, 234
0x101E,//16, 0, 240
0x081E,//14, 0, 242
0x081F,//8, 0, 248
0x001F,//6, 0, 250
0x003F,//0, 4, 252
0x005F,//0, 8, 248
0x005E,//0, 10, 246
0x007E,//0, 12, 244
0x009E,//0, 16, 240
0x009D,//0, 18, 238
0x00BD,//0, 20, 236
0x00DD,//0, 24, 232
0x00DC,//0, 26, 230
0x00FC,//0, 28, 228
0x011C,//0, 32, 224
0x011B,//0, 34, 222
0x013B,//0, 36, 220
0x015B,//0, 40, 216
0x015A,//0, 42, 214
0x017A,//0, 44, 212
0x019A,//0, 48, 208
0x0199,//0, 50, 206
0x01B9,//0, 52, 204
0x01D9,//0, 56, 200
0x01D8,//0, 58, 198
0x01F8,//0, 60, 196
0x0218,//0, 64, 192
0x0238,//0, 70, 194
0x0278,//0, 76, 196
0x0298,//0, 82, 198
0x02D9,//0, 88, 200
0x02F9,//0, 94, 202
0x0339,//0, 100, 204
0x0359,//0, 106, 206
0x039A,//0, 112, 208
0x03BA,//0, 118, 210
0x03FA,//0, 124, 212
0x041A,//0, 130, 214
0x045B,//0, 136, 216
0x047B,//0, 142, 218
0x04BB,//0, 148, 220
0x04DB,//0, 154, 222
0x051C,//0, 160, 224
0x053C,//0, 166, 226
0x057C,//0, 172, 228
0x059C,//0, 178, 230
0x05DD,//0, 184, 232
0x05FD,//0, 190, 234
0x063D,//0, 196, 236
0x065D,//0, 202, 238
0x069E,//0, 208, 240
0x06BE,//0, 214, 242
0x06FE,//0, 220, 244
0x071E,//0, 226, 246
0x075F,//0, 232, 248
0x077F,//0, 238, 250
0x07BF,//0, 244, 252
0x07DF,//0, 250, 254
0x07FF,//0, 255, 255
0x07FE,//0, 252, 244
0x07DD,//0, 250, 238
0x07BC,//0, 246, 226
0x07BB,//0, 244, 220
0x079A,//0, 242, 214
0x0779,//0, 238, 202
0x0778,//0, 236, 196
0x0757,//0, 234, 190
0x0736,//0, 230, 178
0x0735,//0, 228, 172
0x0714,//0, 226, 166
0x06F3,//0, 222, 154
0x06F2,//0, 220, 148
0x06D1,//0, 218, 142
0x06B0,//0, 214, 130
0x06AF,//0, 212, 124
0x068E,//0, 210, 118
0x066D,//0, 206, 106
0x066C,//0, 204, 100
0x064B,//0, 202, 94
0x062A,//0, 198, 82
0x0629,//0, 196, 76
0x0608,//0, 194, 70
0x0607,//0, 194, 62
0x0627,//0, 196, 60
0x0647,//0, 200, 56
0x0646,//0, 202, 54
0x0666,//0, 204, 52
0x0686,//0, 208, 48
0x0685,//0, 210, 46
0x06A5,//0, 212, 44
0x06C5,//0, 216, 40
0x06C4,//0, 218, 38
0x06E4,//0, 220, 36
0x0704,//0, 224, 32
0x0703,//0, 226, 30
0x0723,//0, 228, 28
0x0743,//0, 232, 24
0x0742,//0, 234, 22
0x0762,//0, 236, 20
0x0782,//0, 240, 16
0x0781,//0, 242, 14
0x07A1,//0, 244, 12
0x07C1,//0, 248, 8
0x07C0,//0, 250, 6
0x07E0,//0, 252, 4
0x07C0,//6, 250, 0
0x0FC0,//8, 248, 0
0x0FA0,//10, 246, 0
0x0F80,//14, 242, 0
0x1780,//16, 240, 0
0x1760,//18, 238, 0
0x1740,//22, 234, 0
0x1F40,//24, 232, 0
0x1F20,//26, 230, 0
0x1F00,//30, 226, 0
0x2700,//32, 224, 0
0x26E0,//34, 222, 0
0x26C0,//38, 218, 0
0x2EC0,//40, 216, 0
0x2EA0,//42, 214, 0
0x2E80,//46, 210, 0
0x3680,//48, 208, 0
0x3660,//50, 206, 0
0x3640,//54, 202, 0
0x3E40,//56, 200, 0
0x3E20,//58, 198, 0
0x3E00,//62, 194, 0
0x4600,//64, 192, 0
0x4E20,//76, 196, 0
0x5620,//82, 198, 0
0x5E40,//88, 200, 0
0x6660,//100, 204, 0
0x6E60,//106, 206, 0
0x7680,//112, 208, 0
0x7EA0,//124, 212, 0
0x86A0,//130, 214, 0
0x8EC0,//136, 216, 0
0x96E0,//148, 220, 0
0x9EE0,//154, 222, 0
0xA700,//160, 224, 0
0xAF20,//172, 228, 0
0xB720,//178, 230, 0
0xBF40,//184, 232, 0
0xC760,//196, 236, 0
0xCF60,//202, 238, 0
0xD780,//208, 240, 0
0xDFA0,//220, 244, 0
0xE7A0,//226, 246, 0
0xEFC0,//232, 248, 0
0xF7E0,//244, 252, 0
0xFFE0,//250, 254, 0
0xFFC0,//254, 250, 0
0xFFA0,//252, 244, 0
0xFF60,//250, 238, 0
0xFF40,//248, 232, 0
0xF700,//246, 226, 0
0xF6E0,//244, 220, 0
0xF6A0,//242, 214, 0
0xF680,//240, 208, 0
0xEE40,//238, 202, 0
0xEE20,//236, 196, 0
0xEDE0,//234, 190, 0
0xEDC0,//232, 184, 0
0xE580,//230, 178, 0
0xE560,//228, 172, 0
0xE520,//226, 166, 0
0xE500,//224, 160, 0
0xDCC0,//222, 154, 0
0xDCA0,//220, 148, 0
0xDC60,//218, 142, 0
0xDC40,//216, 136, 0
0xD400,//214, 130, 0
0xD3E0,//212, 124, 0
0xD3A0,//210, 118, 0
0xD380,//208, 112, 0
0xCB40,//206, 106, 0
0xCB20,//204, 100, 0
0xCAE0,//202, 94, 0
0xCAC0,//200, 88, 0
0xC280,//198, 82, 0
0xC260,//196, 76, 0
0xC220,//194, 70, 0
0xC200,//192, 64, 0
0xC1E0,//194, 62, 0
0xC1C0,//198, 58, 0
0xC9C0,//200, 56, 0
0xC9A0,//202, 54, 0
0xC980,//206, 50, 0
0xD180,//208, 48, 0
0xD160,//210, 46, 0
0xD140,//214, 42, 0
0xD940,//216, 40, 0
0xD920,//218, 38, 0
0xD900,//222, 34, 0
0xE100,//224, 32, 0
0xE0E0,//226, 30, 0
0xE0C0,//230, 26, 0
0xE8C0,//232, 24, 0
0xE8A0,//234, 22, 0
0xE880,//238, 18, 0
0xF080,//240, 16, 0
0xF060,//242, 14, 0
0xF040,//246, 10, 0
0xF840,//248, 8, 0
0xF820,//250, 6, 0
0xF800,//254, 2, 0
0xF801,//248, 0, 8
0xF001,//246, 0, 10
0xF002,//240, 0, 16
0xE802,//238, 0, 18
0xE803,//232, 0, 24
0xE003,//230, 0, 26
0xE004,//224, 0, 32
0xD804,//222, 0, 34
0xD805,//216, 0, 40
0xD005,//214, 0, 42
0xD006,//208, 0, 48
0xC806,//206, 0, 50
0xC807,//200, 0, 56
0xC007,//198, 0, 58
0xC008,//192, 0, 64
0xC048,//194, 8, 70
0xC089,//196, 16, 76
0xC0CA,//198, 24, 82
0xC90B,//200, 32, 88
0xC94B,//202, 40, 94
0xC98C,//204, 48, 100
0xC9CD,//206, 56, 106
0xD20E,//208, 64, 112
0xD24E,//210, 72, 118
0xD28F,//212, 80, 124
0xD2D0,//214, 88, 130
0xDB11,//216, 96, 136
0xDB51,//218, 104, 142
0xDB92,//220, 112, 148
0xDBD3,//222, 120, 154
0xE414,//224, 128, 160
0xE454,//226, 136, 166
0xE495,//228, 144, 172
0xE4D6,//230, 152, 178
0xED17,//232, 160, 184
0xED57,//234, 168, 190
0xED98,//236, 176, 196
0xEDD9,//238, 184, 202
0xF61A,//240, 192, 208
0xF65A,//242, 200, 214
0xF69B,//244, 208, 220
0xF6DC,//246, 216, 226
0xFF1D,//248, 224, 232
0xFF5D,//250, 232, 238
0xFF9E,//252, 240, 244
0xFFDF,//254, 248, 250
0xFFFF,//255, 255, 255
};
