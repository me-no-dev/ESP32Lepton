#include "M5Stack.h"
#include "Wire.h"
#include "SPI.h"
#include "Lepton.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

#include "ir_palettes.h"
#include "frame_draw.h"
#include "glcdfont.h"
#include "ip5306.h"

#define BAT_BG_COLOR    COLOR_TRANSFORM(DARKEN, 60)
#define BAT_FRAME_COLOR COLOR_TRANSFORM(LIGHTEN, 80)
#define BAT_FILL_COLOR  COLOR_TRANSFORM(LIGHTEN, 60)
#define BAT_CHG_COLOR   COLOR_TRANSFORM(INVERT, 0)

#define CROSS_COLOR     COLOR_TRANSFORM(INVERT_LIGHT, 90)
#define PALETTE_BORDER_COLOR COLOR_TRANSFORM(DARKEN, 60)

Lepton lepton(
    0, 400, SDA, SCL,             //I2C   (num, khz, sda, scl)
    VSPI, 20, SCK, MISO, MOSI, 5, //SPI   (num, mhz, sck, miso, mosi, cs)
    13                            //VSYNC (pin)
);
const char * mode_names[] = {" RAW", " AGC", " RGB", "", "RAW+T", "AGC+T", " MAX"};
const char * clut_names[] = {" IceFire", "  Gray", "IronBlck", " Fusion", "Contrast", "Rainbow", "168color", "440color", "  TEST"};

static uint8_t clut = PALETTE_IRONBLACK;//lut used in Linear(RAW14) mode
static LEP_VID_LUT_BUFFER_T user_lut;
static bool high_gain = true;

void drawBattery(void * frame_buffer){
    int x = 142;//100;
    int y = 0;
    bool usb = IP5306_GetPowerSource();
    bool full = IP5306_GetBatteryFull();
    uint8_t leds = IP5306_GetLevelLeds();

    ir_frame_fill_rect(frame_buffer, x, y, 18, 9, BAT_BG_COLOR);//bg

    ir_frame_hline(frame_buffer, x+1,  y+1, 15, BAT_FRAME_COLOR);//top line
    ir_frame_hline(frame_buffer, x+1,  y+7, 15, BAT_FRAME_COLOR);//bottom line
    ir_frame_vline(frame_buffer, x+1,  y+2,  5, BAT_FRAME_COLOR);//left border
    ir_frame_vline(frame_buffer, x+15, y+2,  5, BAT_FRAME_COLOR);//right border
    ir_frame_vline(frame_buffer, x+16, y+3,  3, BAT_FRAME_COLOR);//cap
    if(leds & 0x01){
        ir_frame_fill_rect(frame_buffer, x+2,  y+2, 3, 5, BAT_FILL_COLOR);
    }
    if(leds & 0x02){
        ir_frame_fill_rect(frame_buffer, x+5,  y+2, 3, 5, BAT_FILL_COLOR);
    }
    if(leds & 0x04){
        ir_frame_fill_rect(frame_buffer, x+8,  y+2, 3, 5, BAT_FILL_COLOR);
    }
    if(leds & 0x08){
        ir_frame_fill_rect(frame_buffer, x+11, y+2, 3, 5, BAT_FILL_COLOR);
    }
    if(full){//fully charged
        ir_frame_vline(frame_buffer, x+14, y+2, 5, BAT_FILL_COLOR);//100%
    } else if(usb){//charging
        ir_frame_hline(frame_buffer, x+8, y+2, 2, BAT_CHG_COLOR);//top line
        ir_frame_hline(frame_buffer, x+8, y+6, 2, BAT_CHG_COLOR);//bottom line
        ir_frame_hline(frame_buffer, x+7, y+3, 5, BAT_CHG_COLOR);//top plug
        ir_frame_hline(frame_buffer, x+7, y+5, 5, BAT_CHG_COLOR);//bottom plug
        ir_frame_hline(frame_buffer, x+5, y+4, 5, BAT_CHG_COLOR);//cord
    }
}

//FLIR Lepton 3+ specific (160x120 -> 320x240)
void lcdDrawFlirFrame(uint8_t spi_num, const void * data_in){
    if(spi_num > 3){
        return;
    }

    volatile spi_dev_t * _spi[4] = {
        (volatile spi_dev_t *)(DR_REG_SPI0_BASE),
        (volatile spi_dev_t *)(DR_REG_SPI1_BASE),
        (volatile spi_dev_t *)(DR_REG_SPI2_BASE),
        (volatile spi_dev_t *)(DR_REG_SPI3_BASE),
    };

    volatile spi_dev_t * spi = _spi[spi_num];
    uint16_t * data = (uint16_t *)data_in, color;
    static uint32_t line_buf[160];
    uint32_t * line_data = (uint32_t *)line_buf;

    spi->user.usr_mosi = 1;
    spi->user.usr_miso = 0;
    spi->user.doutdin = 0;
    spi->miso_dlen.usr_miso_dbitlen = 0;
    spi->mosi_dlen.usr_mosi_dbitlen = 511;

    for (int y=0; y < 120; y++) {
        for (int x=0; x < 160; x+=16) {
            line_data = (uint32_t *)&line_buf[x];
            for (int i=0; i<16; i++) {
                color = *data++;
                line_data[i] = color << 16 | color;
            }

            while(spi->cmd.usr);
            for (int i=0; i<16; i++) {
                spi->data_buf[i] = line_data[i];
            }
            spi->cmd.usr = 1;
        }
        for (int x=0; x < 160; x+=16) {
            line_data = (uint32_t *)&line_buf[x];
            while(spi->cmd.usr);
            for (int i=0; i<16; i++) {
                spi->data_buf[i] = line_data[i];
            }
            spi->cmd.usr = 1;
        }
    }
    while(spi->cmd.usr);

    spi->user.usr_mosi = 1;
    spi->user.usr_miso = 1;
    spi->user.doutdin = 1;
}

void handleButtons(){
    bool btn_a, btn_b, btn_c;
    static bool last_btn_a = false;
    static bool last_btn_b = false;
    static bool last_btn_c = false;

    M5.update();
    btn_a = M5.BtnA.isPressed();
    btn_b = M5.BtnB.isPressed();
    btn_c = M5.BtnC.isPressed();

    static bool bool_option = false;
    if(btn_a && !last_btn_a){
        bool_option = !bool_option;
        high_gain = !high_gain;
        LEP_SetSysGainMode(lepton.getHandle(), high_gain?LEP_SYS_GAIN_MODE_HIGH:LEP_SYS_GAIN_MODE_LOW);
    }
    if(btn_b && !last_btn_b){
        if(lepton.getFormat() == LEPTON_FORMAT_AGC_TELEMETRY){
            lepton.setFormat(LEPTON_FORMAT_RAW_TELEMETRY);
        } else {
            lepton.setFormat(LEPTON_FORMAT_AGC_TELEMETRY);
        }
    }
    if(btn_c && !last_btn_c){
        clut = (clut + 1) % PALETTE_MAX_LEN;
        ir_palette_to_lut(&palettes[clut], &user_lut);
    }

    last_btn_a = btn_a;
    last_btn_b = btn_b;
    last_btn_c = btn_c;
}

void onSync(){
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(80, 110, 4);
    M5.Lcd.print("Calibrating...");
}

void setup() {
    delay(100);
    M5.begin();

    delay(1000);
    lepton.begin(LEPTON_FORMAT_RAW_TELEMETRY, onSync);
    lepton.setVflip(1);
    lepton.setHmirror(1);

    //load the default palette to the lut (used in AGC mode)
    ir_palette_to_lut(&palettes[clut], &user_lut);

    if(lepton.hasRadiometry()){
        //Set ROI so we can measure the temperature in AGC mode
        LEP_RAD_ROI_T r;
        r.startCol = 79; r.endCol = 80;//x
        r.startRow = 59; r.endRow = 60;//y
        LEP_SetRadSpotmeterRoi(lepton.getHandle(), r);
    }
    LEP_SYS_GAIN_MODE_E gain_mode;
    LEP_GetSysGainMode(lepton.getHandle(), &gain_mode);
    high_gain = gain_mode == LEP_SYS_GAIN_MODE_HIGH;
}

void loop() {
    float max_temp_c, min_temp_c, center_temp_c;//, fpa_temp_c;
    lepton_frame_t * frame = NULL;

    frame = lepton.getFrame();

    if((lepton.getFormat()&3) == LEPTON_FORMAT_RAW){
        ir_raw_to_16(frame->buffer, frame->buffer, &palettes[clut], frame->min_value, frame->max_value);//20ms
    } else {
        ir_agc_to_16(frame->buffer, frame->buffer, &user_lut);//1.8ms
    }

    if(lepton.hasRadiometry()){// || (lepton.getFormat()&3) == LEPTON_FORMAT_RAW){
        //Convert 14bit value to temperature
        min_temp_c = (frame->min_value / 100.0) - 273.15;
        max_temp_c = (frame->max_value / 100.0) - 273.15;
        center_temp_c = (frame->center_value / 100.0) - 273.15;

        //temps UI
        ir_frame_fill_rect(frame->buffer, 0, 0, 37, 9, COLOR_TRANSFORM(DARKEN, 60));
        ir_frame_printf(frame->buffer, 1, 1, TFT_GREEN, TFT_GREEN, 1, "%5.1fC", center_temp_c);
        if((lepton.getFormat()&3) == LEPTON_FORMAT_RAW){
            ir_frame_hline(frame->buffer, 124, 102, 30, COLOR_TRANSFORM(DARKEN, 20));
            ir_frame_printf(frame->buffer, 124, 103, COLOR_TRANSFORM(LIGHTEN, 80), COLOR_TRANSFORM(DARKEN, 20), 1, "%5.1f", min_temp_c);
            ir_frame_hline(frame->buffer, 124, 9, 30, COLOR_TRANSFORM(DARKEN, 20));
            ir_frame_printf(frame->buffer, 124, 10, COLOR_TRANSFORM(LIGHTEN, 80), COLOR_TRANSFORM(DARKEN, 20), 1, "%5.1f", max_temp_c);
        }

        //cross
        ir_frame_hline(frame->buffer, 78, 58, 4, CROSS_COLOR);//top
        ir_frame_hline(frame->buffer, 78, 61, 4, CROSS_COLOR);//bottom
        ir_frame_vline(frame->buffer, 78, 59, 2, CROSS_COLOR);//left
        ir_frame_vline(frame->buffer, 81, 59, 2, CROSS_COLOR);//right

        ir_frame_pixel(frame->buffer, 77, 57, CROSS_COLOR);//top left
        ir_frame_pixel(frame->buffer, 76, 56, CROSS_COLOR);
        ir_frame_pixel(frame->buffer, 82, 57, CROSS_COLOR);//top right
        ir_frame_pixel(frame->buffer, 83, 56, CROSS_COLOR);
        ir_frame_pixel(frame->buffer, 77, 62, CROSS_COLOR);//bottom left
        ir_frame_pixel(frame->buffer, 76, 63, CROSS_COLOR);
        ir_frame_pixel(frame->buffer, 82, 62, CROSS_COLOR);//bottom right
        ir_frame_pixel(frame->buffer, 83, 63, CROSS_COLOR);
    }

    //bottom UI
    ir_frame_fill_rect(frame->buffer, 0, 111, 160, 9, COLOR_TRANSFORM(DARKEN, 60));
    ir_frame_printf(frame->buffer, 22, 112, TFT_CYAN, TFT_CYAN, 1, high_gain?"HIGH":" LOW");
    ir_frame_printf(frame->buffer, 68, 112, TFT_GREEN, TFT_GREEN, 1, mode_names[lepton.getFormat()]);
    ir_frame_printf(frame->buffer, 106, 112, TFT_ORANGE, TFT_ORANGE, 1, "%s", clut_names[clut]);

    //palette
    int palette_index;
    const ir_palette_t * palette;
    uint32_t * palette32;
    uint16_t * palette16, color;
    uint8_t * palette8, r, g, b;
    float scale;

        ir_frame_hline(frame->buffer, 154, 9, 6, PALETTE_BORDER_COLOR);//top
        ir_frame_hline(frame->buffer, 154, 110, 6, PALETTE_BORDER_COLOR);//bottom
        ir_frame_vline(frame->buffer, 154, 10, 100, PALETTE_BORDER_COLOR);//left
        ir_frame_vline(frame->buffer, 159, 10, 100, PALETTE_BORDER_COLOR);//right

        palette = &palettes[clut];
        palette32 = (uint32_t *)palette->colors;
        palette16 = (uint16_t *)palette->colors;
        palette8 = (uint8_t *)palette->colors;
        scale = (float)100 / (float)(palette->len);
        for(int i=0; i<100; i++){
            palette_index = i / scale;
            if(palette->reverse) {
                palette_index = palette->len - palette_index - 1;
            }

            if(palette->type == IR_PALETTE_TYPE_32) {
                color = palette32[palette_index] & 0xFFFF;
                color = color >> 8 | color << 8;
            } else if(palette->type == IR_PALETTE_TYPE_16) {
                color = palette16[palette_index];
                //color = color >> 8 | color << 8;
            } else if(palette->type == IR_PALETTE_TYPE_RGB) {
                palette_index = palette_index * 3;
                r = palette8[palette_index];
                g = palette8[palette_index+1];
                b = palette8[palette_index+2];
                color = (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;
            } else {
                color = 0;
            }
            ir_frame_hline(frame->buffer, 155, 109-i, 4, color);
    }
    drawBattery(frame->buffer);

    //draw frame to screen
    M5.Lcd.startWrite();
    M5.Lcd.setAddrWindow(0, 0, 319, 239);
    lcdDrawFlirFrame(VSPI, frame->buffer);
    M5.Lcd.endWrite();

    handleButtons();
}
