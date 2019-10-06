#include "Lepton.h"
#include "string.h"

#ifdef ARDUINO_ARCH_ESP32
#define USE_ARDUINO_WIRE
#define USE_ARDUINO_SPI
#define USE_ARDUINO_GPIO
#endif


#if defined(USE_ARDUINO_WIRE) || defined(USE_ARDUINO_SPI) || defined(USE_ARDUINO_GPIO)
#include "Arduino.h"
#else
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

/*
 ** Time
 */
#include "esp_attr.h"
#include "esp_timer.h"
#define ESP_GetUS() ((uint64_t)(esp_timer_get_time()))

static void ESP_DelayUS(uint64_t us)
{
    uint64_t m = ESP_GetUS();
    uint64_t e = (m + us);
    if (us) {
        //todo: this sucks if ref tick is 100Hz
        if (us > (3000 * portTICK_PERIOD_MS)) {
            vTaskDelay((us - (2000 * portTICK_PERIOD_MS)) / (portTICK_PERIOD_MS * 1000));
            m = ESP_GetUS();
        }
        if (m > e) { //overflow
            while (ESP_GetUS() > e);
        }
        while (ESP_GetUS() < e);
    }
}

/*
 ** CCI
 */
#ifdef USE_ARDUINO_WIRE
#include "Wire.h"
static TwoWire * _I2C[2] = {&Wire, NULL};
#else
#include "driver/i2c.h"

#define ACK_CHECK_EN            1                 /*!< I2C master will check ack from slave*/
#define ACK_VAL                 (i2c_ack_type_t)0 /*!< I2C ack value */
#define NACK_VAL                (i2c_ack_type_t)1 /*!< I2C nack value */
#endif

extern "C" LEP_RESULT ESP_CCI_Init(uint16_t portID, uint16_t baudRateInkHz, int8_t pinSDA, int8_t pinSCL){
    LEP_RESULT result = LEP_OK;
#ifdef USE_ARDUINO_WIRE
    if(_I2C[portID] == NULL){
        _I2C[portID] = new TwoWire(portID);
    }
    _I2C[portID]->begin(pinSDA, pinSCL, baudRateInkHz * 1000);
#else
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)pinSDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)pinSCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = baudRateInkHz * 1000;
    i2c_param_config((i2c_port_t)portID, &conf);
    i2c_driver_install((i2c_port_t)portID, conf.mode, 0, 0, 0);
#endif
    return result;
}

extern "C" LEP_RESULT ESP_CCI_Close(uint16_t portID){
    LEP_RESULT result = LEP_OK;

    return result;
}

extern "C" void ESP_CCI_Write(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * txdata, uint32_t * bytesActuallyWritten){
#ifdef USE_ARDUINO_WIRE
    *bytesActuallyWritten = 0;
    _I2C[portID]->beginTransmission(deviceAddress);
    _I2C[portID]->write(txdata, len);
    uint8_t error = _I2C[portID]->endTransmission();
    if (error != 0) {
        log_e("error=%u", error);
    } else {
        *bytesActuallyWritten = len;
    }
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, txdata, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)portID, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    *bytesActuallyWritten = (ret == 0)?len:0;
#endif
}

extern "C" void ESP_CCI_Read(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * rxdata, uint32_t * bytesActuallyRead){
#ifdef USE_ARDUINO_WIRE
    _I2C[portID]->requestFrom(deviceAddress, len);
    for (int i = 0; i < len; i++) {
        rxdata[i] = _I2C[portID]->read();
    }
    *bytesActuallyRead = len;
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, rxdata, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, rxdata + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)portID, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    *bytesActuallyRead = (ret == 0)?len:0;
#endif
}

/*
 ** VSYNC
 */
#ifndef USE_ARDUINO_GPIO
#include "driver/gpio.h"
#endif
static volatile bool vsync_triggered = 0;
static volatile uint64_t vsync_time = 0;

static void IRAM_ATTR onVsyncISR(
#ifndef USE_ARDUINO_GPIO
    void * arg
#endif
){
    vsync_triggered = 1;
    vsync_time = (uint64_t)esp_timer_get_time();
}

extern void ESP_VSYNC_Wait(int pinVSYNC){
    if(pinVSYNC < 0){
        return;
    }
    vsync_triggered = 0;
    while(vsync_triggered == 0);
}

extern void ESP_VSYNC_Init(int pinVSYNC){
    if(pinVSYNC < 0){
        return;
    }
#ifdef USE_ARDUINO_GPIO
    pinMode(pinVSYNC, INPUT);//vsync
    attachInterrupt(pinVSYNC, onVsyncISR, RISING);
#else
    gpio_config_t conf = {
        .pin_bit_mask = 1LLU << pinVSYNC,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)pinVSYNC, onVsyncISR, (void*)pinVSYNC);
#endif
}

/*
 ** VOSPI
 */
#ifdef USE_ARDUINO_SPI
#include "SPI.h"
static SPIClass * _SPI[4] = {NULL, NULL, NULL, &SPI};
#else
#include "driver/spi_master.h"
static spi_device_handle_t _SPI[3] = {NULL, NULL, NULL};
#endif

LEP_RESULT ESP_VOSPI_Init(uint16_t portID, uint16_t portMHz, int8_t pinSck, int8_t pinMiso, int8_t pinMosi, int8_t pinCS){
    LEP_RESULT result = LEP_OK;
#ifdef USE_ARDUINO_SPI
    if(portID < 2 || portID > 3){
        return LEP_ERROR;
    }
    pinMode(pinCS, OUTPUT);
    digitalWrite(pinCS, HIGH);
    if(_SPI[portID] == NULL){
        _SPI[portID] = new SPIClass(portID);
    }
    _SPI[portID]->begin(pinSck, pinMiso, pinMosi, -1);
#else
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .mosi_io_num=pinMosi,
        .miso_io_num=pinMiso,
        .sclk_io_num=pinSck,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=244,
        .flags=0,
        .intr_flags=0
    };
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=3,                                //SPI mode 3
        .duty_cycle_pos=0,
        .cs_ena_pretrans=0,
        .cs_ena_posttrans=0,
        .clock_speed_hz=portMHz*1000*1000,      //Clock out at 20 MHz
        .input_delay_ns=0,
        .spics_io_num=-1,                    //CS pin
        .flags=0,
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,
        .post_cb=NULL,
    };
#ifdef ARDUINO_ARCH_ESP32
    portID -= 1;
#endif
    if(portID < 1 || portID > 2){
        return LEP_ERROR;
    }
    ret=spi_bus_initialize((spi_host_device_t)portID, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device((spi_host_device_t)portID, &devcfg, &_SPI[portID]);
    ESP_ERROR_CHECK(ret);

    gpio_config_t conf = {
        .pin_bit_mask = 1LLU << pinCS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&conf);
    gpio_set_level((gpio_num_t)pinCS, 1);
#endif
    return result;
}

extern "C" LEP_RESULT ESP_VOSPI_Close(uint16_t portID){
    LEP_RESULT result = LEP_OK;

    return result;
}

extern "C" LEP_RESULT ESP_VOSPI_Select(uint16_t portID, int8_t pinCS, uint16_t portMHz){
    LEP_RESULT result = LEP_OK;
#ifdef USE_ARDUINO_SPI
    _SPI[portID]->beginTransaction(SPISettings(1000000UL * portMHz, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
#else
    gpio_set_level((gpio_num_t)pinCS, 0);
#endif
    return result;
}

extern "C" LEP_RESULT ESP_VOSPI_Release(uint16_t portID, int8_t pinCS){
    LEP_RESULT result = LEP_OK;
#ifdef USE_ARDUINO_SPI
    digitalWrite(pinCS, HIGH);
    _SPI[portID]->endTransaction();
#else
    gpio_set_level((gpio_num_t)pinCS, 1);
#endif
    return result;
}

extern "C" void ESP_VOSPI_Read(uint16_t portID, uint32_t len, uint8_t * rxdata, uint32_t * bytesActuallyRead){
    *bytesActuallyRead = 0;
#ifdef USE_ARDUINO_SPI
    _SPI[portID]->transferBytes(NULL, rxdata, len);
    *bytesActuallyRead = len;
#else
#ifdef ARDUINO_ARCH_ESP32
    portID -= 1;
#endif
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * len;
    t.rx_buffer = rxdata;

    // ret=spi_device_queue_trans(_SPI[portID], &t, portMAX_DELAY);
    // assert(ret==ESP_OK);
    //can do things here
    // spi_transaction_t *rtrans;
    // ret=spi_device_get_trans_result(_SPI[portID], &rtrans, portMAX_DELAY);

    ret = spi_device_polling_transmit(_SPI[portID], &t);
    if(ret == ESP_OK){
        *bytesActuallyRead = len;
    } else {
        *bytesActuallyRead = 0;
    }
    *bytesActuallyRead = len;
#endif
}

extern "C" void ESP_VOSPI_Write(uint16_t portID, uint32_t len, uint8_t * txdata, uint32_t * bytesActuallyWritten){
    *bytesActuallyWritten = 0;
#ifdef USE_ARDUINO_SPI
    _SPI[portID]->transferBytes(txdata, NULL, len);
    *bytesActuallyWritten = len;
#else
#ifdef ARDUINO_ARCH_ESP32
    portID -= 1;
#endif
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*len;
    t.tx_buffer = txdata;

    // ret=spi_device_queue_trans(_SPI[portID], &t, portMAX_DELAY);
    // assert(ret==ESP_OK);
    //can do things here
    // spi_transaction_t *rtrans;
    // ret=spi_device_get_trans_result(_SPI[portID], &rtrans, portMAX_DELAY);

    ret = spi_device_polling_transmit(_SPI[portID], &t);
    if(ret == ESP_OK){
        *bytesActuallyWritten = len;
    } else {
        *bytesActuallyWritten = 0;
    }
#endif
}

/*
 ** VOSPI to FRAME Logic
 */
static int LEP_VOSPI_GetPackage(uint16_t portID, uint8_t * tmp_buf, lepton_frame_t * frame, uint8_t line, uint8_t seg, bool agc, bool telemetry, bool hmirror, bool vflip){
    uint16_t * data16 = (uint16_t *)&tmp_buf[4];
    uint8_t * data8 = (uint8_t *)data16;
    uint16_t * frame16 = (uint16_t *)&frame->buffer[0];
    uint8_t * frame8 = (uint8_t *)frame16;
    uint16_t * telemetry16 = (uint16_t *)&frame->telemetry.buffer[0];
    uint16_t line_index, column_index, pixel_index, value;
    size_t packageLen = 164, olen = 0, seg_length = 4800;

    if(frame->format == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
        packageLen = 244;
    }
    ESP_VOSPI_Read(portID, packageLen, tmp_buf, &olen);

    if(packageLen != olen) {
        return 1;
    }
    
    if(seg == 0) {
        return 0;
    }

    if((tmp_buf[0] & 0x0F) == 0x0F) {
        return 2;
    }

    //Check if the line number matches the expected line
    if (tmp_buf[1] != line) {
        //ets_printf("3: %u:%u!=%u\n", seg, line, tmp_buf[1]);
        return 3;
    }

    //For the Lepton3.x, check if the segment number matches
    if (line == 20) {
        uint8_t segment = (tmp_buf[0] >> 4);
        if (segment == 0) {
            //ets_printf("4: %u:%u\n", seg, line);
            return 4;
        }
        if (segment != seg) {
            //if(seg == 1)ets_printf("5: %u!=%u\n", seg, segment);
            return 5;
        }
    }

    //if telemetry is on (and we set it's location to header) the first 4 line buffers are the telemetry data.
    if(telemetry && seg == 1 && line < 4){
        if(line == 3){
            return 0;
        }
        telemetry16 = (uint16_t *)&frame->telemetry.buffer[line * 160];
        for(int i=0; i<80; i++){
            value = data16[i];
            telemetry16[i] =  value >> 8 | value << 8;
        }
        return 0;
    }

    if(telemetry){
        seg_length = 4880;//4 lines
    }
    line_index = ((seg - 1) * seg_length) + (line * 80);
    if(telemetry){
        line_index -= 320;//4 lines
    }

    //if flip and mirror are not used, we might use memcpy or faster loop
    if (!vflip && !hmirror) {
        if(frame->format == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
            memcpy(&frame8[line_index*3], data8, 240);
            return 0;
        }
    }

    if (vflip != hmirror) {
        if(line & 1){
            line_index -= 80;
        } else {
            line_index += 80;
        }
    }

    for (int column = 0; column < 80; column++) {
        pixel_index = line_index + column;
        if (vflip) {
            pixel_index = 19199 - pixel_index;
        }

        column_index = column;
        if (vflip != hmirror) {
            column_index = 79 - column;
        }

        if(frame->format == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
            column_index = column_index * 3;
            pixel_index = pixel_index * 3;
            frame8[pixel_index++] = data8[column_index++];
            frame8[pixel_index++] = data8[column_index++];
            frame8[pixel_index++] = data8[column_index++];
        } else {
            value = data16[column_index];
            value = value >> 8 | value << 8;
            frame16[pixel_index] = value;

            if(!agc){
                if (value == 0) {
                    //ets_printf("6: %u:%u\n", seg, line);
                    return 6;
                }
                if (value > frame->max_value) {
                    frame->max_value = value;
                } else if (value < frame->min_value) {
                    frame->min_value = value;
                }
                if(pixel_index == (160*60+80)) {
                    frame->center_value = value;
                }
            }
        }
    }
    return 0;
}

extern void LEP_VOSPI_GetFrame(uint16_t portID, int8_t pinCS, uint16_t portMHz, int8_t vsyncPin, LEP_OEM_VIDEO_OUTPUT_FORMAT_TAG format, bool agc, bool telemetry, bool hmirror, bool vflip, uint8_t * line_buf, lepton_frame_t * frame, OnSyncCB cb) {
    uint8_t line, max_lines = 60;
    uint16_t  error;
    uint64_t start_at, sync_start, sync_end;
    static uint64_t vsync_next = 0;

    int res = 0;
    static bool sync_lost = true;
    //static int last_error = 0;
    //uint16_t errors[4] = {0,0,0,0};
    //static const char * _pkg_errors[] = {"OK","SPI","DISCARD","BAD_LINE","SEG0","BAD_SEG","DATA0"};

    frame->format = format;
    frame->min_value = 65535;
    frame->max_value = 0;

    start_at = ESP_GetUS();
    if(start_at < vsync_next) {
        start_at = vsync_next - start_at;
        //ets_printf("early:%u, ", start_at);
        ESP_DelayUS(start_at);
    }

    if(telemetry){
        max_lines = 61;
    }

    ESP_VOSPI_Select(portID, pinCS, portMHz);
    LEP_VOSPI_GetPackage(portID, line_buf, frame, 0, 0, false, false, hmirror, vflip);
    ESP_VSYNC_Wait(vsyncPin);
    for (uint8_t segment = 1; segment < 5; segment++) {
        if(segment > 1){
            ESP_VSYNC_Wait(vsyncPin);
        }
        error = 0;
        do {
            for (line = 0; line < max_lines; line++) {
                res = LEP_VOSPI_GetPackage(portID, line_buf, frame, line, segment, agc, telemetry, hmirror, vflip);
                if (res == 0){
                    //if(segment > 1 || (segment == 1 && line >= 20)){
                    sync_lost = false;
                    //}
                    continue;
                } else {
                    if(!sync_lost && res != 2) {
                        sync_lost = true;
                        // ets_printf("PKG[%u:%u]: %s\n", segment, line, _pkg_errors[res]);
                        if(res == 4){
                            ESP_VSYNC_Wait(vsyncPin);
                        }
                    }
                }
                if (error++ == 512) {
                    segment = 1;
                    error = 0;
                    ESP_VOSPI_Release(portID, pinCS);
                    ESP_VSYNC_Wait(vsyncPin);
                    if(cb){
                        sync_start = ESP_GetUS();
#ifdef USE_ARDUINO_SPI
                        sync_end = sync_start + 188000;
#else
                        sync_end = sync_start + 197000;
#endif
                        cb();
                        ESP_DelayUS(sync_end - ESP_GetUS());
                    } else {
#ifdef USE_ARDUINO_SPI
                        ESP_DelayUS(188000);
#else
                        ESP_DelayUS(197000);
#endif
                    }
                    ESP_VOSPI_Select(portID, pinCS, portMHz);
                }
                break;
            }
        } while (line != max_lines);
        //errors[segment - 1] = error;
    }
    ESP_VOSPI_Release(portID, pinCS);
    vsync_next = vsync_time + 83000;
    //ets_printf("errors: %u, %u, %u, %u\n", errors[0], errors[1], errors[2], errors[3]);
}

/*
 ** Lepton Public Interface
 */
Lepton::Lepton(uint16_t i2cPort, uint16_t i2cKHz, int sdaPin, int sclPin, uint16_t spiPort, uint16_t spiMHz, int sckPin, int misoPin, int mosiPin, int ssPin, int vsyncPin)
: _i2cPort(i2cPort)
, _i2cKHz(i2cKHz)
, _sdaPin(sdaPin)
, _sclPin(sclPin)
, _spiPort(spiPort)
, _spiMHz(spiMHz)
, _sckPin(sckPin)
, _misoPin(misoPin)
, _mosiPin(mosiPin)
, _ssPin(ssPin)
, _vsyncPin(vsyncPin)
, _format(LEPTON_FORMAT_MAX)
, _radiometry(false)
, _telemetry(false)
, _hmirror(false)
, _vflip(false)
, _cb(NULL)
{}

void Lepton::begin(lepton_format_t format, OnSyncCB cb) {
    _cb = cb;
    //Init SPI
    ESP_VOSPI_Init(_spiPort, _spiMHz, _sckPin, _misoPin, _mosiPin, _ssPin);

    //Init VSYNC
    ESP_VSYNC_Init(_vsyncPin);

    //Init I2C
    LEP_OpenPort(&_handle, _i2cPort, _i2cKHz, _sdaPin, _sclPin);

    //Enable VSYNC
    LEP_SetOemGpioVsyncPhaseDelay(&_handle, LEP_OEM_VSYNC_DELAY_MINUS_3);
    LEP_SetOemGpioMode(&_handle, LEP_OEM_GPIO_MODE_VSYNC);

    //Detect Radiometry
    LEP_RAD_ENABLE_E tlinear = LEP_RAD_DISABLE;
    LEP_GetRadTLinearEnableState(&_handle, &tlinear);
    if(tlinear == LEP_RAD_DISABLE){
        LEP_SetRadTLinearEnableState(&_handle, LEP_RAD_ENABLE);
        LEP_GetRadTLinearEnableState(&_handle, &tlinear);
    }
    _radiometry = tlinear == LEP_RAD_ENABLE;

    //Set Initial Format
    setFormat(format);
}

void Lepton::setFormat(lepton_format_t format){
    if(format == _format){
        return;
    }
    _format = format;
    _telemetry = false;

    //Telemetry
    if(_format & 4){
        _telemetry = true;
        LEP_SetSysTelemetryLocation(&_handle, LEP_TELEMETRY_LOCATION_HEADER);
    }
    LEP_SetSysTelemetryEnableState(&_handle, _telemetry?LEP_TELEMETRY_ENABLED:LEP_TELEMETRY_DISABLED);

    //AGC
    if(format == LEPTON_FORMAT_RGB || (format&3) == LEPTON_FORMAT_AGC) {
        if(_radiometry){
            LEP_SetRadEnableState(&_handle, LEP_RAD_ENABLE);
            LEP_SetRadTLinearEnableState(&_handle, LEP_RAD_DISABLE);
        } else {
            LEP_SetRadEnableState(&_handle, LEP_RAD_DISABLE);
        }
        LEP_SetAgcEnableState(&_handle, LEP_AGC_ENABLE);
    } else {
        LEP_SetAgcEnableState(&_handle, LEP_AGC_DISABLE);
        LEP_SetRadEnableState(&_handle, LEP_RAD_ENABLE);
        if(_radiometry && (format&3) == LEPTON_FORMAT_RAW){
            LEP_SetRadTLinearEnableState(&_handle, LEP_RAD_ENABLE);
            LEP_SetRadTLinearResolution(&_handle, LEP_RAD_RESOLUTION_0_01);
            LEP_SetRadTLinearAutoResolution(&_handle, LEP_RAD_DISABLE);
        }
    }

    //Out Format
    if(_format == LEPTON_FORMAT_RGB){
        LEP_SetOemVideoOutputFormat(&_handle, LEP_VIDEO_OUTPUT_FORMAT_RGB888);
    } else {
        LEP_SetOemVideoOutputFormat(&_handle, LEP_VIDEO_OUTPUT_FORMAT_RAW14);
    }
}

lepton_frame_t * Lepton::getFrame() {
    LEP_OEM_VIDEO_OUTPUT_FORMAT_TAG format = LEP_VIDEO_OUTPUT_FORMAT_RAW14;
    if(_format == LEPTON_FORMAT_RGB){
        format = LEP_VIDEO_OUTPUT_FORMAT_RGB888;
    }

    //Get Frame
    LEP_VOSPI_GetFrame(_spiPort, _ssPin, _spiMHz, _vsyncPin, format, (_format & 3) == LEPTON_FORMAT_AGC, _telemetry, _hmirror, _vflip, _packageBuf, &_frame, _cb);
    _frame.format = (LEP_OEM_VIDEO_OUTPUT_FORMAT_TAG)_format;
    _frame.has_radiometry = _radiometry;

    //Get Spot Temperature
    if(_radiometry){
        if(_format == LEPTON_FORMAT_RGB || _format == LEPTON_FORMAT_AGC){
            LEP_RAD_SPOTMETER_OBJ_KELVIN_T spot;
            LEP_GetRadSpotmeterObjInKelvinX100(&_handle, &spot);
            _frame.center_value = spot.radSpotmeterValue;
        } else if(_telemetry){
            _frame.center_value = _frame.telemetry.spot_mean_kelvin;
        }
    }
    if(_telemetry){
        _frame.fpa_temp = _frame.telemetry.fpa_kelvin;
    } else {
        LEP_GetSysFpaTemperatureKelvin(&_handle, &_frame.fpa_temp);
    }
    //ets_printf("fpa: %u, min:%u, max:%u, center:%u\n", _frame.fpa_temp, _frame.min_value, _frame.max_value, _frame.center_value);
    return &_frame;
}

bool Lepton::hasRadiometry(){
    return _radiometry;
}

lepton_format_t Lepton::getFormat(){
    return _format;
}

LEP_CAMERA_PORT_DESC_T_PTR Lepton::getHandle() {
    return &_handle;
}

void Lepton::setVflip(bool enable){
    _vflip = enable;
}

bool Lepton::getVflip(){
    return _vflip;
}

void Lepton::setHmirror(bool enable){
    _hmirror = enable;
}

bool Lepton::getHmirror(){
    return _hmirror;
}

