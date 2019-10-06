#include "Arduino.h"
#include "Lepton.h"
#include "SPI.h"
#include "Wire.h"
#include "string.h"
#include "esp_attr.h"
#include "esp_timer.h"


static SPIClass * _SPI[4] = {NULL, NULL, NULL, &SPI};

/*
 ** CCI
 */
static TwoWire * _I2C[2] = {&Wire, NULL};

extern "C" LEP_RESULT ESP_CCI_Init(uint16_t portID, uint16_t baudRateInkHz, int8_t pinSDA, int8_t pinSCL){
    LEP_RESULT result = LEP_OK;
    if(_I2C[portID] == NULL){
        _I2C[portID] = new TwoWire(portID);
    }
    _I2C[portID]->begin(pinSDA, pinSCL, baudRateInkHz * 1000);
    return result;
}

extern "C" LEP_RESULT ESP_CCI_Close(uint16_t portID){
    LEP_RESULT result = LEP_OK;

    return result;
}

extern "C" void ESP_CCI_Write(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * txdata, uint32_t * bytesActuallyWritten){
    *bytesActuallyWritten = 0;
    _I2C[portID]->beginTransmission(deviceAddress);
    _I2C[portID]->write(txdata, len);
    uint8_t error = _I2C[portID]->endTransmission();
    if (error != 0) {
        log_e("error=%u", error);
    } else {
        *bytesActuallyWritten = len;
    }
}

extern "C" void ESP_CCI_Read(uint16_t portID, uint8_t deviceAddress, uint32_t len, uint8_t * rxdata, uint32_t * bytesActuallyRead){
    _I2C[portID]->requestFrom(deviceAddress, len);
    for (int i = 0; i < len; i++) {
        rxdata[i] = _I2C[portID]->read();
    }
    *bytesActuallyRead = len;
}

/*
 ** Time
 */
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
 ** VSYNC
 */
static volatile bool vsync_triggered = 0;
static volatile uint64_t vsync_time = 0;

static void IRAM_ATTR onVsyncISR(){
    vsync_triggered = 1;
    vsync_time = (uint64_t)esp_timer_get_time();
}

extern void waitForVsync(int pinVSYNC){
    if(pinVSYNC < 0){
        return;
    }
    vsync_triggered = 0;
    while(vsync_triggered == 0);
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
    size_t packageLen = 164, seg_length = 4800;

    if(frame->format == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
        packageLen = 244;
    }
    _SPI[portID]->transferBytes(NULL, tmp_buf, packageLen);
    
    if(seg == 0) {
        return 0;
    }

    if((tmp_buf[0] & 0x0F) == 0x0F) {
        return 2;
    }

    //Check if the line number matches the expected line
    if (tmp_buf[1] != line) {
        return 3;
    }

    //For the Lepton3.x, check if the segment number matches
    if (line == 20) {
        uint8_t segment = (tmp_buf[0] >> 4);
        if (segment == 0) {
            return 4;
        }
        if (segment != seg) {
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
    if (!vflip && !hmirror && frame->format == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
        memcpy(&frame8[line_index*3], data8, 240);
        return 0;
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

void LEP_VOSPI_GetFrame(uint16_t portID, int8_t pinCS, uint16_t portMHz, int8_t vsyncPin, LEP_OEM_VIDEO_OUTPUT_FORMAT_TAG format, bool agc, bool telemetry, bool hmirror, bool vflip, uint8_t * line_buf, lepton_frame_t * frame, OnSyncCB cb) {
    uint8_t line, max_lines = 60;
    uint16_t  error;
    uint64_t start_at, sync_start, sync_end;
    static uint64_t vsync_next = 0;

    int res = 0;
    static bool sync_lost = true;

    frame->format = format;
    frame->min_value = 65535;
    frame->max_value = 0;

    start_at = ESP_GetUS();
    if(start_at < vsync_next) {
        start_at = vsync_next - start_at;
        ESP_DelayUS(start_at);
    }

    if(telemetry){
        max_lines = 61;
    }

    _SPI[portID]->beginTransaction(SPISettings(1000000UL * portMHz, MSBFIRST, SPI_MODE3));
    digitalWrite(pinCS, LOW);
    LEP_VOSPI_GetPackage(portID, line_buf, frame, 0, 0, false, false, hmirror, vflip);
    waitForVsync(vsyncPin);
    for (uint8_t segment = 1; segment < 5; segment++) {
        if(segment > 1){
            waitForVsync(vsyncPin);
        }
        error = 0;
        do {
            for (line = 0; line < max_lines; line++) {
                res = LEP_VOSPI_GetPackage(portID, line_buf, frame, line, segment, agc, telemetry, hmirror, vflip);
                if (res == 0){
                    sync_lost = false;
                    continue;
                } else {
                    if(!sync_lost && res != 2) {
                        sync_lost = true;
                        if(res == 4){
                            waitForVsync(vsyncPin);
                        }
                    }
                }
                if (error++ == 512) {
                    segment = 1;
                    error = 0;
                    digitalWrite(pinCS, HIGH);
                    _SPI[portID]->endTransaction();
                    waitForVsync(vsyncPin);
                    if(cb){
                        sync_start = ESP_GetUS();
                        sync_end = sync_start + 188000;
                        cb();
                        ESP_DelayUS(sync_end - ESP_GetUS());
                    } else {
                        ESP_DelayUS(188000);
                    }
                    _SPI[portID]->beginTransaction(SPISettings(1000000UL * portMHz, MSBFIRST, SPI_MODE3));
                    digitalWrite(pinCS, LOW);
                }
                break;
            }
        } while (line != max_lines);
    }
    digitalWrite(pinCS, HIGH);
    _SPI[portID]->endTransaction();
    vsync_next = vsync_time + 83000;
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
    if(_spiPort >1 && _spiPort < 4){
        pinMode(_ssPin, OUTPUT);
        digitalWrite(_ssPin, HIGH);
        if(_SPI[_spiPort] == NULL){
            _SPI[_spiPort] = new SPIClass(_spiPort);
        }
        _SPI[_spiPort]->begin(_sckPin, _misoPin, _mosiPin, -1);
    }

    //Init VSYNC
    if(_vsyncPin >= 0){
        pinMode(_vsyncPin, INPUT);//vsync
        attachInterrupt(_vsyncPin, onVsyncISR, RISING);
    }

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

