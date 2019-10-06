#ifndef Lepton_h
#define Lepton_h

#include "LEPTON_SDK.h"
#include "LEPTON_AGC.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"
#include "LEPTON_SYS.h"
#include "LEPTON_VID.h"

typedef struct {
  uint16_t startRow;
  uint16_t startColumn;
  uint16_t endRow;
  uint16_t endColumn;
} __attribute__((packed)) lepton_roi_t;

typedef union {
  struct {
    //Packet A
    uint16_t telemetry_revision;
    uint32_t counter;
    union {
      struct {
        uint32_t _r0:3;
        uint32_t fcc_desired:1;
        uint32_t fcc_state:2;
        uint32_t _r1:6;
        uint32_t agc_state:1;
        uint32_t _r2:2;
        uint32_t shutter_lockout:1;
        uint32_t _r3:4;
        uint32_t overtemo_shutdown_imminent:1;
        uint32_t _r4:11;
      } __attribute__((packed));
      uint32_t val;
    } __attribute__((packed)) status;
    char serial_num[16];
    uint8_t software_revision[8];
    uint8_t _r1[6];
    uint32_t frame_counter;
    uint16_t frame_mean;
    uint16_t fpa_count;
    uint16_t fpa_kelvin;
    uint16_t housing_count;
    uint16_t housing_kelvin;
    uint8_t _r2[4];
    uint16_t fpa_kelvin_last_ffc;
    uint32_t counter_last_ffc;
    uint16_t housing_kelvin_last_ffc;
    uint8_t _r3[2];
    lepton_roi_t agc_roi;
    uint16_t clip_limit_high;
    uint16_t clip_limit_low;
    uint8_t _r4[64];
    uint32_t video_out_format;
    uint16_t log2_ffc;
    uint8_t _r5[10];

    //Packet B
    uint8_t _r6[38];
    uint16_t emissivity;//scaled by 8192
    uint16_t background_kelvin;
    uint16_t atmospheric_transmission;//scaled by 8192
    uint16_t atmospheric_kelvin;
    uint16_t window_transmission;//scaled by 8192
    uint16_t window_reflection;//scaled by 8192
    uint16_t window_kelvin;
    uint16_t window_reflected_kelvin;
    uint8_t _r7[106];

    //Packet C
    uint8_t _r8[10];
    uint16_t gain_mode;
    uint16_t effective_gain;
    uint16_t gain_desired;
    uint16_t temp_gain_thresh_high2low_c;
    uint16_t temp_gain_thresh_low2high_c;
    uint16_t temp_gain_thresh_high2low_k;
    uint16_t temp_gain_thresh_low2high_k;
    uint8_t _r9[4];
    uint16_t pop_gain_thresh_high2low;
    uint16_t pop_gain_thresh_low2high;
    uint8_t _r10[12];
    lepton_roi_t gain_roi;
    uint8_t _r11[44];
    uint16_t tlinear_enable;
    uint16_t tlinear_resolution;
    uint16_t spot_mean_kelvin;
    uint16_t spot_max_kelvin;
    uint16_t spot_min_kelvin;
    uint16_t spot_population;
    lepton_roi_t spot_roi;
    uint8_t _r12[44];
  } __attribute__((packed));
  uint8_t buffer[160*3];
} __attribute__((packed)) lepton_telemetry_data_t;

typedef struct {
  LEP_OEM_VIDEO_OUTPUT_FORMAT_TAG format;
  uint16_t min_value;
  uint16_t max_value;
  uint16_t center_value;
  uint16_t fpa_temp;
  bool has_radiometry;
  uint8_t buffer[160 * 120 * 3];
  lepton_telemetry_data_t telemetry;//3 lines telemetry
} __attribute__((packed)) lepton_frame_t;

typedef enum {
  LEPTON_FORMAT_RAW = 0,
  LEPTON_FORMAT_AGC,
  LEPTON_FORMAT_RGB,
  LEPTON_FORMAT_RAW_TELEMETRY = 4,
  LEPTON_FORMAT_AGC_TELEMETRY,
  LEPTON_FORMAT_MAX
} lepton_format_t;

typedef void (*OnSyncCB)(void);

class Lepton {
public:

  Lepton(uint16_t i2cPort, uint16_t i2cKHz, int sdaPin, int sclPin, uint16_t spiPort, uint16_t spiMHz, int sckPin, int misoPin, int mosiPin, int ssPin, int vsyncPin);
  ~Lepton(){}

  void begin(lepton_format_t format = LEPTON_FORMAT_RAW_TELEMETRY, OnSyncCB cb=NULL);

  //Use to access the LEPTON API
  LEP_CAMERA_PORT_DESC_T_PTR getHandle();

  bool hasRadiometry();

  void setVflip(bool enable);
  bool getVflip();

  void setHmirror(bool enable);
  bool getHmirror();

  void setFormat(lepton_format_t format);
  lepton_format_t getFormat();

  lepton_frame_t * getFrame();

private:
  uint16_t _i2cPort;
  uint16_t _i2cKHz;
  int _sdaPin;
  int _sclPin;
  uint16_t _spiPort;
  uint16_t _spiMHz;
  int _sckPin;
  int _misoPin;
  int _mosiPin;
  int _ssPin;
  int _vsyncPin;
  lepton_format_t _format;
  bool _radiometry;
  bool _telemetry;
  bool _hmirror;
  bool _vflip;
  OnSyncCB _cb;
  LEP_CAMERA_PORT_DESC_T _handle;
  uint8_t _packageBuf[244];
  lepton_frame_t _frame;
};

#endif
