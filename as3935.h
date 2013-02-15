#ifndef __as3935_h
#define __as3935_h

#define AS3935_ADDR 0x00   // For embedded Adventures board
    
#define AS3935_INDOOR_GAIN       0x12
#define AS3935_OUTDOOR_GAIN      0x0E

//#define AS3936_RESET_REGISTERS 0x3c
//#define AS3935_CALIB_RCO       0x3d


#define AS3935_INT_NOISE_LEVEL_HIGH 0b00000001
#define AS3935_INT_DISTURBER        0b00000100
#define AS3935_INT_LIGHTNING        0b00001000
#define AS3935_INT_DISTANCE_CHANGED 0b00000000

typedef enum {
  MIN_1  = 0,
  MIN_5  = 1,
  MIN_9  = 2,
  MIN_16 = 3
} MinStrikes_e;

typedef enum {
  INT_NONE      = 0,
  INT_NOISY     = 1,
  INT_DISTURBER = 4,
  INT_STRIKE    = 8
} InterruptReason_e;

typedef enum {
  LCO_DIV_16  = 0,
  LCO_DIV_32  = 1,
  LCO_DIV_64  = 2,
  LCO_DIV_128 = 3
} LCO_DIV_e;

typedef enum {
  REG00 = 0,
  REG01 = 1,
  REG02 = 2,
  REG03 = 3,
  REG04 = 4,
  REG05 = 5,
  REG06 = 6,
  REG07 = 7,
  REG08 = 8,
  REG09 = 9,
  REG_LUT_TOP = 9,
  REG_LUT_BOT = 0x32,
  REG_RESET   = 0x3C,
  REG_CAL_RCO = 0x3D
} RegisterID_e;

#define MAGIC_VALUE 0x96

typedef union _energy {
  INT32U val;
  INT8U  bits8[4];
} Energy_u;

typedef struct {
  //LSB
  INT8U PWD:1;
  INT8U AFE_GB:5;
  INT8U reserved:2;
} REG00_t;

typedef struct {
  INT8U WDTH:4;
  INT8U NF_LEV:3;
  INT8U reserved:1;
} REG01_t;

typedef struct {
  INT8U SREJ:4;
  INT8U MIN_NUM_LIGHT:2;
  INT8U CL_STAT:1;
  INT8U reserved:1;
} REG02_t;

typedef struct {
  INT8U INT:4;
  INT8U reserved:1;
  INT8U MASK_DIST:1;
  INT8U LCO_FDIV:2;
} REG03_t;

typedef struct {
  INT8U S_LIG_L:8;
} REG04_t;

typedef struct {
  INT8U S_LIG_M:8;
} REG05_t;

typedef struct {
  INT8U S_LIG_MM:5;
  INT8U reserved:3;
} REG06_t;

typedef struct {
  INT8U DISTANCE:6;
  INT8U reserved:2;
} REG07_t;

typedef struct {
  INT8U TUN_CAP:4;
  INT8U reserved:1;
  INT8U DISP_TRCO:1;
  INT8U DISP_SRCO:1;
  INT8U DISP_LCO:1;
} REG08_t;


typedef union {
  REG00_t R0;
  REG01_t R1;
  REG02_t R2;
  REG03_t R3;
  REG04_t R4;
  REG05_t R5;
  REG06_t R6;
  REG07_t R7;
  REG08_t R8;
  INT8U   data;
} REG_u;

INT8U         as3935_get_storm_distance(void);
INT8U         as3935_get_powerdown(void);
void          as3935_set_powerdown(INT8U pwr);
void          as3935_set_afe(INT8U location);
INT8U         as3935_get_afe(void);
void          as3935_set_watchdog_threshold(INT8U threshold);
INT8U         as3935_get_watchdog_threshold(void);
void          as3935_set_noise_floor_level(INT8U nfl);
INT8U         as3935_get_noise_floor_level(void);
void          as3935_set_spike_rejection(INT8U srej);
INT8U         as3935_get_spike_rejection(void);
void          as3935_calibrate_rco(void);
INT32U        as3935_get_energy_calc(void);
InterruptReason_e   as3935_get_interrupt_reason(void);
void          as3935_set_minimum_lightning(MinStrikes_e min);
MinStrikes_e  as3935_get_minimum_lightning(void);
void          as3935_clear_statistics(void);
void          as3935_reset_registers(void);
void          as3935_display_responance_freq_on_irq(INT8U on);
void          as3935_display_trco_on_irq(INT8U on);
void          as3935_display_srco_on_irq(INT8U on);
void          as3935_set_tune_cap(INT8U cap);
INT8U         as3935_get_tune_cap(void);
void          as3935_set_freq_div_ratio(LCO_DIV_e div_ratio);
void          as3935_calibrate(void);
void          as3935_dump(INT8U n);


#endif

