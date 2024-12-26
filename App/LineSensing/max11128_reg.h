// clang-format off
#ifndef MAX11128_REGS_H
#define MAX11128_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(push, 1)
typedef enum
{
  MAX11128_AIN_0,
  MAX11128_AIN_1,
  MAX11128_AIN_2,
  MAX11128_AIN_3,
  MAX11128_AIN_4,
  MAX11128_AIN_5,
  MAX11128_AIN_6,
  MAX11128_AIN_7,
  MAX11128_AIN_8,
  MAX11128_AIN_9,
  MAX11128_AIN_10,
  MAX11128_AIN_11,
  MAX11128_AIN_12,
  MAX11128_AIN_13,
  MAX11128_AIN_14,
  MAX11128_AIN_15,
} MAX11128_AIN;
#define NUM_MAX11128_AIN (MAX11128_AIN_15 + 1)
typedef enum
{
  MAX11128_REG_IDENT_REG_CTRL = 0b00000,
  MAX11128_REG_IDENT_CONFIG_SETUP = 0b10000,
  MAX11128_REG_IDENT_UNI_SETUP = 0b10001,
  MAX11128_REG_IDENT_BIP_SETUP = 0b10010,
  MAX11128_REG_IDENT_RANGE_SETUP = 0b10011,
  MAX11128_REG_IDENT_CUST_SCAN0 = 0b10100,
  MAX11128_REG_IDENT_CUST_SCAN1 = 0b10101,
  MAX11128_REG_IDENT_SMPL_SET = 0b10110,
} MAX11128_REG_IDENT;
// Table 2. ADC Mode Control Register
typedef enum
{
  // Table 3. ADC Scan Control
  MAX11128_ADC_MODE_CTRL_SCAN_NONE = 0b0000,
  MAX11128_ADC_MODE_CTRL_SCAN_MANUAL = 0b0001,
  MAX11128_ADC_MODE_CTRL_SCAN_REPEAT = 0b0010,
  MAX11128_ADC_MODE_CTRL_SCAN_STANDARD_INT = 0b0011,
  MAX11128_ADC_MODE_CTRL_SCAN_STANDARD_EXT = 0b0100,
  MAX11128_ADC_MODE_CTRL_SCAN_UPPER_INT = 0b0101,
  MAX11128_ADC_MODE_CTRL_SCAN_UPPER_EXT = 0b0110,
  MAX11128_ADC_MODE_CTRL_SCAN_CUSTOM_INT = 0b0111,
  MAX11128_ADC_MODE_CTRL_SCAN_CUSTOM_EXT = 0b1000,
  MAX11128_ADC_MODE_CTRL_SCAN_SAMPLESET = 0b1001,
} MAX11128_ADC_MODE_CTRL_SCAN;
typedef enum
{
  MAX11128_ADC_MODE_CTRL_RESET_NO = 0b00,
  MAX11128_ADC_MODE_CTRL_RESET_FIFO = 0b01,
  MAX11128_ADC_MODE_CTRL_RESET_ALL = 0b10,
} MAX11128_ADC_MODE_CTRL_RESET;
typedef enum
{
  // Table 5. Power Management Modes
  MAX11128_ADC_MODE_CTRL_PM_NORMAL = 0b00,
  MAX11128_ADC_MODE_CTRL_PM_AUTO_SHUTDOWN = 0b01,
  MAX11128_ADC_MODE_CTRL_PM_AUTO_STANDBY = 0b10,
} MAX11128_ADC_MODE_CTRL_PM;
typedef struct
{
  uint16_t unused : 1;
  uint16_t swcnv : 1;
  uint16_t chan_id : 1;
  MAX11128_ADC_MODE_CTRL_PM pm : 2;
  MAX11128_ADC_MODE_CTRL_RESET reset : 2;
  MAX11128_AIN chsel : 4;
  MAX11128_ADC_MODE_CTRL_SCAN scan : 4;
  uint16_t reg_ctrl : 1;
} MAX11128_ADC_MODE_CTRL;
// Table 6. ADC Configuration Register
typedef enum
{
  MAX11128_ADC_CONFIG_NAVG_4 = 0b00,
  MAX11128_ADC_CONFIG_NAVG_8 = 0b01,
  MAX11128_ADC_CONFIG_NAVG_16 = 0b10,
  MAX11128_ADC_CONFIG_NAVG_32 = 0b11,
} MAX11128_ADC_CONFIG_NAVG;
typedef enum
{
  MAX11128_ADC_CONFIG_NSCAN_4 = 0b00,
  MAX11128_ADC_CONFIG_NSCAN_8 = 0b01,
  MAX11128_ADC_CONFIG_NSCAN_12 = 0b10,
  MAX11128_ADC_CONFIG_NSCAN_16 = 0b11,

} MAX11128_ADC_CONFIG_NSCAN;
typedef enum
{
  MAX11128_ADC_CONFIG_SPM_NORMAL = 0b00,
  MAX11128_ADC_CONFIG_SPM_FULL_SHUTDOWN = 0b01,
  MAX11128_ADC_CONFIG_SPM_PARTIAL_SHUTDOWN = 0b10,
} MAX11128_ADC_CONFIG_SPM;
typedef struct
{
  uint16_t unused : 1;
  uint16_t echo : 1;
  MAX11128_ADC_CONFIG_SPM spm : 2;
  MAX11128_ADC_CONFIG_NSCAN nscan : 2;
  MAX11128_ADC_CONFIG_NAVG navg : 3;
  uint16_t avgon : 1;
  uint16_t refsel : 1;
  MAX11128_REG_IDENT config_setup : 5;
} MAX11128_ADC_CONFIG;
// Table 7. RANGE Register
typedef struct
{
  uint16_t unused : 2;
  uint16_t range14_15 : 1;
  uint16_t range12_13 : 1;
  uint16_t range10_11 : 1;
  uint16_t range8_9 : 1;
  uint16_t range6_7 : 1;
  uint16_t range4_5 : 1;
  uint16_t range2_3 : 1;
  uint16_t range0_1 : 1;
  MAX11128_REG_IDENT range_setup : 5;
} MAX11128_RANGE;
// Table 10. Unipoar Register
typedef struct
{
  uint16_t unused : 2;
  uint16_t pdiff_com : 1;
  uint16_t uch14_15 : 1;
  uint16_t uch12_13 : 1;
  uint16_t uch10_11 : 1;
  uint16_t uch8_9 : 1;
  uint16_t uch6_7 : 1;
  uint16_t uch4_5 : 1;
  uint16_t uch2_3 : 1;
  uint16_t uch0_1 : 1;
  MAX11128_REG_IDENT uni_setup : 5;
} MAX11128_UNI;
// Table 11. Bipolar Register
typedef struct
{
  uint16_t unused : 3;
  uint16_t bch14_15 : 1;
  uint16_t bch12_13 : 1;
  uint16_t bch10_11 : 1;
  uint16_t bch8_9 : 1;
  uint16_t bch6_7 : 1;
  uint16_t bch4_5 : 1;
  uint16_t bch2_3 : 1;
  uint16_t bch0_1 : 1;
  MAX11128_REG_IDENT bip_setup : 5;
} MAX11128_BIP;
// Table 12. Custom Scan0 Register
typedef struct
{
  uint16_t unused : 2;
  uint16_t chscan8 : 1;
  uint16_t chscan9 : 1;
  uint16_t chscan10 : 1;
  uint16_t chscan11 : 1;
  uint16_t chscan12 : 1;
  uint16_t chscan13 : 1;
  uint16_t chscan14 : 1;
  uint16_t chscan15 : 1;
  MAX11128_REG_IDENT cust_scan0 : 5;
} MAX11128_CUSTOM_SCAN0;
// Table 13. Custom Scan1 Register
typedef struct
{
  uint16_t unused : 2;
  uint16_t chscan0 : 1;
  uint16_t chscan1 : 1;
  uint16_t chscan2 : 1;
  uint16_t chscan3 : 1;
  uint16_t chscan4 : 1;
  uint16_t chscan5 : 1;
  uint16_t chscan6 : 1;
  uint16_t chscan7 : 1;
  MAX11128_REG_IDENT cust_scan1 : 5;
} MAX11128_CUSTOM_SCAN1;
// Table 14. SampleSet Register
typedef struct
{
  uint16_t unused : 3;
  uint16_t seq_length : 8;
  MAX11128_REG_IDENT smpl_set : 5;
} MAX11128_SAMPLESET;
#define MAX11128_TO_SAMPLESET_FRAME(ch1, ch2, ch3, ch4) \
  ((ch1) << 12) | ((ch2) << 8) | ((ch3) << 4) | (ch4)

typedef union
{
  struct
  {
    uint16_t data : 12;
    uint16_t ch : 4;
  };
  uint16_t raw;
} MAX11128_DATA;

typedef union
{
  uint16_t raw;
  MAX11128_ADC_MODE_CTRL ctrl;
  MAX11128_ADC_CONFIG config;
  MAX11128_RANGE range;
  MAX11128_UNI uni;
  MAX11128_BIP bip;
  MAX11128_CUSTOM_SCAN0 cust_scan0;
  MAX11128_CUSTOM_SCAN1 cust_scan1;
  MAX11128_SAMPLESET smpl_set;
} MAX11128_REG;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // MAX11128_REGS_H
