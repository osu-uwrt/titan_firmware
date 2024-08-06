#ifndef BQ40Z80_REG_MAP_H_
#define BQ40Z80_REG_MAP_H_

#include <stdint.h>

#define sbs_check_bit(val, bit) (!!((val) & (1 << (bit))))

enum sbs_command {
    // Manufacturer Access Commands
    SBS_CMD_MANUFACTURER_ACCESS = 0x00,
    SBS_CMD_MANUFACTURER_BLOCK_ACCESS = 0x44,

    // SMBus SBS Standardized Word Commands
    SBS_CMD_REMAINING_CAPACITY_ALARM = 0x01,
    SBS_CMD_REMAINING_TIME_ALARM = 0x02,
    SBS_CMD_BATTERY_MODE = 0x03,
    SBS_CMD_AT_RATE = 0x04,
    SBS_CMD_AT_RATE_TIME_TO_FULL = 0x05,
    SBS_CMD_AT_RATE_TIME_TO_EMPTY = 0x06,
    SBS_CMD_AT_RATE_OK = 0x07,
    SBS_CMD_TEMPERATURE = 0x08,
    SBS_CMD_VOLTAGE = 0x09,
    SBS_CMD_CURRENT = 0x0A,
    SBS_CMD_AVERAGE_CURRENT = 0x0B,
    SBS_CMD_MAX_ERROR = 0x0C,
    SBS_CMD_RELATIVE_STATE_OF_CHARGE = 0x0D,
    SBS_CMD_ABSOLUTE_STATE_OF_CHARGE = 0x0E,
    SBS_CMD_REMAINING_CAPACITY = 0x0F,
    SBS_CMD_FULL_CHARGE_CAPACITY = 0x10,
    SBS_CMD_RUN_TIME_TO_EMPTY = 0x11,
    SBS_CMD_AVERAGE_TIME_TO_EMPTY = 0x12,
    SBS_CMD_AVERAGE_TIME_TO_FULL = 0x13,
    SBS_CMD_CHARGING_CURRENT = 0x14,
    SBS_CMD_CHARGING_VOLTAGE = 0x15,
    SBS_CMD_BATTERY_STATUS = 0x16,
    SBS_CMD_CYCLE_COUNT = 0x17,
    SBS_CMD_DESIGN_CAPACITY = 0x18,
    SBS_CMD_DESIGN_VOLTAGE = 0x19,
    SBS_CMD_SPECIFICATION_INFO = 0x1A,
    SBS_CMD_MANUFACTURER_DATE = 0x1B,
    SBS_CMD_SERIAL_NUMBER = 0x1C,

    // SMBus SBS Standardized Block Commands
    SBS_CMD_MANUFACTURER_NAME = 0x20,
    SBS_CMD_DEVICE_NAME = 0x21,
    SBS_CMD_DEVICE_CHEMISTRY = 0x22,
    SBS_CMD_MANUFACTURER_DATA = 0x23,
    SBS_CMD_AUTH_CHALLENGE = 0x27,
    SBS_CMD_AUTH_RESPONSE = 0x28,

    // TI Specific Word Commands
    SBS_CMD_CELL_VOLTAGE_7 = 0x3C,
    SBS_CMD_CELL_VOLTAGE_6 = 0x3D,
    SBS_CMD_CELL_VOLTAGE_5 = 0x3E,
    SBS_CMD_CELL_VOLTAGE_4 = 0x3F,
    SBS_CMD_ADC1_READ = 0x46,
    SBS_CMD_ADC2_READ = 0x47,
    SBS_CMD_GPIO_READ = 0x48,
    SBS_CMD_GPIO_WRITE = 0x49,
    SBS_CMD_BTP_DISCHARGE_SET = 0x4A,
    SBS_CMD_BTP_CHARGE_SET = 0x4B,
    SBS_CMD_STATE_OF_HEALTH = 0x4F,

    // TI Specific 32-bit Block Access
    SBS_CMD_SAFETY_ALERT = 0x50,
    SBS_CMD_SAFETY_STATUS = 0x51,
    SBS_CMD_PF_ALERT = 0x52,
    SBS_CMD_PF_STATUS = 0x53,
    SBS_CMD_OPERATION_STATUS = 0x54,
    SBS_CMD_CHARGING_STATUS = 0x55,
    SBS_CMD_GAUGING_STATUS = 0x56,
    SBS_CMD_MANUFACTURING_STATUS = 0x57,

    // TI Specific SMBus Long Block Access
    SBS_CMD_AFE_REGISTER = 0x58,

    // TI Specific Turbo Commands (Word Access)
    SBS_CMD_MAX_TURBO_PWR = 0x59,
    SBS_CMD_SUS_TURBO_PWR = 0x5A,
    SBS_CMD_TURBO_PACK_R = 0x5B,
    SBS_CMD_TURBO_SYS_R = 0x5C,
    SBS_CMD_TURBO_EDV = 0x5D,
    SBS_CMD_MAX_TURBO_CURR = 0x5E,
    SBS_CMD_SUS_TURBO_CURR = 0x5F,

    // Assorted Exposed Manufacturer Access Commands
    SBS_CMD_LIFETIME_DATA_BLOCK1 = 0x60,
    SBS_CMD_LIFETIME_DATA_BLOCK2 = 0x61,
    SBS_CMD_LIFETIME_DATA_BLOCK3 = 0x62,
    SBS_CMD_LIFETIME_DATA_BLOCK4 = 0x63,
    SBS_CMD_LIFETIME_DATA_BLOCK5 = 0x64,
    SBS_CMD_MANUFACTURER_INFO = 0x70,
    SBS_CMD_DA_STATUS1 = 0x71,
    SBS_CMD_DA_STATUS2 = 0x72,
    SBS_CMD_GAUGING_STATUS1 = 0x73,
    SBS_CMD_GAUGING_STATUS2 = 0x74,
    SBS_CMD_GAUGING_STATUS3 = 0x75,
    SBS_CMD_CB_STATUS = 0x76,
    SBS_CMD_STATE_OF_HEALTH_CAP = 0x77,
    SBS_CMD_FILTERED_CAPACITY = 0x78,
    SBS_CMD_MANUFACTURER_INFOB = 0x7A,
    SBS_CMD_DA_STATUS3 = 0x7B,
    SBS_CMD_GAUGING_STATUS4 = 0x7C,
    SBS_CMD_GAUGING_STATUS5 = 0x7D,
    SBS_CMD_MANUFACTURER_INFOC = 0x80,
    SBS_CMD_MANUFACTURER_INFOD = 0x81,
    SBS_CMD_CURRENT_LONG = 0x82
};

#define SBS_GPIO_READ_RC2 0
#define SBS_GPIO_READ_RC3 1
#define SBS_GPIO_READ_RH0 2
#define SBS_GPIO_READ_RH1 3
#define SBS_GPIO_READ_RH2 4
#define SBS_GPIO_READ_RL0 5
#define SBS_GPIO_READ_RL1 6
#define SBS_GPIO_READ_RL2 7

#define SBS_BATTERY_MODE_ICC 0
#define SBS_BATTERY_MODE_PBS 1
#define SBS_BATTERY_MODE_CF 7
#define SBS_BATTERY_MODE_CC 8
#define SBS_BATTERY_MODE_PB 9
#define SBS_BATTERY_MODE_AM 10
#define SBS_BATTERY_MODE_CHGM 14
#define SBS_BATTERY_MODE_CAPM 15

#define SBS_BATTERY_STATUS_EC_MASK 0xF
#define SBS_BATTERY_STATUS_EC0 0
#define SBS_BATTERY_STATUS_EC1 1
#define SBS_BATTERY_STATUS_EC2 2
#define SBS_BATTERY_STATUS_EC3 3
#define SBS_BATTERY_STATUS_FD 4
#define SBS_BATTERY_STATUS_FC 5
#define SBS_BATTERY_STATUS_DSG 6
#define SBS_BATTERY_STATUS_INIT 7
#define SBS_BATTERY_STATUS_RTA 8
#define SBS_BATTERY_STATUS_RCA 9
#define SBS_BATTERY_STATUS_TDA 11
#define SBS_BATTERY_STATUS_OTA 12
#define SBS_BATTERY_STATUS_TCA 14
#define SBS_BATTERY_STATUS_OCA 15

#define SBS_SAFETY_ALERT_OCDL 29
#define SBS_SAFETY_ALERT_COVL 28
#define SBS_SAFETY_ALERT_UTD 27
#define SBS_SAFETY_ALERT_UTC 26
#define SBS_SAFETY_ALERT_PCHGC 25
#define SBS_SAFETY_ALERT_CHGV 24
#define SBS_SAFETY_ALERT_CHGC 23
#define SBS_SAFETY_ALERT_OC 22
#define SBS_SAFETY_ALERT_CTOS 21
#define SBS_SAFETY_ALERT_CTO 20
#define SBS_SAFETY_ALERT_PTOS 19
#define SBS_SAFETY_ALERT_PTO 18
#define SBS_SAFETY_ALERT_OTF 16
#define SBS_SAFETY_ALERT_CUVC 14
#define SBS_SAFETY_ALERT_OTD 13
#define SBS_SAFETY_ALERT_OTC 12
#define SBS_SAFETY_ALERT_ASCDL 11
#define SBS_SAFETY_ALERT_ASCCL 9
#define SBS_SAFETY_ALERT_AOLDL 7
#define SBS_SAFETY_ALERT_OCD2 5
#define SBS_SAFETY_ALERT_OCD1 4
#define SBS_SAFETY_ALERT_OCC2 4
#define SBS_SAFETY_ALERT_OCC1 2
#define SBS_SAFETY_ALERT_COV 1
#define SBS_SAFETY_ALERT_CUV 0

#define SBS_SAFETY_STATUS_OCDL 29
#define SBS_SAFETY_STATUS_COVL 28
#define SBS_SAFETY_STATUS_UTD 27
#define SBS_SAFETY_STATUS_UTC 26
#define SBS_SAFETY_STATUS_PCHGC 25
#define SBS_SAFETY_STATUS_CHGV 24
#define SBS_SAFETY_STATUS_CHGC 23
#define SBS_SAFETY_STATUS_OC 22
#define SBS_SAFETY_STATUS_CTO 20
#define SBS_SAFETY_STATUS_PTO 18
#define SBS_SAFETY_STATUS_OTF 16
#define SBS_SAFETY_STATUS_CUVC 14
#define SBS_SAFETY_STATUS_OTD 13
#define SBS_SAFETY_STATUS_OTC 12
#define SBS_SAFETY_STATUS_ASCDL 11
#define SBS_SAFETY_STATUS_ASCD 10
#define SBS_SAFETY_STATUS_ASCCL 9
#define SBS_SAFETY_STATUS_ASCC 8
#define SBS_SAFETY_STATUS_AOLDL 7
#define SBS_SAFETY_STATUS_AOLD 6
#define SBS_SAFETY_STATUS_OCD2 5
#define SBS_SAFETY_STATUS_OCD1 4
#define SBS_SAFETY_STATUS_OCC2 3
#define SBS_SAFETY_STATUS_OCC1 2
#define SBS_SAFETY_STATUS_COV 1
#define SBS_SAFETY_STATUS_CUV 0

#define SBS_PF_ALERT_TS4 31
#define SBS_PF_ALERT_TS3 30
#define SBS_PF_ALERT_TS2 29
#define SBS_PF_ALERT_TS1 28
#define SBS_PF_ALERT_2LVL 22
#define SBS_PF_ALERT_AFEC 21
#define SBS_PF_ALERT_AFER 20
#define SBS_PF_ALERT_FUSE 19
#define SBS_PF_ALERT_OCDL 18
#define SBS_PF_ALERT_DFETF 17
#define SBS_PF_ALERT_CFETF 16
#define SBS_PF_ALERT_ASCDL 15
#define SBS_PF_ALERT_ASCCL 14
#define SBS_PF_ALERT_AOLDL 13
#define SBS_PF_ALERT_VIMA 12
#define SBS_PF_ALERT_VIMR 11
#define SBS_PF_ALERT_CD 10
#define SBS_PF_ALERT_IMP 9
#define SBS_PF_ALERT_CB 8
#define SBS_PF_ALERT_QIM 7
#define SBS_PF_ALERT_SOTF 6
#define SBS_PF_ALERT_COVL 5
#define SBS_PF_ALERT_SOT 4
#define SBS_PF_ALERT_SOCD 3
#define SBS_PF_ALERT_SOCC 2
#define SBS_PF_ALERT_SOV 1
#define SBS_PF_ALERT_SUV 0

#define SBS_PF_STATUS_TS4 31
#define SBS_PF_STATUS_TS3 30
#define SBS_PF_STATUS_TS2 29
#define SBS_PF_STATUS_TS1 28
#define SBS_PF_STATUS_DFW 26
#define SBS_PF_STATUS_IFC 24
#define SBS_PF_STATUS_PTC 23
#define SBS_PF_STATUS_2LVL 22
#define SBS_PF_STATUS_AFEC 21
#define SBS_PF_STATUS_AFER 20
#define SBS_PF_STATUS_FUSE 19
#define SBS_PF_STATUS_OCDL 18
#define SBS_PF_STATUS_DFETF 17
#define SBS_PF_STATUS_CFETF 16
#define SBS_PF_STATUS_ASCDL 15
#define SBS_PF_STATUS_ASCCL 14
#define SBS_PF_STATUS_AOLDL 13
#define SBS_PF_STATUS_VIMA 12
#define SBS_PF_STATUS_VIMR 11
#define SBS_PF_STATUS_CD 10
#define SBS_PF_STATUS_IMP 9
#define SBS_PF_STATUS_CB 8
#define SBS_PF_STATUS_QIM 7
#define SBS_PF_STATUS_SOTF 6
#define SBS_PF_STATUS_COVL 5
#define SBS_PF_STATUS_SOT 4
#define SBS_PF_STATUS_SOCD 3
#define SBS_PF_STATUS_SOCC 2
#define SBS_PF_STATUS_SOV 1
#define SBS_PF_STATUS_SUV 0

#define SBS_OPERATION_STATUS_IATA_CTERM 31
#define SBS_OPERATION_STATUS_EMSHUT 29
#define SBS_OPERATION_STATUS_CB 28
#define SBS_OPERATION_STATUS_SLPCC 27
#define SBS_OPERATION_STATUS_SLPAD 26
#define SBS_OPERATION_STATUS_SMBLCAL 25
#define SBS_OPERATION_STATUS_INIT 24
#define SBS_OPERATION_STATUS_SLEEPM 23
#define SBS_OPERATION_STATUS_XL 22
#define SBS_OPERATION_STATUS_CAL_OFFSET 21
#define SBS_OPERATION_STATUS_CAL 20
#define SBS_OPERATION_STATUS_AUTOCALM 19
#define SBS_OPERATION_STATUS_AUTH 18
#define SBS_OPERATION_STATUS_LED 17
#define SBS_OPERATION_STATUS_SDM 16
#define SBS_OPERATION_STATUS_SLEEP 15
#define SBS_OPERATION_STATUS_XCHG 14
#define SBS_OPERATION_STATUS_XDSG 13
#define SBS_OPERATION_STATUS_PF 12
#define SBS_OPERATION_STATUS_SS 11
#define SBS_OPERATION_STATUS_SDV 10
#define SBS_OPERATION_STATUS_SEC1 9
#define SBS_OPERATION_STATUS_SEC0 8
#define SBS_OPERATION_STATUS_BTP_INT 7
#define SBS_OPERATION_STATUS_FUSE 5
#define SBS_OPERATION_STATUS_PDSG 4
#define SBS_OPERATION_STATUS_PCHG 3
#define SBS_OPERATION_STATUS_CHG 2
#define SBS_OPERATION_STATUS_DSG 1
#define SBS_OPERATION_STATUS_PRES 0

#define SBS_CHARGING_STATUS_NCT 19
#define SBS_CHARGING_STATUS_CCC 18
#define SBS_CHARGING_STATUS_CVR 17
#define SBS_CHARGING_STATUS_CCR 16
#define SBS_CHARGING_STATUS_VCT 15
#define SBS_CHARGING_STATUS_MCHG 14
#define SBS_CHARGING_STATUS_SU 13
#define SBS_CHARGING_STATUS_IN 12
#define SBS_CHARGING_STATUS_HV 11
#define SBS_CHARGING_STATUS_MV 10
#define SBS_CHARGING_STATUS_LV 9
#define SBS_CHARGING_STATUS_PV 8
#define SBS_CHARGING_STATUS_OT 6
#define SBS_CHARGING_STATUS_HT 5
#define SBS_CHARGING_STATUS_STH 4
#define SBS_CHARGING_STATUS_RT 3
#define SBS_CHARGING_STATUS_STL 2
#define SBS_CHARGING_STATUS_LT 1
#define SBS_CHARGING_STATUS_UT 0

#define SBS_GAUGING_STATUS_OCVFR 20
#define SBS_GAUGING_STATUS_LDMD 19
#define SBS_GAUGING_STATUS_RX 18
#define SBS_GAUGING_STATUS_QMAX 17
#define SBS_GAUGING_STATUS_VDQ 16
#define SBS_GAUGING_STATUS_NSFM 15
#define SBS_GAUGING_STATUS_SLPQMAX 13
#define SBS_GAUGING_STATUS_QEN 12
#define SBS_GAUGING_STATUS_VOK 11
#define SBS_GAUGING_STATUS_R_DIS 10
#define SBS_GAUGING_STATUS_REST 8
#define SBS_GAUGING_STATUS_CF 7
#define SBS_GAUGING_STATUS_DSG 6
#define SBS_GAUGING_STATUS_EDV 5
#define SBS_GAUGING_STATUS_BAL_EN 4
#define SBS_GAUGING_STATUS_TC 3
#define SBS_GAUGING_STATUS_TD 2
#define SBS_GAUGING_STATUS_FC 1
#define SBS_GAUGING_STATUS_FD 0

#define SBS_MANUFACTURING_STATUS_CAL_EN 15
#define SBS_MANUFACTURING_STATUS_LT_TEST 14
#define SBS_MANUFACTURING_STATUS_PDSG_EN 13
#define SBS_MANUFACTURING_STATUS_LED_EN 9
#define SBS_MANUFACTURING_STATUS_FUSE_EN 8
#define SBS_MANUFACTURING_STATUS_BBR_EN 7
#define SBS_MANUFACTURING_STATUS_PF_EN 6
#define SBS_MANUFACTURING_STATUS_LF_EN 5
#define SBS_MANUFACTURING_STATUS_FET_EN 4
#define SBS_MANUFACTURING_STATUS_GAUGE_EN 3
#define SBS_MANUFACTURING_STATUS_DSG_EN 2
#define SBS_MANUFACTURING_STATUS_CHG_EN 1
#define SBS_MANUFACTURING_STATUS_PCHG_EN 0

enum sbs_mfg_commands {
    MFG_CMD_DEVICE_TYPE = 0x0001,
    MFG_CMD_FIRMWARE_VERSION = 0x0002,
    MFG_CMD_HARDWARE_VERSION = 0x0003,
    MFG_CMD_IF_CHECKSUM = 0x0004,
    MFG_CMD_STATIC_DF_SIGNATURE = 0x0005,
    MFG_CMD_CHEM_ID = 0x0006,
    MFG_CMD_STATIC_CHEM_DF_SIGNATURE = 0x0008,
    MFG_CMD_ALL_DF_SIGNATURE = 0x0009,
    MFG_CMD_SHUTDOWN_MODE = 0x0010,
    MFG_CMD_SLEEP_MODE = 0x0011,
    MFG_CMD_AUTO_CC_OFSET = 0x0013,
    MFG_CMD_PRE_DISCHARGE_FET_TOGGLE = 0x001C,
    MFG_CMD_FUSE_TOGGLE = 0x001D,
    MFG_CMD_PRECHARGE_FET_TOGGLE = 0x001E,
    MFG_CMD_CHARGE_FET_TOGGLE = 0x001F,
    MFG_CMD_DISCHARGE_FET_TOGGLE = 0x0020,
    MFG_CMD_GAUGING = 0x0021,
    MFG_CMD_FET_CONTROL = 0x0022,
    MFG_CMD_LIFETIME_DATA_COLLECTION = 0x0023,
    MFG_CMD_PERMANENT_FAILURE = 0x0024,
    MFG_CMD_BLACK_BOX_RECORDER = 0x0025,
    MFG_CMD_FUSE = 0x0026,
    MFG_CMD_LED_DISPLAY_ENABLE = 0x0027,
    MFG_CMD_LIFETIME_DATA_RESET = 0x0028,
    MFG_CMD_PERMANENT_FAILURE_DATA_RESET = 0x0029,
    MFG_CMD_BLACK_BOX_RECORDER_RESET = 0x002A,
    MFG_CMD_LED_TOGGLE = 0x002B,
    MFG_CMD_LED_DISPLAY_PRESS = 0x002C,
    MFG_CMD_CALIBRATION_MODE = 0x002D,
    MFG_CMD_LIFETIME_DATA_FLUSH = 0x002E,
    MFG_CMD_LIFETIME_DATA_SPEED_UP_MODE = 0x002F,
    MFG_CMD_SEAL_DEVICE = 0x0030,
    MFG_CMD_SECURITY_KEYS = 0x0035,
    MFG_CMD_AUTHENTICATION_KEY = 0x0037,
    MFG_CMD_DEVICE_RESET = 0x0041,
    MFG_CMD_SAFETY_ALERT = 0x0050,
    MFG_CMD_SAFETY_STATUS = 0x0051,
    MFG_CMD_PF_ALERT = 0x0052,
    MFG_CMD_PF_STATUS = 0x0053,
    MFG_CMD_OPERATION_STATUS = 0x0054,
    MFG_CMD_CHARGING_STATUS = 0x0055,
    MFG_CMD_GAUGING_STATUS = 0x0056,
    MFG_CMD_MANUFACTURING_STATUS = 0x0057,
    MFG_CMD_AFE_REGISTER = 0x0058,
    MFG_CMD_NO_LOAD_REM_CAP = 0x005A,
    MFG_CMD_LIFETIME_DATA_BLOCK1 = 0x0060,
    MFG_CMD_LIFETIME_DATA_BLOCK2 = 0x0061,
    MFG_CMD_LIFETIME_DATA_BLOCK3 = 0x0062,
    MFG_CMD_LIFETIME_DATA_BLOCK4 = 0x0063,
    MFG_CMD_LIFETIME_DATA_BLOCK5 = 0x0064,
    MFG_CMD_MANUFACTURER_INFO = 0x0070,
    MFG_CMD_DA_STATUS1 = 0x0071,
    MFG_CMD_DA_STATUS2 = 0x0072,
    MFG_CMD_GAUGING_STATUS1 = 0x0073,
    MFG_CMD_GAUGING_STATUS2 = 0x0074,
    MFG_CMD_GAUGING_STATUS3 = 0x0075,
    MFG_CMD_CB_STATUS = 0x0076,
    MFG_CMD_STATEOF_HEALTH = 0x0077,
    MFG_CMD_FILTER_CAPACITY = 0x0078,
    MFG_CMD_RSOC_WRITE = 0x0079,
    MFG_CMD_MANUFACTURER_INFO_B = 0x007A,
    MFG_CMD_DA_STATUS3 = 0x007B,
    MFG_CMD_GAUGING_STATUS4 = 0x007C,
    MFG_CMD_GAUGING_STATUS5 = 0x007D,
    MFG_CMD_MANUFACTURER_INFO_C = 0x0080,
    MFG_CMD_MANUFACTURER_INFO_D = 0x0081,
    MFG_CMD_CURRENT_LONG = 0x0082,
    MFG_CMD_IATA_SHUTDOWN = 0x00F0,
    MFG_CMD_IATA_RM = 0x00F1,
    MFG_CMD_IATA_FCC = 0x00F2,
    MFG_CMD_IATA_CHARGE = 0x00F3,
    MFG_CMD_MFC_ENABLE_B = 0x043D,
    MFG_CMD_ROM_MODE = 0x0F00,
    MFG_CMD_MFC_DISABLE = 0x23A7,
    MFG_CMD_MFC_ENABLE_A = 0x270C,
    MFG_CMD_EXIT_CALIBRATION_OUTPUT = 0xF080,
    MFG_CMD_OUTPUT_CCADC_CAL = 0xF081,
    MFG_CMD_OUTPUT_SHORTED_CCADC_CAL = 0xF082,
    MFG_CMD_OUTPUT_CELL7_CCADC_CAL = 0xF083,
};

struct sbs_afe_register {
    uint8_t afe_interrupt_status;
    uint8_t afe_fet_status;
    uint8_t afe_rxin;
    uint8_t afe_latch_status;
    uint8_t afe_interrupt_enable;
    uint8_t afe_control;
    uint8_t afe_rxien;
    uint8_t afe_rlout;
    uint8_t afe_rhout;
    uint8_t afe_rhint;
    uint8_t afe_cell_balance;
    uint8_t afe_adc_cc_ctrl;
    uint8_t afe_adc_mux_ctrl;
    uint8_t afe_led_ctrl;
    uint8_t afe_ctrl;
    uint8_t afe_timer_ctrl;
    uint8_t afe_protection;
    uint8_t afe_ocd;
    uint8_t afe_scc;
    uint8_t afe_scd1;
    uint8_t afe_scd2;
};

struct sbs_lifetime_data_block1 {
    uint16_t cell1_max_voltage;
    uint16_t cell2_max_voltage;
    uint16_t cell3_max_voltage;
    uint16_t cell4_max_voltage;
    uint16_t cell5_max_voltage;
    uint16_t cell6_max_voltage;
    uint16_t cell7_max_voltage;
    uint16_t cell1_min_voltage;
    uint16_t cell2_min_voltage;
    uint16_t cell3_min_voltage;
    uint16_t cell4_min_voltage;
    uint16_t cell5_min_voltage;
    uint16_t cell6_min_voltage;
    uint16_t cell7_min_voltage;
    uint16_t max_delta_cell_voltage;
};

struct sbs_lifetime_data_block2 {
    uint16_t max_charge_current;
    uint16_t max_discharge_current;
    uint16_t max_avg_dsg_current;
    uint16_t max_avg_dsg_power;
    uint8_t max_temp_cell;
    uint8_t min_temp_cell;
    uint8_t max_delta_cell_temp;
    uint8_t max_temp_int_sensor;
    uint8_t min_temp_int_sensor;
    uint8_t max_temp_fet;
    uint8_t num_shutdowns;
    uint8_t num_partial_resets;
    uint8_t num_full_resets;
    uint8_t num_wdt_resets;
    uint8_t cb_time_cell1;
    uint8_t cb_time_cell2;
    uint8_t cb_time_cell3;
    uint8_t cb_time_cell4;
    uint8_t cb_time_cell5;
    uint8_t cb_time_cell6;
    uint8_t cb_time_cell7;
};

struct sbs_lifetime_data_block3 {
    uint16_t total_fw_runtime;
    uint16_t time_spent_ut;
    uint16_t time_spent_lt;
    uint16_t time_spent_stl;
    uint16_t tmie_spent_rt;
    uint16_t time_spent_sth;
    uint16_t time_spent_ht;
    uint16_t time_spent_ot;
};

struct sbs_lifetime_data_block4 {
    uint16_t num_cov_events;
    uint16_t last_cov_event;
    uint16_t num_cuv_events;
    uint16_t last_cuv_event;
    uint16_t num_ocd1_events;
    uint16_t last_ocd1_event;
    uint16_t num_ocd2_events;
    uint16_t last_ocd2_event;
    uint16_t num_occ1_events;
    uint16_t last_occ1_event;
    uint16_t num_occ2_events;
    uint16_t last_occ2_event;
    uint16_t num_old_events;
    uint16_t last_aold_event;
    uint16_t num_ascd_events;
    uint16_t last_ascd_event;
};

struct sbs_lifetime_data_block5 {
    uint16_t num_ascc_events;
    uint16_t last_ascc_event;
    uint16_t num_otc_events;
    uint16_t last_otc_event;
    uint16_t num_otd_events;
    uint16_t last_otd_event;
    uint16_t num_otf_events;
    uint16_t last_otf_event;
    uint16_t num_valid_charge_term;
    uint16_t last_valid_charge_term;
    uint16_t num_qmax_updates;
    uint16_t last_qmax_update;
    uint16_t num_ra_updates;
    uint16_t last_ra_update;
    uint16_t num_ra_disable;
    uint16_t last_ra_disable;
};

struct sbs_da_status1 {
    uint16_t cell1_voltage;
    uint16_t cell2_voltage;
    uint16_t cell3_voltage;
    uint16_t cell4_voltage;
    uint16_t batt_voltage;
    uint16_t pack_voltage;
    int16_t cell1_current;
    int16_t cell2_current;
    int16_t cell3_current;
    int16_t cell4_current;
    int16_t cell1_power;
    int16_t cell2_power;
    int16_t cell3_power;
    int16_t cell4_power;
    int16_t instant_power;
    int16_t avg_power;
};

struct sbs_da_status2 {
    uint16_t int_temperature;
    uint16_t ts1_temperature;
    uint16_t ts2_temperature;
    uint16_t ts3_temperature;
    uint16_t ts4_temperature;
    uint16_t cell_temperature;
    uint16_t fet_temperature;
    uint16_t gauging_temperature;
};

struct sbs_gauging_status1 {
    uint16_t true_rem_q;
    uint16_t true_rem_e;
    uint16_t initial_q;
    uint16_t initial_e;
    uint16_t true_fcc_q;
    uint16_t true_fcc_e;
    uint16_t t_sim;
    uint16_t t_ambient;
    uint16_t ra_scale0;
    uint16_t ra_scale1;
    uint16_t ra_scale2;
    uint16_t ra_scale3;
    uint16_t comp_res0;
    uint16_t comp_res1;
    uint16_t comp_res2;
    uint16_t comp_res3;
};

struct sbs_gauging_status2 {
    uint8_t pack_grid;
    uint8_t lstatus;
    uint8_t cell_grid0;
    uint8_t cell_grid1;
    uint8_t cell_grid2;
    uint8_t cell_grid3;
    uint32_t state_time;
    uint16_t dod0_0;
    uint16_t dod0_1;
    uint16_t dod0_2;
    uint16_t dod0_3;
    uint16_t dod0_passed_q;
    uint16_t dod0_passed_e;
    uint16_t dod0_time;
    uint16_t dodeoc0;
    uint16_t dodeoc1;
    uint16_t dodeoc2;
    uint16_t dodeoc3;
};

struct sbs_gauging_status3 {
    uint16_t qmax0;
    uint16_t qmax1;
    uint16_t qmax2;
    uint16_t qmax3;
    uint16_t qmax_dod0_0;
    uint16_t qmax_dod0_1;
    uint16_t qmax_dod0_2;
    uint16_t qmax_dod0_3;
    uint16_t qmax_passed_q;
    uint16_t qmax_time;
    uint16_t temp_k;
    uint16_t temp_a;
    uint16_t raw_dod0_0;
    uint16_t raw_dod0_1;
    uint16_t raw_dod0_2;
    uint16_t raw_dod0_3;
};

struct sbs_cb_status {
    uint16_t cell_balance_time0;
    uint16_t cell_balance_time1;
    uint16_t cell_balance_time2;
    uint16_t cell_balance_time3;
    uint16_t cell_balance_time4;
    uint16_t cell_balance_time5;
    uint16_t cell_balance_time6;
    uint16_t dod_balance_cell1;
    uint16_t dod_balance_cell2;
    uint16_t dod_balance_cell3;
    uint16_t dod_balance_cell4;
    uint16_t dod_balance_cell5;
    uint16_t dod_balance_cell6;
    uint16_t dod_balance_cell7;
    uint16_t total_dod_charge;
};

struct sbs_state_of_health {
    uint16_t state_of_health_fcc;
    uint16_t state_of_health_energy;
};

struct sbs_filter_capacity {
    uint16_t filtered_remaining_capacity;
    uint16_t filtered_remaining_energy;
    uint16_t filtered_full_charge_capacity;
    uint16_t filtered_full_charge_energy;
};

struct sbs_da_status3 {
    uint16_t cell5_voltage;
    int16_t cell5_current;
    int16_t cell5_power;
    uint16_t cell6_voltage;
    int16_t cell6_current;
    int16_t cell6_power;
    uint16_t cell7_voltage;
    int16_t cell7_current;
    int16_t cell7_power;
};

struct sbs_gauging_status4 {
    uint16_t rascale4;
    uint16_t compres4;
    uint16_t dod0_4;
    uint16_t dodeoc4;
    uint16_t qmax4;
    uint16_t qmax_dod0_4;
    uint16_t cell_raw_dod0_4;
    uint8_t cell_grid4;
    uint16_t rascale5;
    uint16_t compres5;
    uint16_t dod0_5;
    uint16_t dodeoc5;
    uint16_t qmax5;
    uint16_t qmax_dod0_5;
    uint16_t cell_raw_dod0_5;
    uint8_t cell_grid5;
};

struct gauging_status5 {
    uint16_t rascale6;
    uint16_t compres6;
    uint16_t dod0_6;
    uint16_t dodeoc6;
    uint16_t qmax6;
    uint16_t qmax_dod0_6;
    uint16_t cell_raw_dod0_6;
    uint8_t cell_grid6;
};

#endif
