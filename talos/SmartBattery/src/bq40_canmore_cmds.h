// NOTE THIS FILE IS INCLUDED IN CORE1.C
// THIS IS SEPARATED INTO A DIFFERENT FILE TO KEEP CLUTTER OUT

// ============================================
// Register Map Definitions
// ============================================

static const char *const batt_state_names[] = {
    [BATT_STATE_UNINITIALIZED] = "Uninitialized",
    [BATT_STATE_DISCONNECTED] = "Disconnected",
    [BATT_STATE_REMOVED] = "Removed",
    [BATT_STATE_NEEDS_SHUTDOWN] = "Shutting Off",
    [BATT_STATE_INITIALIZING] = "Initializing",
    [BATT_STATE_XDSG] = "Dsg Inhibited",
    [BATT_STATE_DISCHARGING] = "Discharging",
    [BATT_STATE_CHARGING] = "Charging",
    [BATT_STATE_PERMENANT_FAIL] = "Permanent Fail",
    [BATT_STATE_POWER_CYCLE] = "Power Cycling",
    [BATT_STATE_LATCH_OFF] = "Latched Off",
};
static const size_t batt_state_name_count = (sizeof(batt_state_names) / sizeof(*batt_state_names));

static const char *const manufacturing_status_names[] = {
    [SBS_MANUFACTURING_STATUS_PCHG_EN] = "Precharge FET Test [PCHG_EN]",
    [SBS_MANUFACTURING_STATUS_CHG_EN] = "Charge FET Test [CHG_EN]",
    [SBS_MANUFACTURING_STATUS_DSG_EN] = "Discharge FET Test [DSG_EN]",
    [SBS_MANUFACTURING_STATUS_GAUGE_EN] = "Gas Gauging [GAUGE_EN]",
    [SBS_MANUFACTURING_STATUS_FET_EN] = "All FET Action [FET_EN]",
    [SBS_MANUFACTURING_STATUS_LF_EN] = "Lifetime Data Collection [LF_EN]",
    [SBS_MANUFACTURING_STATUS_PF_EN] = "Permanent Failure [PF_EN]",
    [SBS_MANUFACTURING_STATUS_BBR_EN] = "Black Box Recorder [BBR_EN]",
    [SBS_MANUFACTURING_STATUS_FUSE_EN] = "Fuse Action [FUSE_EN]",
    [SBS_MANUFACTURING_STATUS_LED_EN] = "LED Display [LED_EN]",
    [SBS_MANUFACTURING_STATUS_PDSG_EN] = "Pre-discharge FET test [PDSG_EN]",
    [SBS_MANUFACTURING_STATUS_LT_TEST] = "Lifetime Speed Up [LT_TEST]",
    [SBS_MANUFACTURING_STATUS_CAL_EN] = "Calibration Mode [CAL_EN]",
};
static const size_t manufacturing_status_name_count =
    (sizeof(manufacturing_status_names) / sizeof(*manufacturing_status_names));

static const char *const safety_status_names[] = {
    [SBS_SAFETY_STATUS_OCDL] = "Overcurrent in discharge [OCDL]",
    [SBS_SAFETY_STATUS_COVL] = "Cell overvoltage latch [COVL]",
    [SBS_SAFETY_STATUS_UTD] = "Undertemperature during discharge [UTD]",
    [SBS_SAFETY_STATUS_UTC] = "Undertemperature during charge [UTC]",
    [SBS_SAFETY_STATUS_PCHGC] = "Over-precharge current [PCHGC]",
    [SBS_SAFETY_STATUS_CHGV] = "Overcharging voltage [CHGV]",
    [SBS_SAFETY_STATUS_CHGC] = "Overcharging current [CHGC]",
    [SBS_SAFETY_STATUS_OC] = "Overcharge [OC]",
    [SBS_SAFETY_STATUS_CTO] = "Charge timeout [CTO]",
    [SBS_SAFETY_STATUS_PTO] = "Precharge timeout [PTO]",
    [SBS_SAFETY_STATUS_OTF] = "Overtemperature FET [OTF]",
    [SBS_SAFETY_STATUS_CUVC] = "Cell undervoltage compensated [CUVC]",
    [SBS_SAFETY_STATUS_OTD] = "Overtemperature during discharge [OTD]",
    [SBS_SAFETY_STATUS_OTC] = "Overtemperature during charge [OTC]",
    [SBS_SAFETY_STATUS_ASCDL] = "Short-circuit during discharge latch [ASCDL]",
    [SBS_SAFETY_STATUS_ASCD] = "Short-circuit during discharge [ASCD]",
    [SBS_SAFETY_STATUS_ASCCL] = "Short-circuit during charge latch [ASCCL]",
    [SBS_SAFETY_STATUS_ASCC] = "Short-circuit during charge [ASCC]",
    [SBS_SAFETY_STATUS_AOLDL] = "Overload during discharge latch [AOLDL]",
    [SBS_SAFETY_STATUS_AOLD] = "Overload during discharge [AOLD]",
    [SBS_SAFETY_STATUS_OCD2] = "Overcurrent during discharge 2 [OCD2]",
    [SBS_SAFETY_STATUS_OCD1] = "Overcurrent during discharge 1 [OCD1]",
    [SBS_SAFETY_STATUS_OCC2] = "Overcurrent during charge 2 [OCC2]",
    [SBS_SAFETY_STATUS_OCC1] = "Overcurrent during charge 1 [OCC1]",
    [SBS_SAFETY_STATUS_COV] = "Cell overvoltage [COV]",
    [SBS_SAFETY_STATUS_CUV] = "Cell undervoltage [CUV]",
};
static const size_t safety_status_name_count = (sizeof(safety_status_names) / sizeof(*safety_status_names));

static const char *const pf_status_names[] = {
    [SBS_PF_STATUS_TS4] = "Open thermistor–TS4 failure [TS4]",
    [SBS_PF_STATUS_TS3] = "Open thermistor–TS3 failure [TS3]",
    [SBS_PF_STATUS_TS2] = "Open thermistor–TS2 failure [TS2]",
    [SBS_PF_STATUS_TS1] = "Open thermistor–TS1 failure [TS1]",
    [SBS_PF_STATUS_DFW] = "Data flash wearout failure [DFW]",
    [SBS_PF_STATUS_IFC] = "Instruction flash checksum failure [IFC]",
    [SBS_PF_STATUS_PTC] = "PTC failure [PTC]",
    [SBS_PF_STATUS_2LVL] = "Second level protector failure [2LVL]",
    [SBS_PF_STATUS_AFEC] = "AFE communication failure [AFEC]",
    [SBS_PF_STATUS_AFER] = "AFE register failure [AFER]",
    [SBS_PF_STATUS_FUSE] = "Chemical fuse failure [FUSE]",
    [SBS_PF_STATUS_OCDL] = "Overcurrent in discharge [OCDL]",
    [SBS_PF_STATUS_DFETF] = "Discharge FET failure [DFETF]",
    [SBS_PF_STATUS_CFETF] = "Charge FET failure [CFETF]",
    [SBS_PF_STATUS_ASCDL] = "Short circuit in discharge [ASCDL]",
    [SBS_PF_STATUS_ASCCL] = "Short circuit in charge [ASCCL]",
    [SBS_PF_STATUS_AOLDL] = "Overload in discharge [AOLDL]",
    [SBS_PF_STATUS_VIMA] = "Voltage imbalance while pack is active failure [VIMA]",
    [SBS_PF_STATUS_VIMR] = "Voltage imbalance while pack at rest failure [VIMR]",
    [SBS_PF_STATUS_CD] = "Capacity degradation failure [CD]",
    [SBS_PF_STATUS_IMP] = "Impedance failure [IMP]",
    [SBS_PF_STATUS_CB] = "Cell balancing failure [CB]",
    [SBS_PF_STATUS_QIM] = "QMax imbalance failure [QIM]",
    [SBS_PF_STATUS_SOTF] = "Safety overtemperature FET failure [SOTF]",
    [SBS_PF_STATUS_COVL] = "Cell overvoltage latch [COVL]",
    [SBS_PF_STATUS_SOT] = "Safety overtemperature cell failure [SOT]",
    [SBS_PF_STATUS_SOCD] = "Safety overcurrent in discharge [SOCD]",
    [SBS_PF_STATUS_SOCC] = "Safety overcurrent in charge [SOCC]",
    [SBS_PF_STATUS_SOV] = "Safety cell overvoltage failure [SOV]",
    [SBS_PF_STATUS_SUV] = "Safety cell undervoltage failure [SUV]",
};
static const size_t pf_status_name_count = (sizeof(pf_status_names) / sizeof(*pf_status_names));

static const char *const battery_status_names[] = {
    [SBS_BATTERY_STATUS_EC0] = "Error Code Bit 0 [EC0]",
    [SBS_BATTERY_STATUS_EC1] = "Error Code Bit 1 [EC1]",
    [SBS_BATTERY_STATUS_EC2] = "Error Code Bit 2 [EC2]",
    [SBS_BATTERY_STATUS_EC3] = "Error Code Bit 3 [EC3]",
    [SBS_BATTERY_STATUS_FD] = "Fully Discharged [FD]",
    [SBS_BATTERY_STATUS_FC] = "Fully Charged [FC]",
    [SBS_BATTERY_STATUS_DSG] = "Discharging/Relax Mode [DSG]",
    [SBS_BATTERY_STATUS_INIT] = "Initialization [INIT]",
    [SBS_BATTERY_STATUS_RTA] = "Remaining Time Alarm [RTA]",
    [SBS_BATTERY_STATUS_RCA] = "Remaining Capacity Alarm [RCA]",
    [SBS_BATTERY_STATUS_TDA] = "Terminate Discharge Alarm [TDA]",
    [SBS_BATTERY_STATUS_OTA] = "Overtemperature Alarm [OTA]",
    [SBS_BATTERY_STATUS_TCA] = "Terminate Charge Alarm [TCA]",
    [SBS_BATTERY_STATUS_OCA] = "Overcharged Alarm [OCA]",
};
static const size_t battery_status_name_count = (sizeof(battery_status_names) / sizeof(*battery_status_names));

static const char *const operation_status_names[] = {
    [SBS_OPERATION_STATUS_IATA_CTERM] = "IATA charge control [IATA_CTERM]",
    [SBS_OPERATION_STATUS_EMSHUT] = "Emergency FET shutdown [EMSHUT]",
    [SBS_OPERATION_STATUS_CB] = "Cell balancing status [CB]",
    [SBS_OPERATION_STATUS_SLPCC] = "CC measurement in SLEEP mode [SLPCC]",
    [SBS_OPERATION_STATUS_SLPAD] = "ADC measurement in SLEEP mode [SLPAD]",
    [SBS_OPERATION_STATUS_INIT] = "Initialization after full reset [INIT]",
    [SBS_OPERATION_STATUS_SLEEPM] = "SLEEP mode triggered via command [SLEEPM]",
    [SBS_OPERATION_STATUS_XL] = "400-kHz SMBus mode [XL]",
    [SBS_OPERATION_STATUS_CAL_OFFSET] = "CAL_OFFSET",
    [SBS_OPERATION_STATUS_CAL] = "CAL",
    [SBS_OPERATION_STATUS_AUTOCALM] = "AUTOCALM",
    [SBS_OPERATION_STATUS_AUTH] = "Auth in progress [AUTH]",
    [SBS_OPERATION_STATUS_LED] = "LED Display [LED]",
    [SBS_OPERATION_STATUS_SDM] = "Shutdown triggered via command [SDM]",
    [SBS_OPERATION_STATUS_SLEEP] = "SLEEP mode conditions met [SLEEP]",
    [SBS_OPERATION_STATUS_XCHG] = "Charging disabled [XCHG]",
    [SBS_OPERATION_STATUS_XDSG] = "Discharging disabled [XDSG]",
    [SBS_OPERATION_STATUS_PF] = "PERMANENT FAILURE status [PF]",
    [SBS_OPERATION_STATUS_SS] = "SAFETY status [SS]",
    [SBS_OPERATION_STATUS_SDV] = "Shutdown triggered from low batt voltage [SDV]",
    [SBS_OPERATION_STATUS_SEC1] = "Security Mdoe Bit 1 [SEC1]",
    [SBS_OPERATION_STATUS_SEC0] = "Security Mode Bit 0 [SEC0]",
    [SBS_OPERATION_STATUS_BTP_INT] = "Battery trip point interrupt [BTP_INT]",
    [SBS_OPERATION_STATUS_FUSE] = "Fuse status [FUSE]",
    [SBS_OPERATION_STATUS_PDSG] = "Pre-discharge FET status [PDSG]",
    [SBS_OPERATION_STATUS_PCHG] = "Precharge FET status [PCHG]",
    [SBS_OPERATION_STATUS_CHG] = "CHG FET status [CHG]",
    [SBS_OPERATION_STATUS_DSG] = "DSG FET status [DSG]",
    [SBS_OPERATION_STATUS_PRES] = "System present low [PRES]",
};
static const size_t operation_status_name_count = (sizeof(operation_status_names) / sizeof(*operation_status_names));

static const char *const charging_status_names[] = {
    [SBS_CHARGING_STATUS_NCT] = "Near charge termination [NCT]",
    [SBS_CHARGING_STATUS_CCC] = "Charging loss compensation [CCC]",
    [SBS_CHARGING_STATUS_CVR] = "Charging voltage rate of change [CVR]",
    [SBS_CHARGING_STATUS_CCR] = "Charging current rate of change [CCR]",
    [SBS_CHARGING_STATUS_VCT] = "Charge termination [VCT]",
    [SBS_CHARGING_STATUS_MCHG] = "Maintenance charge [MCHG]",
    [SBS_CHARGING_STATUS_SU] = "Suspend charge [SU]",
    [SBS_CHARGING_STATUS_IN] = "Charge inhibit [IN]",
    [SBS_CHARGING_STATUS_HV] = "High voltage region [HV]",
    [SBS_CHARGING_STATUS_MV] = "Mid voltage region [MV]",
    [SBS_CHARGING_STATUS_LV] = "Low voltage region [LV]",
    [SBS_CHARGING_STATUS_PV] = "Precharge voltage region [PV]",
    [SBS_CHARGING_STATUS_OT] = "Overtemperature region [OT]",
    [SBS_CHARGING_STATUS_HT] = "High temperature region [HT]",
    [SBS_CHARGING_STATUS_STH] = "Standard temperature high region [STH]",
    [SBS_CHARGING_STATUS_RT] = "Recommended temperature region [RT]",
    [SBS_CHARGING_STATUS_STL] = "Standard temperature low region [STL]",
    [SBS_CHARGING_STATUS_LT] = "Low temperature region [LT]",
    [SBS_CHARGING_STATUS_UT] = "Undertemperature region [UT]",
};
static const size_t charging_status_name_count = (sizeof(charging_status_names) / sizeof(*charging_status_names));

static const char *const gauging_status_names[] = {
    [SBS_GAUGING_STATUS_OCVFR] = "Open circuit voltage in flat region (during RELAX) [OCVFR]",
    [SBS_GAUGING_STATUS_LDMD] = "LOAD mode [LDMD]",
    [SBS_GAUGING_STATUS_RX] = "Resistance update (toggles after every resistance update) [RX]",
    [SBS_GAUGING_STATUS_QMAX] = "QMax update (toggles after every QMax update) [QMAX]",
    [SBS_GAUGING_STATUS_VDQ] = "Discharge qualified for learning (opposite of the R_DIS flag) [VDQ]",
    [SBS_GAUGING_STATUS_NSFM] = "Negative scale factor mode [NSFM]",
    [SBS_GAUGING_STATUS_SLPQMAX] = "OCV update in SLEEP mode [SLPQMAX]",
    [SBS_GAUGING_STATUS_QEN] = "Impedance Track Gauging (Ra and QMax updates are enabled) [QEN]",
    [SBS_GAUGING_STATUS_VOK] = "Voltages are OK for QMax update. This flag is updated at exit of the RELAX mode. [VOK]",
    [SBS_GAUGING_STATUS_R_DIS] = "Resistance updates [R_DIS]",
    [SBS_GAUGING_STATUS_REST] = "Rest [REST]",
    [SBS_GAUGING_STATUS_CF] = "Condition Flag [CF]",
    [SBS_GAUGING_STATUS_DSG] = "Discharge/relax [DSG]",
    [SBS_GAUGING_STATUS_EDV] = "End-of-discharge termination voltage [EDV]",
    [SBS_GAUGING_STATUS_BAL_EN] = "Cell balancing [BAL_EN]",
    [SBS_GAUGING_STATUS_TC] = "Terminate charge [TC]",
    [SBS_GAUGING_STATUS_TD] = "Terminate discharge [TD]",
    [SBS_GAUGING_STATUS_FC] = "Fully charged [FC]",
    [SBS_GAUGING_STATUS_FD] = "Fully discharged [FD]",
};
static const size_t gauging_status_name_count = (sizeof(gauging_status_names) / sizeof(*gauging_status_names));

// ============================================
// Utility Functions
// ============================================

static void canmore_print_bit_array(FILE *fout, uint32_t val, const char *const *name_array, size_t name_arr_len,
                                    bool list_all) {
    for (unsigned int i = 0; i < 32; i++) {
        char bit_val = (val >> i) & 1;
        if (list_all) {
            if (i < name_arr_len && name_array[i]) {
                fprintf(fout, " - %s: %d\n", name_array[i], bit_val);
            }
            else if (bit_val) {
                // Report any unknown set bits
                fprintf(fout, " - Unknown Bit %d: %d\n", i, bit_val);
            }
        }
        else {
            if (!bit_val)
                continue;

            if (i < name_arr_len && name_array[i]) {
                fprintf(fout, " - %s\n", name_array[i]);
            }
            else {
                fprintf(fout, " - Unknown (%d)\n", i);
            }
        }
    }
}

static void bq40_decode_err(bq_error_t err, FILE *fout) {
    fprintf(fout, "Error Executing Command: [bq40z80.c:%d] ", err.fields.line);
    const char *nested_error;

    switch (err.fields.error_code) {
    case BQ_ERROR_SUCCESS:
        fprintf(fout, "BQ_ERROR_SUCCESS\n");
        break;
    case BQ_ERROR_INVALID_DEVICE_TYPE:
        fprintf(fout, "BQ_ERROR_INVALID_DEVICE_TYPE (Found 0x%04X instead)\n", err.fields.arg);
        break;
    case BQ_ERROR_INVALID_SERIAL:
        fprintf(fout, "BQ_ERROR_SERIAL_CHANGED (Was %d, Now %d)\n", shared_status.mfg_info.serial, err.fields.arg);
        break;
    case BQ_ERROR_INVALID_STATE:
        fprintf(fout, "BQ_ERROR_INVALID_STATE\n");
        break;
    case BQ_ERROR_INVALID_MAC_RESP_LEN:
        fprintf(fout, "BQ_ERROR_INVALID_MAC_RESP_LEN (got %d instead)\n", err.fields.arg);
        break;
    case BQ_ERROR_BATT_STATUS_ERROR:
        switch (err.fields.arg) {
        case 0x00:
            nested_error = "Ok";
            break;
        case 0x01:
            nested_error = "Busy";
            break;
        case 0x02:
            nested_error = "Reserved Command";
            break;
        case 0x03:
            nested_error = "Unsupported Command";
            break;
        case 0x04:
            nested_error = "Access Denied";
            break;
        case 0x05:
            nested_error = "Overflow/Underflow";
            break;
        case 0x06:
            nested_error = "Bad Size";
            break;
        case 0x07:
            nested_error = "Unknown Error";
            break;
        default:
            nested_error = "Invalid Error";
            break;
        }
        fprintf(fout, "BQ_ERROR_BATT_STATUS_ERROR (BQ40 returned error: %s)\n", nested_error);
        break;
    case PIO_SMBUS_ERR_ADDR_NAK:
        fprintf(fout, "PIO_SMBUS_ERR_ADDR_NAK\n");
        break;
    case PIO_SMBUS_ERR_ADDR_RESTART_NAK:
        fprintf(fout, "PIO_SMBUS_ERR_ADDR_RESTART_NAK\n");
        break;
    case PIO_SMBUS_ERR_NAK:
        fprintf(fout, "PIO_SMBUS_ERR_NAK\n");
        break;
    case PIO_SMBUS_ERR_TIMEOUT:
        fprintf(fout, "PIO_SMBUS_ERR_TIMEOUT\n");
        break;
    case PIO_SMBUS_ERR_BUS_STUCK_LOW:
        fprintf(fout, "PIO_SMBUS_ERR_BUS_STUCK_LOW\n");
        break;
    case PIO_SMBUS_ERR_ABRITRATION_LOST:
        fprintf(fout, "PIO_SMBUS_ERR_ABRITRATION_LOST\n");
        break;
    case PIO_SMBUS_ERR_BAD_CHECKSUM:
        fprintf(fout, "PIO_SMBUS_ERR_BAD_CHECKSUM\n");
        break;
    case PIO_SMBUS_ERR_BUF_TOO_SMALL:
        fprintf(fout, "PIO_SMBUS_ERR_BUF_TOO_SMALL\n");
        break;
    case PIO_SMBUS_ERR_BUF_TOO_LARGE:
        fprintf(fout, "PIO_SMBUS_ERR_BUF_TOO_LARGE\n");
        break;
    case PIO_SMBUS_ERR_INVALID_RESP:
        fprintf(fout, "PIO_SMBUS_ERR_INVALID_RESP\n");
        break;
    default:
        fprintf(fout, "Unknown Error (%d, arg: %d)\n", err.fields.error_code, err.fields.arg);
        break;
    };
}

static int dbg_cmd_xfer_and_wait(FILE *fout, absolute_time_t cmd_timeout, enum dbg_result_type expected_result) {
    // Make sure that there isn't already a command in progress

    uint32_t irq = spin_lock_blocking(shared_status.lock);
    hard_assert(!debug_cmd_status.in_progress);
    debug_cmd_status.in_progress = true;
    spin_unlock(shared_status.lock, irq);

    // Send command to core 1
    sio_fifo_req_t req = { .type = FIFO_CMD_DBG_COMMAND };
    multicore_fifo_push_blocking(req.raw);

    // Wait for response
    while (debug_cmd_status.in_progress) {
        if (time_reached(cmd_timeout)) {
            fprintf(fout, "Timed out waiting for response from core 1!\n");

            // Need to set in progress to false under lock to prevent race condition
            // Shouldn't be too bad if we didn't, but best not to add in race conditions
            uint32_t irq = spin_lock_blocking(shared_status.lock);
            debug_cmd_status.in_progress = false;
            spin_unlock(shared_status.lock, irq);

            return 1;
        }
        sleep_ms(1);
    }

    if (debug_cmd_status.result_type == DBG_RESULT_ERR) {
        bq40_decode_err(debug_cmd_status.result.err_result, fout);
        return 2;
    }
    else if (debug_cmd_status.result_type != expected_result) {
        fprintf(fout, "Invalid Result Type from Core 1: %d\n", debug_cmd_status.result_type);
        return 1;
    }

    return 0;
}

static int dbg_cmd_get_int_type(FILE *fout, const char *type, enum read_width *width_out, bool *is_signed_out) {
    if (!strcmp(type, "u8")) {
        *width_out = READ_WIDTH_8;
        *is_signed_out = false;
    }
    else if (!strcmp(type, "s8")) {
        *width_out = READ_WIDTH_8;
        *is_signed_out = true;
    }
    else if (!strcmp(type, "u16")) {
        *width_out = READ_WIDTH_16;
        *is_signed_out = false;
    }
    else if (!strcmp(type, "s16")) {
        *width_out = READ_WIDTH_16;
        *is_signed_out = true;
    }
    else if (!strcmp(type, "u32")) {
        *width_out = READ_WIDTH_32;
        *is_signed_out = false;
    }
    else if (!strcmp(type, "s32")) {
        *width_out = READ_WIDTH_32;
        *is_signed_out = true;
    }
    else {
        fprintf(fout, "Invalid Integer Type: '%s'\n", type);
        return 1;
    }

    return 0;
}

static int dbg_cmd_get_uint(FILE *fout, const char *int_str, long max_val) {
    // Decode cmd parameter
    char *end;
    long val;
    if (int_str[0] == '0' && int_str[1] == 'x') {
        val = strtol(int_str + 2, &end, 16);
    }
    else {
        val = strtol(int_str, &end, 10);
    }
    if (*end != 0 || end == int_str) {
        fprintf(fout, "Invalid Number: '%s'\n", int_str);
        return -1;
    }
    if (val < 0x00 || val > max_val) {
        fprintf(fout, "Integer '%ld' out of range\n", val);
        return -1;
    }
    return val;
}

static int dbg_cmd_show_int_result(FILE *fout, uint32_t val, enum read_width width, bool is_signed) {
    // Process result
    if (is_signed) {
        long result;
        int field_width;

        switch (width) {
        case READ_WIDTH_8:
            result = (int8_t) (val);
            field_width = 2;
            break;
        case READ_WIDTH_16:
            result = (int16_t) (val);
            field_width = 4;
            break;
        case READ_WIDTH_32:
            result = (int32_t) (val);
            field_width = 8;
            break;
        default:
            return 1;
        };

        fprintf(fout, "0x%0*lX (%ld)\n", field_width, result, result);
    }
    else {
        unsigned long result;
        int field_width;

        switch (width) {
        case READ_WIDTH_8:
            result = (uint8_t) (val);
            field_width = 2;
            break;
        case READ_WIDTH_16:
            result = (uint16_t) (val);
            field_width = 4;
            break;
        case READ_WIDTH_32:
            result = (uint32_t) (val);
            field_width = 8;
            break;
        default:
            return 1;
        };

        fprintf(fout, "0x%0*lX (%lu)\n", field_width, result, result);
    }
    return 0;
}

// ============================================
// Canmore CLI Debug Commands
// ============================================

static int bq40_sbs_readint_cb(size_t argc, const char *const *argv, FILE *fout) {
    uint8_t cmd;
    enum read_width width;
    bool is_signed;
    int ret;

    if (argc != 3) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    ret = dbg_cmd_get_uint(fout, argv[1], UINT8_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Decode int type
    ret = dbg_cmd_get_int_type(fout, argv[2], &width, &is_signed);
    if (ret)
        return ret;

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_SBS_READINT;
    debug_cmd_status.cmd = cmd;
    debug_cmd_status.req_data.int_read_width = width;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_INT);
    if (ret)
        return ret;

    fprintf(fout, "SBS Reg[0x%02X]: ", cmd);
    ret = dbg_cmd_show_int_result(fout, debug_cmd_status.result.int_result, width, is_signed);

    return 0;
}

static int bq40_sbs_readblock_cb(size_t argc, const char *const *argv, FILE *fout) {
    int ret;
    uint8_t cmd;

    if (argc != 2) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    // Decode cmd parameter
    ret = dbg_cmd_get_uint(fout, argv[1], UINT8_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_SBS_READBLOCK;
    debug_cmd_status.cmd = cmd;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_BLOCK);
    if (ret)
        return ret;

    // Process result
    size_t read_len = debug_cmd_status.result.block_read_count;
    fprintf(fout, "SBS Reg[0x%02X] (Len: %d): ", cmd, read_len);
    for (size_t i = 0; i < read_len; i++) {
        fprintf(fout, "%02X", debug_cmd_status.block_data[i]);
    }
    fprintf(fout, "\n");

    return 0;
}

static int bq40_mfg_readblock_cb(size_t argc, const char *const *argv, FILE *fout) {
    int ret;
    uint16_t cmd;

    if (argc != 2) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    // Decode cmd parameter
    ret = dbg_cmd_get_uint(fout, argv[1], UINT16_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_MFG_READBLOCK;
    debug_cmd_status.cmd = cmd;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_BLOCK);
    if (ret)
        return ret;

    // Process result
    size_t read_len = debug_cmd_status.result.block_read_count;
    fprintf(fout, "MFG Reg[0x%04X] (Len: %d): ", cmd, read_len);
    for (size_t i = 0; i < read_len; i++) {
        fprintf(fout, "%02X", debug_cmd_status.block_data[i]);
    }
    fprintf(fout, "\n");

    return 0;
}

static int bq40_mfg_runcmd_cb(size_t argc, const char *const *argv, FILE *fout) {
    uint16_t cmd;
    int ret;

    if (argc != 2) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    // Decode cmd parameter
    ret = dbg_cmd_get_uint(fout, argv[1], UINT16_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_MFG_CMD;
    debug_cmd_status.cmd = cmd;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_CMD_OK);
    if (ret)
        return ret;

    fprintf(fout, "OK\n");

    return 0;
}

static int bq40_df_readint_cb(size_t argc, const char *const *argv, FILE *fout) {
    uint16_t cmd;
    enum read_width width;
    bool is_signed;
    int ret;

    if (argc != 3) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    ret = dbg_cmd_get_uint(fout, argv[1], UINT16_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Decode int type
    ret = dbg_cmd_get_int_type(fout, argv[2], &width, &is_signed);
    if (ret)
        return ret;

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_MFG_READBLOCK;
    debug_cmd_status.cmd = cmd;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_BLOCK);
    if (ret)
        return ret;

    uint32_t data;
    switch (width) {
    case READ_WIDTH_8:
        if (debug_cmd_status.result.block_read_count < 1) {
            fprintf(fout, "Block read didn't return enough bytes: %d\n", debug_cmd_status.result.block_read_count);
            return 1;
        }
        data = debug_cmd_status.block_data[0];
        break;
    case READ_WIDTH_16:
        if (debug_cmd_status.result.block_read_count < 2) {
            fprintf(fout, "Block read didn't return enough bytes: %d\n", debug_cmd_status.result.block_read_count);
            return 1;
        }
        data = debug_cmd_status.block_data[0] | (debug_cmd_status.block_data[1] << 8);
        break;
    case READ_WIDTH_32:
        if (debug_cmd_status.result.block_read_count < 4) {
            fprintf(fout, "Block read didn't return enough bytes: %d\n", debug_cmd_status.result.block_read_count);
            return 1;
        }
        data = debug_cmd_status.block_data[0] | (debug_cmd_status.block_data[1] << 8) |
               (debug_cmd_status.block_data[2] << 16) | (debug_cmd_status.block_data[3] << 24);
        break;
    default:
        return 1;
    }

    fprintf(fout, "Data Flash [0x%02X]: ", cmd);
    ret = dbg_cmd_show_int_result(fout, data, width, is_signed);

    return 0;
}

static int bq40_df_write_cb(size_t argc, const char *const *argv, FILE *fout) {
    uint16_t cmd;
    int ret;

    if (argc < 3) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    if (strcmp(argv[1], "--iknowwhatimdoingiswear")) {
        fprintf(fout, "Missing Guard Flag! Refusing to execute!\n");
        return 1;
    }

    if (argc < 4) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    // Decode cmd parameter
    ret = dbg_cmd_get_uint(fout, argv[2], UINT16_MAX);
    if (ret < 0)
        return ret;
    cmd = ret;

    // Decode data parameter
    size_t hex_len = strlen(argv[3]);
    if (hex_len % 2 != 0 || hex_len == 0) {
        fprintf(fout, "Invalid hex string\n");
        return 1;
    }
    size_t byte_len = hex_len / 2;
    if (byte_len > 32) {
        fprintf(fout, "Can only write up to 32 bytes at a time\n");
        return 1;
    }

    static_assert(sizeof(debug_cmd_status.block_data) >= 32, "Expected 32 byte block data buffer");
    for (size_t i = 0; i < hex_len; i++) {
        unsigned char nibble = 0;
        // Invalid device nibble
        char hexchar = argv[3][i];
        if (hexchar >= '0' && hexchar <= '9') {
            nibble = hexchar - '0';
        }
        else if (hexchar >= 'A' && hexchar <= 'F') {
            nibble = hexchar - 'A' + 0xA;
        }
        else if (hexchar >= 'a' && hexchar <= 'f') {
            nibble = hexchar - 'a' + 0xA;
        }
        else {
            fprintf(fout, "Invalid hex string\n");
            return 1;
        }

        if (i % 2 == 0) {
            debug_cmd_status.block_data[i / 2] = nibble << 4;
        }
        else {
            debug_cmd_status.block_data[i / 2] |= nibble;
        }
    }

    // Set command type
    debug_cmd_status.cmd_type = DBG_CMD_DF_WRITE;
    debug_cmd_status.cmd = cmd;
    debug_cmd_status.req_data.block_write_count = byte_len;

    ret = dbg_cmd_xfer_and_wait(fout, make_timeout_time_ms(200), DBG_RESULT_CMD_OK);
    if (ret)
        return ret;

    fprintf(fout, "OK\n");

    return 0;
}

static int bq40_mfginfo_cb(size_t argc, const char *const *argv, FILE *fout) {
    (void) argc;
    (void) argv;

    if (!shared_status_is_connected) {
        fprintf(fout, "Not Connected to BQ40!\n");
        return 1;
    }

    fprintf(fout, "Serial: %dd\n", shared_status.mfg_info.serial);
    fprintf(fout, "Manufacture Date: %d/%d/%d\n", shared_status.mfg_info.mfg_mo, shared_status.mfg_info.mfg_day,
            shared_status.mfg_info.mfg_year);
    fprintf(fout, "Device Type: 0x%04X\n", shared_status.mfg_info.device_type);
    fprintf(fout, "Firmware Version: ");
    for (size_t i = 0; i < sizeof(shared_status.mfg_info.firmware_version); i++) {
        fprintf(fout, "%02X", shared_status.mfg_info.firmware_version[i]);
    }
    fprintf(fout, "\n");
    fprintf(fout, "Scale Factor: %d\n", shared_status.mfg_info.scale_factor);

    return 0;
}

static int bq40_state_cb(size_t argc, const char *const *argv, FILE *fout) {
    (void) argc;
    (void) argv;

    const char *batt_state_str = "Unknown?";
    batt_state_t batt_state = shared_status.state;
    if (batt_state < batt_state_name_count) {
        batt_state_str = batt_state_names[batt_state];
    }
    fprintf(fout, "Batt State: %s\n", batt_state_str);

    if (!shared_status_is_connected) {
        return 0;
    }

    fprintf(fout, "SOC: %d%%\n", shared_status.batt_info.relative_soc);
    fprintf(fout, "SOC Max Error: %d%%\n", shared_status.batt_info.max_error);
    fprintf(fout, "Temperature: %.1f C\n", (shared_status.batt_info.temperature / 10.0) - 273.2);

    uint16_t time_left = shared_status.batt_info.time_to_empty;
    if (time_left != UINT16_MAX) {
        fprintf(fout, "Time to Empty: %dh %dm\n", time_left / 60, time_left % 60);
    }
    else {
        fprintf(fout, "Time to Empty: <A Long Time>\n");
    }
    time_left = shared_status.batt_info.time_to_full;
    if (time_left != UINT16_MAX) {
        fprintf(fout, "Time to Full: %dh %dm\n", time_left / 60, time_left % 60);
    }
    else {
        fprintf(fout, "Time to Full: <A Long Time>\n");
    }

    uint16_t voltage = shared_status.batt_info.da_status1.batt_voltage;
    fprintf(fout, "Batt Voltage: %d.%dV\n", voltage / 1000, voltage % 1000);
    voltage = shared_status.batt_info.da_status1.pack_voltage;
    fprintf(fout, "Pack Voltage: %d.%dV\n", voltage / 1000, voltage % 1000);

    fprintf(fout, "Current Now: %.3fA\n", shared_status.batt_info.current / 1000.0);
    fprintf(fout, "Avg Current: %.3fA\n", shared_status.batt_info.avg_current / 1000.0);

    voltage = shared_status.batt_info.charging_voltage;
    fprintf(fout, "Requested Charging Voltage: %d.%dV\n", voltage / 1000, voltage % 1000);
    fprintf(fout, "Requested Charging Current: %.3fA\n", shared_status.batt_info.charging_current / 1000.0);

    uint32_t safety_status = shared_status.batt_info.safety_status;
    if (safety_status != 0) {
        fprintf(fout, "Safety Status: 0x%08lX\n", safety_status);
        canmore_print_bit_array(fout, safety_status, safety_status_names, safety_status_name_count, false);
    }
    else {
        fprintf(fout, "Safety Status: No Faults\n");
    }

    uint32_t pf_status = shared_status.batt_info.pf_status;
    if (pf_status != 0) {
        fprintf(fout, "PF Status: 0x%08lX\n", pf_status);
        canmore_print_bit_array(fout, pf_status, pf_status_names, pf_status_name_count, false);
    }
    else {
        fprintf(fout, "PF Status: No Faults\n");
    }

    return 0;
}

static int bq40_cell_state_cb(size_t argc, const char *const *argv, FILE *fout) {
    (void) argc;
    (void) argv;

    if (!shared_status_is_connected) {
        fprintf(fout, "Not Connected to BQ40!\n");
        return 1;
    }

    fprintf(fout, "Cell Voltages:\n");
    uint16_t voltage = shared_status.batt_info.da_status1.cell1_voltage;
    fprintf(fout, "  1: %d.%dV\n", voltage / 1000, voltage % 1000);
    voltage = shared_status.batt_info.da_status1.cell2_voltage;
    fprintf(fout, "  2: %d.%dV\n", voltage / 1000, voltage % 1000);
    voltage = shared_status.batt_info.da_status1.cell3_voltage;
    fprintf(fout, "  3: %d.%dV\n", voltage / 1000, voltage % 1000);
    voltage = shared_status.batt_info.da_status1.cell4_voltage;
    fprintf(fout, "  4: %d.%dV\n", voltage / 1000, voltage % 1000);
    voltage = shared_status.batt_info.cell5_voltage;
    fprintf(fout, "  5: %d.%dV\n", voltage / 1000, voltage % 1000);

    fprintf(fout, "Cell Currents:\n");
    fprintf(fout, "  1: %.3fA\n", (shared_status.batt_info.da_status1.cell1_current / 1000.0));
    fprintf(fout, "  2: %.3fA\n", (shared_status.batt_info.da_status1.cell2_current / 1000.0));
    fprintf(fout, "  3: %.3fA\n", (shared_status.batt_info.da_status1.cell3_current / 1000.0));
    fprintf(fout, "  4: %.3fA\n", (shared_status.batt_info.da_status1.cell4_current / 1000.0));
    fprintf(fout, "  5: Not Monitored\n");

    return 0;
}

static int bq40_status_flags_cb(size_t argc, const char *const *argv, FILE *fout) {
    if (argc != 2) {
        fprintf(fout, "Invalid Syntax!\n");
        return 1;
    }

    if (!shared_status_is_connected) {
        fprintf(fout, "Not Connected to BQ40!\n");
        return 1;
    }

    if (!strcmp(argv[1], "manufacturing")) {
        uint16_t mfg_status = shared_status.mfg_info.manufacturing_status;
        fprintf(fout, "Manufacturing Status: 0x%04X\n", mfg_status);
        canmore_print_bit_array(fout, mfg_status, manufacturing_status_names, manufacturing_status_name_count, true);
    }
    else if (!strcmp(argv[1], "operation")) {
        uint32_t operation_status = shared_status.batt_info.operation_status;
        fprintf(fout, "Operation Status: 0x%08lX\n", operation_status);
        canmore_print_bit_array(fout, operation_status, operation_status_names, operation_status_name_count, true);
    }
    else if (!strcmp(argv[1], "battery")) {
        uint16_t battery_status = shared_status.batt_info.battery_status;
        fprintf(fout, "Operation Status: 0x%04X\n", battery_status);
        canmore_print_bit_array(fout, battery_status, battery_status_names, battery_status_name_count, true);
    }
    else if (!strcmp(argv[1], "gauging")) {
        uint32_t gauging_status;
        if (!core1_get_gauging_status(&gauging_status)) {
            fprintf(fout, "Failed to read gauging status register!\n");
            return 1;
        }
        fprintf(fout, "Gauging Status: 0x%08lX\n", gauging_status);
        canmore_print_bit_array(fout, gauging_status, gauging_status_names, gauging_status_name_count, true);
    }
    else if (!strcmp(argv[1], "charging")) {
        uint32_t charging_status;
        if (!core1_get_charging_status(&charging_status)) {
            fprintf(fout, "Failed to read charging status register!\n");
            return 1;
        }
        fprintf(fout, "Charging Status: 0x%08lX\n", charging_status);
        canmore_print_bit_array(fout, charging_status, charging_status_names, charging_status_name_count, true);
    }
    else {
        fprintf(fout, "Unknown Status Register: '%s'\n", argv[1]);
        return 1;
    }

    return 0;
}

static void core1_register_canmore_cmds(void) {
    debug_remote_cmd_register("bq40_sbsreadint", "[cmd] [type]",
                              "[DEBUG CMD] Sends the requested sbs command to the bq40 to read that int\n"
                              "  See the bq40 reference manual sections 18.2 through 18.84 for list of commands\n"
                              "  [type] can be of: u8, s8, u16, s16, u32, or s32",
                              bq40_sbs_readint_cb);
    debug_remote_cmd_register("bq40_sbsreadblock", "[cmd]",
                              "[DEBUG CMD] Sends the requested sbs command to the bq40 to read that block\n"
                              "  See the bq40 reference manual sections 18.2 through 18.84 for list of commands",
                              bq40_sbs_readblock_cb);
    debug_remote_cmd_register("bq40_mfgreadblock", "[cmd]",
                              "[DEBUG CMD] Sends the requested manufacturing command to the bq40 to read that block\n"
                              "  See the bq40 reference manual section 18.1 for list of commands",
                              bq40_mfg_readblock_cb);
    debug_remote_cmd_register("bq40_dfreadint", "[addr] [type]",
                              "[DEBUG CMD] Reads an integer from bq40 dataflash\n"
                              "  See the bq40 reference manual sections 19.17 for list of dataflash addresses\n"
                              "  [type] can be of: u8, s8, u16, s16, u32, or s32",
                              bq40_df_readint_cb);
    debug_remote_cmd_register(
        "bq40_mfgruncmd", "[cmd]",
        "[DEBUG CMD] Sends the requested manufacturing command to the bq40 to run that command (no readback)\n"
        "  See the bq40 reference manual section 18.1 for list of commands",
        bq40_mfg_runcmd_cb);

    debug_remote_cmd_register("bq40_mfginfo", "", "Dumps the manufacturing info pulled from the device at startup",
                              bq40_mfginfo_cb);
    debug_remote_cmd_register("bq40_state", "", "Dumps the general state of the bq40", bq40_state_cb);
    debug_remote_cmd_register("bq40_cell_state", "", "Dumps the state of individual cells on the bq40",
                              bq40_cell_state_cb);
    debug_remote_cmd_register("bq40_status_flags", "[register]",
                              "Dumps the requested status flags register. Valid options for register are:\n"
                              " - operation: Reports the general operation status of the bq40\n"
                              " - battery: Reports the standardized Smart Battery System status\n"
                              " - manufacturing: Reports the enable status of the various subsystems on the B40\n"
                              " - gauging: Reports the state of gas gauging system\n"
                              " - charging: Reports the state of advanced charge algorithm system",
                              bq40_status_flags_cb);
    debug_remote_cmd_register(
        "bq40_dfwrite", "[--iknowwhatimdoingiswear] [flash addr] [hex data...]",
        "[DEBUG CMD] Writes the requested data to the bq40 dataflash at the requested address\n"
        "!!! WARNING !!! This command can modify behavior of the battery charge chip!\n"
        "THIS MAY PUT IT IN AN UNSAFE STATE! DO NOT USE THIS COMMAND UNLESS YOU KNOW WHAT YOU\n"
        "ARE DOING! This command can be used in place of disassembling the battery to connect\n"
        "it to bq studio. DO NOT USE IF YOU HAVEN'T THOROUGHLY READ THE BQ40 REFERENCE MANUAL!\n"
        "  [flash addr]: The starting flash address to write (in decimal, or prefixed with 0x hex)\n"
        "  [hex data...]: String of hex characters to write to dataflash (up to 32 bytes, 64 chars, no spaces or 0x)",
        bq40_df_write_cb);
}
