#include "ledc_registers.h"

// need a function to read registers and set them before doing anything with them if they have reserved bits

cr1_t encode_cr1(cr1_fields_t *fields) {
    cr1_t reg = 0;
    reg |= (fields->duty1 << CR1_BUCK1_DUTY_SHIFT) & CR1_BUCK1_DUTY_MASK;
    reg |= (fields->duty2 << CR1_BUCK2_DUTY_SHIFT) & CR1_BUCK2_DUTY_MASK;
    reg |= (fields->hled_cur1 << CR1_HLED_CUR1_SHIFT) & CR1_HLED_CUR1_MASK;
    reg |= (fields->hled_cur2 << CR1_HLED_CUR2_SHIFT) & CR1_HLED_CUR2_MASK;
    reg |= (fields->unlock << CR1_UNLOCK_SHIFT) & CR1_UNLOCK_MASK;
    return (reg & MASK_24);
}

cr1_fields_t decode_cr1(uint32_t register_contents) {
    cr1_fields_t fields = { 0 };
    fields.duty1 = (register_contents >> CR1_BUCK1_DUTY_SHIFT) & CR1_BUCK1_DUTY_MASK;
    fields.duty2 = (register_contents >> CR1_BUCK2_DUTY_SHIFT) & CR1_BUCK2_DUTY_MASK;
    fields.hled_cur1 = (register_contents >> CR1_HLED_CUR1_SHIFT) & CR1_HLED_CUR1_MASK;
    fields.hled_cur2 = (register_contents >> CR1_HLED_CUR2_SHIFT) & CR1_HLED_CUR2_MASK;
    fields.unlock = (register_contents >> CR1_UNLOCK_SHIFT) & CR1_UNLOCK_MASK;
    return fields;
}

cr2_t encode_cr2(cr2_fields_t *fields) {
    cr2_t reg = 0;
    reg |= (fields->il1_peak << CR2_IL1_PEAK_SHIFT) & CR2_IL1_PEAK_MASK;
    reg |= (fields->il2_peak << CR2_IL2_PEAK_SHIFT) & CR2_IL2_PEAK_MASK;
    reg |= (fields->vled1_toff << CR2_VLED_TOFF1_SHIFT) & CR2_VLED_TOFF1_MASK;
    reg |= (fields->vled2_toff << CR2_VLED_TOFF2_SHIFT) & CR2_VLED_TOFF2_MASK;
    reg |= (fields->gostby << CR2_GOSTBY_SHIFT) & CR2_GOSTBY_MASK;
    reg |= (fields->enable << CR2_EN_SHIFT) & CR2_EN_MASK;
    reg |= (fields->reserved << CR2_RESERVED_SHIFT) & CR2_RESERVED_MASK;
    return (reg & MASK_24);
}
cr2_fields_t decode_cr2(uint32_t register_contents) {
    cr2_fields_t fields = { 0 };
    fields.enable = (register_contents >> CR2_EN_SHIFT) & CR2_EN_MASK;
    fields.gostby = (register_contents >> CR2_GOSTBY_SHIFT) & CR2_GOSTBY_MASK;
    fields.il1_peak = (register_contents >> CR2_IL1_PEAK_SHIFT) & CR2_IL1_PEAK_MASK;
    fields.il2_peak = (register_contents >> CR2_IL2_PEAK_SHIFT) & CR2_IL2_PEAK_MASK;
    fields.vled1_toff = (register_contents >> CR2_VLED_TOFF1_SHIFT) & CR2_VLED_TOFF1_MASK;
    fields.vled2_toff = (register_contents >> CR2_VLED_TOFF2_SHIFT) & CR2_VLED_TOFF2_MASK;
    fields.reserved = (register_contents >> CR2_RESERVED_SHIFT) & CR2_RESERVED_MASK;
    return fields;
}

cr3_t encode_cr3(cr3_fields_t *fields) {
    cr3_t reg = 0;
    reg |= (fields->b_in_w1 << CR3_B_IN_W1_SHIFT) & CR3_B_IN_W1_MASK;
    reg |= (fields->b_in_w2 << CR3_B_IN_W2_SHIFT) & CR3_B_IN_W2_MASK;
    reg |= (fields->din_map1 << CR3_DIN_MAP1_SHIFT) & CR3_DIN_MAP1_MASK;
    reg |= (fields->din_map2 << CR3_DIN_MAP2_SHIFT) & CR3_DIN_MAP2_MASK;
    reg |= (fields->ph1 << CR3_PH1_SHIFT) & CR3_PH1_MASK;
    reg |= (fields->ph2 << CR3_PH2_SHIFT) & CR3_PH2_MASK;
    reg |= (fields->pwm_sync << CR3_PWM_SYNC_SHIFT) & CR3_PWM_SYNC_MASK;
    reg |= (fields->reserved1 << CR3_RESERVED1_SHIFT) & CR3_RESERVED1_MASK;
    reg |= (fields->reserved2 << CR3_RESERVED2_SHIFT) & CR3_RESERVED2_MASK;
    return (reg & MASK_24);
}

cr3_fields_t decode_cr3(uint32_t register_contents) {
    cr3_fields_t fields = { 0 };
    fields.ph1 = (register_contents >> CR3_PH1_SHIFT) & CR3_PH1_MASK;
    fields.ph2 = (register_contents >> CR3_PH2_SHIFT) & CR3_PH2_MASK;
    fields.din_map1 = (register_contents >> CR3_DIN_MAP1_SHIFT) & CR3_DIN_MAP1_MASK;
    fields.din_map2 = (register_contents >> CR3_DIN_MAP2_SHIFT) & CR3_DIN_MAP2_MASK;
    fields.b_in_w1 = (register_contents >> CR3_B_IN_W1_SHIFT) & CR3_B_IN_W1_MASK;
    fields.b_in_w2 = (register_contents >> CR3_B_IN_W2_SHIFT) & CR3_B_IN_W2_MASK;
    fields.pwm_sync = (register_contents >> CR3_PWM_SYNC_SHIFT) & CR3_PWM_SYNC_MASK;
    fields.reserved1 = (register_contents >> CR3_RESERVED1_SHIFT) & CR3_RESERVED1_MASK;
    fields.reserved2 = (register_contents >> CR3_RESERVED2_SHIFT) & CR3_RESERVED2_MASK;
    return fields;
}

uint32_t decode_register(spi_frame_t frame) {
    uint32_t reg = 0;
    return ((reg | frame) & MASK_24);
}

// cr4_t encode_cr4() {}

sr1_t decode_sr1(spi_frame_t frame) {
    sr1_t reg = 0;
    return ((reg | frame) & MASK_24);
}

sr2_t decode_sr2(spi_frame_t frame) {
    sr2_t reg = 0;
    return ((reg | frame) & MASK_24);
}

sr3_t decode_sr3(spi_frame_t frame) {
    sr3_t reg = 0;
    return ((reg | frame) & MASK_24);
}
