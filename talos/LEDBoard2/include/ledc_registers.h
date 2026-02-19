#ifndef LEDC_REGISTERS_H
#define LEDC_REGISTERS_H

#include "ledc_io.h"
#include "shifts_and_masks.h"

#include <stdint.h>

#define CR1_ADDRESS 0x01
#define CR2_ADDRESS 0x02
#define CR3_ADDRESS 0x03
#define CR4_ADDRESS 0x04
#define SR1_ADDRESS 0x05
#define SR2_ADDRESS 0x06
#define SR3_ADDRESS 0x07

/* =========================
 * L99LD20 Control Register 1 (CR#1) — Addr 0x01, R/W
 * Bits:
 * 23..14  DUTY1     — 10-bit PWM duty for Buck1 (0..0x3FF); default 50%  (CR#1<23:14>)
 * 13..4   DUTY2     — 10-bit PWM duty for Buck2 (0..0x3FF); default 50%  (CR#1<13:4>)
 * 3       HLEDCUR1  — High LED current conf Buck1 (OTP-set: 1=High, 0=Low)
 * 2       HLEDCUR2  — High LED current conf Buck2 (OTP-set: 1=High, 0=Low)
 * 1       UNLOCK    — 1 enables setting EN/GOSTBY/BST_DIS on next SPI frame; auto-clears
 * 0       PARITY    — Odd parity bit (over full 32-bit frame)
 * ========================= */
typedef uint32_t cr1_t; /* CR#1 fields above.  */

/* =========================
 * L99LD20 Control Register 2 (CR#2) — Addr 0x02, R/W
 * Bits:
 * 23..18  IL1_PEAK   — Peak inductor current select Buck1
 * 17..12  IL2_PEAK   — Peak inductor current select Buck2
 * 11..8   VLED_TOFF1 — Constant VLED×TOFF select Buck1 (0000=10 V·µs … 1111=72 V·µs)
 * 7..4    VLED_TOFF2 — Constant VLED×TOFF select Buck2 (0000=10 V·µs … 1111=72 V·µs)
 * 3       GOSTBY     — 1: Standby (if EN=0); 0: Woken
 * 2       EN         — 1: Enable Active mode (requires UNLOCK=1 in prior frame)
 * 1       RESERVED   — (don’t care / keep at default unless otherwise stated)
 * 0       PARITY     — Odd parity bit
 * Notes: Enter Active mode with two frames: (1) CR#1.UNLOCK=1, then (2) CR#2.EN=1 & GOSTBY=0.
 * ========================= */
typedef uint32_t cr2_t; /* CR#2 fields above. */

/* =========================
 * L99LD20 Control Register 3 (CR#3) — Addr 0x03, R/W
 * Bits:
 * 23..20  PH1       — Buck1 phase select (phase = PH1 * 360/16)
 * 19..16  PH2       — Buck2 phase select (phase = PH2 * 360/16)
 * 15..14  DIN_MAP1  — Buck1 control source map (see Table 18)
 * 13..12  DIN_MAP2  — Buck2 control source map (see Table 18)
 * 11..7   RESERVED
 * 6       PWM_SYNC  — 1: reset PWM counter (auto-clears), 0: no reset
 * 5..4    B_IN_W1   — Buck1 input-voltage window select
 * 3..2    B_IN_W2   — Buck2 input-voltage window select
 * 1       RESERVED  — (must be 1 in CR#4; for CR#3 keep default per datasheet)
 * 0       PARITY    — Odd parity bit
 * ========================= */
typedef uint32_t cr3_t; /* CR#3 fields above. */

/* =========================
 * L99LD20 Control Register 4 (CR#4) — Addr 0x04, R/W
 * Bits:
 * 23..2   (various / test / WD_TRIG/unused per datasheet; not commonly used in app flow)
 * 3..2    B_IN_W2   — Buck2 input-voltage window select
 * 1       RESERVED  — Must be set to 1
 * 0       PARITY    — Odd parity bit
 * ========================= */
typedef uint32_t cr4_t; /* CR#4 fields above. */

/* =========================
 * L99LD20 Status Register 1 (SR#1) — Addr 0x05, R/C
 * Bits (selection; see datasheet for full list):
 * 23..??  (fault/status flags for Buck1/Buck2 like SHTx/OVTx/OLx, etc.)
 * 4..3    TW11/TW12 — Temp warning flags for Buck1 (thresholds 1/2)
 * ...     (additional status bits as defined)
 * 0       PARITY    — Odd parity bit
 * Read & Clear: reading clears latched bits (R/C).
 * ========================= */
typedef uint32_t sr1_t; /* SR#1: read to obtain/clear faults; parity at bit0. */

/* =========================
 * L99LD20 Status Register 2 (SR#2) — Addr 0x06, R/C
 * Bits (selection; see datasheet for full list):
 * 22..21  TW21/TW22 — Temp warning flags for Buck2
 * 13      VS_UV     — 1 when VS ≤ VS_UV (undervoltage)
 * 11      TOFF_MIN2 — Buck2 min off-time violation
 * 12      TOFF_MIN1 — Buck1 min off-time violation
 * 10      TOFF_MAX1 — Buck1 max off-time reached
 * 9       TOFF_MAX2 — Buck2 max off-time reached
 * 5       DIN_ST    — Filtered DIN pin status
 * 0       PARITY    — Odd parity bit
 * Read & Clear: reading clears latched bits (R/C).
 * ========================= */
typedef uint32_t sr2_t; /* SR#2 fields above. */

/* =========================
 * L99LD20 Status Register 3 (SR#3) — Addr 0x07, R/C
 * Bits:
 * 23..16  VLED1,OFF — ADC of VLED1 sampled during Buck1 off-time (0..~52.5V)
 * 15..8   VLED2,OFF — ADC of VLED2 sampled during Buck2 off-time (0..~52.5V)
 * 7..1    UNUSED
 * 0       PARITY    — Odd parity bit
 * Read & Clear: reading clears latched bits (R/C).
 * ========================= */
typedef uint32_t sr3_t; /* SR#3 fields above. */

typedef struct {
    uint32_t duty1, duty2, hled_cur1, hled_cur2, unlock;
} cr1_fields_t;

typedef struct {
    uint32_t il1_peak, il2_peak, vled1_toff, vled2_toff, gostby, reserved, enable;
} cr2_fields_t;

typedef struct {
    uint32_t ph1, ph2, din_map1, din_map2, reserved1, pwm_sync, b_in_w1, b_in_w2, reserved2;
} cr3_fields_t;

cr1_t encode_cr1(cr1_fields_t *fields);
cr2_t encode_cr2(cr2_fields_t *fields);
cr3_t encode_cr3(cr3_fields_t *fields);
cr1_fields_t decode_cr1(uint32_t register_contents);
cr2_fields_t decode_cr2(uint32_t register_contents);
cr3_fields_t decode_cr3(uint32_t register_contents);
uint32_t decode_register(spi_frame_t frame);

#endif  // LEDC_REGISTERS_H
