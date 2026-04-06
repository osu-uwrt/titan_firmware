// Patch to enable reversable dshot mode (3D dshot) on the APD 80F3 ESCs.
// This will permanently enable 3D mode, causing throttle values 48-1047 to map to forward, and 1049-2048 to maps to reverse
// Designed for Firmware Build 2.3.0 (Build timestamp 'Jan  4 2023 15:20:50')

.extern patch_entry
.global patch_entry
.type patch_entry,%function
.thumb_func
patch_entry:
    // Save all registers (Caller saved and modified registers)
    push       {r0-r7}
    push       {lr}

    mov        r5, r1               // Save r1 to callee saved to avoid unnecessary pushing

    cmp        r4,#0
    beq        store_and_exit

    // 0. Preload motor direction values
    ldr        r0, f_dir_check_0    // Load direction check 0 function ptr
    blx        r0                   // Load direction check 0
    mov        r6, r0               // Save result
    ldr        r0, f_dir_check_1    // Load direction check 1 function ptr
    blx        r0                   // Load direction check 1
    mov        r7, r0               // Save result

    // 1. check if throttle is forward or reverse (or 0 should also be okay)
    sub        r4,#0x2f             // Convert from DShot value to throttle value
                                    // This results in the first positive throttle (48) to equal 1
    ldr        r0, =#1001           // Subtracting 1001 so that 1048 maps to zero, any positive value is negative, and any negative value maps positive
    sub        r0, r4, r0
    beq        exit_patch           // If value was 1048, don't process it (following DShot protocol)
    bpl        do_dshot_reverse     // If throttle was greater than 1048, it was reverse throttle
    // Else, it was negative, then process r4 as expected for forward throttle

    // === Forward throttle ===
    // 2. Check if throttle is below minimum in motion value
    // Motor moving check
    ldr        r0, [r5,#0x78]       // Check motor_moving_bool
    cmp        r0, #0x0             // If motor is moving don't allow value below some minimum
    beq        write_fw_throttle

    // Included in this check, make sure we're actually going the right direction (not actually in their code, but I think we should probably do this)
    cmp        r6, r7
    bne        write_fw_throttle    // If values aren't equal, then we're going the wrong direction for positive rotation, don't try to slow down gracefully

    // Check if throttle is low enough to qualify for override
    ldr        r0, [r5,#0x10]       // Load in minimum while in motion preset value to override
    cmp        r0, r4
    bls        write_fw_throttle    // Only override if the requested throttle is smaller than minimum in motion preset

    // Override
    str        r0, [r5,#0xc]        // Store the minimum preset rather than the requested value
    b          exit_patch           // Direction already checked, we can just quit now

write_fw_throttle:
    // 3. Store throttle value
    // Throttle has been verified to be in the correct range
    // Throttle should be > 0, < 1000, so it'll map properly to 0-1000 (although we technically lose the 1000 throttle value)
    str        r4, [r5,#0xc]

    // 4. If direction does not match expected, jump to swap code
    //    If it does, return
    cmp        r6, r7               // Check direction comparison result
    bne        swap_direction       // If the two direction checks weren't equal, we're going the wrong way, try to swap direction
    b          exit_patch           // If we're going the right way, then we're done



    // === Reverse Throttle: ===
do_dshot_reverse:
    mov r4, r0                      // Move the subtracted throttle to the new absolute throttle value

    // 2. Check if throttle is below minimum in motion value
    // Motor moving check
    ldr        r0, [r5,#0x78]       // Check motor_moving_bool
    cmp        r0, #0x0             // If motor is moving don't allow value below some minimum
    beq        write_rev_throttle

    // Included in this check, make sure we're actually going the right direction (not actually in their code, but I think we should probably do this)
    cmp        r6, r7
    beq        write_rev_throttle   // If values aren't equal, then we're going the wrong direction for positive rotation, don't try to slow down gracefully

    // Check if throttle is low enough to qualify for override
    ldr        r0, [r5,#0x10]       // Load in minimum while in motion preset value to override
    cmp        r0, r4
    bls        write_rev_throttle   // Only override if the requested throttle is smaller than minimum in motion preset

    // Override
    str        r0, [r5,#0xc]        // Store the minimum preset rather than the requested value
    b          exit_patch           // Direction already checked, we can just quit now

write_rev_throttle:
    // 3. Store throttle value
    // Throttle has been verified to be in the correct range
    // Throttle should be > 0, < 1000, so it'll map properly to 0-1000 (although we technically lose the 1000 throttle value)
    str        r4, [r5,#0xc]

    // 4. If direction does not match expected, jump to swap code
    //    If it does, return
    cmp        r6, r7               // Check direction comparison result
    beq        swap_direction       // If the two direction checks weren't equal, we're going the wrong way, try to swap direction
    b          exit_patch           // If we're going the right way, then we're done




    // === Swap Direction Code ===
swap_direction:
    ldr        r0, [r5,#0x20]       // Load current motor period (inverse of RPM)
    ldr        r1, =#8000
    cmp        r0, r1
    bhi        swap_okay            // If motor speed is slow enough (period higher than threshold), its good to swap
    ldr        r0, [r5,#0x10]       // If motor moving too fast, set throttle speed to minimum throttle + 5
    add        r0, #0x5
    str        r0, [r5,#0xc]        // Write throttle speed
    b          exit_patch

swap_okay:
    // Save the value of tim1_cc_irq_cb and enable_bidir_telemtry
    // This is because swap direction clears this to empty, but it never gets reinitialized
    // Since PWM doesn't need this (for whatever reason?), but dshot/proshot does, and they never put in logic to handle this
    // So we'll just restore it to its current value after calling swap_direction
    ldr        r4, enable_bidir_telemtry
    ldr        r5, [r4]
    ldr        r6, ptr_tim1_cc_irq_cb
    ldr        r7, [r6]

    ldr        r0, f_swap_direction
    blx        r0                   // Perform direction swap

    // Restore the previous value of tim1_cc_irq_cb and enable_bidir_telemtry
    str        r5, [r4]
    str        r7, [r6]

    b          exit_patch

store_and_exit:
    // Directly store value in r4 then quit
    // Required for 0 value where it should just be written and exited
    // TODO: Check if zero should have minimum throttle smoothing applied to it
    str        r4, [r5,#0xc]

    // Return code
exit_patch:
    pop {r0}
    mov lr, r0
    pop {r0-r7}

return_patch:
    // Patch this with the proper return address
    // As of last firmware it's 'b.w 0x080080ba'
    mov r0,r0
    mov r0,r0
    b .

.align 2
f_dir_check_0:
.word 0x080099c4+1
f_dir_check_1:
.word 0x080099d0+1
f_swap_direction:
.word 0x0800b338+1
ptr_tim1_cc_irq_cb:
.word 0x20000704
enable_bidir_telemtry:
.word 0x200007a8
