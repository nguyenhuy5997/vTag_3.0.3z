       
       
       
 .set adc_channel, 0
 .set adc_oversampling_factor_log, 2
 .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)
 .bss
 .global charging
charging:
 .long 0
 .global low_thr
low_thr:
 .long 0
 .global high_thr
high_thr:
 .long 0
 .global sample_counter
sample_counter:
 .long 0
 .global last_result
last_result:
 .long 0
 .text
 .global entry
entry:
 .global loop
 loop:
 stage_rst
   REG_WR ((((0x3ff48400 + 0x0)) - 0x3ff48000) / 4), ((14 +13) + (1) - 1), (14 +13), ((1) & 0xff)
   wait_loop_1:
  WAIT 65535
  stage_inc 1
  jumps wait_loop_1, 10, lt
   REG_WR ((((0x3ff48400 + 0x0)) - 0x3ff48000) / 4), ((14 +13) + (1) - 1), (14 +13), ((0) & 0xff)
  wait_loop_2:
  WAIT 65535
  stage_inc 1
  jumps wait_loop_2, 20, lt
   REG_WR ((((0x3ff48400 + 0x0)) - 0x3ff48000) / 4), ((14 +13) + (1) - 1), (14 +13), ((1) & 0xff)
   wait_loop_3:
  WAIT 65535
  stage_inc 1
  jumps wait_loop_3, 30, lt
   REG_WR ((((0x3ff48400 + 0x0)) - 0x3ff48000) / 4), ((14 +13) + (1) - 1), (14 +13), ((0) & 0xff)
  wait_loop_4:
  WAIT 65535
  stage_inc 1
  jumps wait_loop_4, 40, lt
  wait_loop_5:
  WAIT 65535
  stage_inc 1
  jumps wait_loop_5, 160, lt
    REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 +5) + (16) - 1), (14 +5)
   move r3, r0
 and r0, r3, 1
    JUMP loop, EQ
 JUMP exit, EQ
 .global exit
exit:
 REG_WR ((((0x3ff48000 + 0x18)) - 0x3ff48000) / 4), ((24) + (((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1))))) - 1), (24), ((((0) & 0x1)) & 0xff)
    halt
 .global wake_up
wake_up:
 REG_RD ((((0x3ff48000 + 0xc0)) - 0x3ff48000) / 4), ((19) + ((((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1)))) + ((((((0x1) >> (0 + 8)) & 1) + (((0x1) >> (0 + 8 + 1)) & 1)) + ((((0x1) >> (0 + 8 + 2)) & 1) + (((0x1) >> (0 + 8 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 8 + 4)) & 1) + (((0x1) >> (0 + 8 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 8 + 4 + 2)) & 1) + (((0x1) >> (0 + 8 + 4 + 2 + 1)) & 1)))))) - 1), (19)
 and r0, r0, 1
 jump exit, eq
 wake
 REG_WR ((((0x3ff48000 + 0x18)) - 0x3ff48000) / 4), ((24) + (((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1))))) - 1), (24), ((((0) & 0x1)) & 0xff)
 halt
