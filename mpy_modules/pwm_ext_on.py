# /pwm_ext_on.py -- bPuppy boot switch: enable PWM_EXT servo output on power-up.
#
# HOW IT WORKS
#   The firmware's frozen main.py looks for /pwm_ext_on.py on the board's
#   filesystem at every boot.  File present -> it runs.  File absent -> nothing
#   happens.  So THIS FILE IS THE SWITCH:  keep it = ON,  delete it = OFF.
#   No recompiling, no typing commands.  Takes effect on the next power-up.
#
#   It lives next to /main.py (the program KittenBlock downloads) and the two
#   are completely independent -- KittenBlock rewrites /main.py on every
#   download but never touches this file.
#
# WHY THESE COMMENTS ARE IN ENGLISH
#   Uploading over Bluetooth (ViperIDE) drops non-ASCII bytes, which would
#   corrupt the file.  Keep it ASCII-only.
#
# PIN / CHANNEL MAP (channel number = PWM_EXT number - 1)
#   PWM_EXT1  GPIO 3   ch 0    shares pin with battery ADC   -> needs adc_stop
#   PWM_EXT2  GPIO 47  ch 1    free pin                      -> no adc_stop  <-- enabled below
#   PWM_EXT3  GPIO 48  ch 2    shares pin with WS2812 LED    -> needs adc_stop
#
#   PWM_EXT1 (GPIO3) and PWM_EXT3 (GPIO48) share their pin with the battery
#   measurement (ADC divider / WS2812 LED).  Before using those, call
#   bpuppy_adc.stop() -- otherwise the firmware keeps driving/reading the same
#   pin and fights your PWM signal.  bpuppy_adc.stop() also stops the LED
#   monitor task, so one call covers both.  Side effect: battery voltage
#   reading and the battery LED go away until bpuppy_adc.init() again.
#   PWM_EXT2 (GPIO47) is a free pin -- none of that applies here.
#
# TO TURN IT OFF AGAIN
#   Delete /pwm_ext_on.py from the board (ViperIDE file manager, or
#   `mpremote rm :pwm_ext_on.py`).  Next boot will not start anything.
#
# MANUAL USE INSTEAD (from the REPL, without this file):
#   import bpuppy_pwm_ext
#   bpuppy_pwm_ext.init(1, 47)          # ch 1 = PWM_EXT2
#   bpuppy_pwm_ext.set_angle(1, 90)
#   bpuppy_pwm_ext.deinit(1)

import bpuppy_pwm_ext

bpuppy_pwm_ext.init(1, 47)              # ch 1 = PWM_EXT2, signal on GPIO47
bpuppy_pwm_ext.set_angle(1, 90)         # move to 90 deg right away

# Enable more outputs by uncommenting:
#   import bpuppy_adc
#   bpuppy_adc.stop()                   # required before PWM_EXT1 or PWM_EXT3
#   bpuppy_pwm_ext.init(0, 3)           # PWM_EXT1
#   bpuppy_pwm_ext.set_angle(0, 90)
#   bpuppy_pwm_ext.init(2, 48)          # PWM_EXT3
#   bpuppy_pwm_ext.set_angle(2, 90)

print("[pwm_ext_on] PWM_EXT2 ready on GPIO47")
