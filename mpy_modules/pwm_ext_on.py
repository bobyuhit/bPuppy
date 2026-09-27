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
# WHAT TO EDIT
#   Only the macros below.  1 = enable that output at boot, 0 = leave it off.
#   You do NOT need to know which GPIO or which MCPWM channel anything uses,
#   and you do NOT need to stop the battery ADC yourself -- all of that lives
#   in the firmware (frozen/pwm_ext.py, function pwm_ext.on).
#
#   PWM_EXT1  GPIO 3   shares pin with battery ADC   -> battery reading goes away
#   PWM_EXT2  GPIO 47  free pin                      -> no side effect
#   PWM_EXT3  GPIO 48  shares pin with WS2812 LED    -> battery reading AND LED go away
#     (EXT3 goes through the same bpuppy_adc.stop() as EXT1, so it loses the
#      voltage reading too -- not just the LED.  See frozen/pwm_ext.py, _NEED_ADC_STOP.)
#
#   If EXT1 or EXT3 is 1, the battery voltage reads -1.0 and the battery LED
#   stays dark for the whole session -- the pin now belongs to PWM, that is
#   expected, not a fault.
#
# TO TURN IT OFF AGAIN
#   Delete /pwm_ext_on.py from the board (ViperIDE file manager, or
#   `mpremote rm :pwm_ext_on.py`).  Next boot will not start anything.
#
# MANUAL USE INSTEAD (from the REPL, without this file):
#   import pwm_ext
#   pwm_ext.on(2)               # 2 = PWM_EXT2
#   pwm_ext.set_angle(2, 90)
#   pwm_ext.off(2)

# ============================================================
# MACROS -- 1 = enable at boot, 0 = leave off
# ============================================================
EXT1 = 0        # GPIO 3   shares pin with battery ADC
EXT2 = 1        # GPIO 47  free pin
EXT3 = 0        # GPIO 48  shares pin with WS2812 LED
ANGLE = 90      # initial angle (deg) for every enabled output
# ============================================================

import pwm_ext

_WANT = {1: EXT1, 2: EXT2, 3: EXT3}
for _n, _want in _WANT.items():
    if _want:
        pwm_ext.on(_n)
        pwm_ext.set_angle(_n, ANGLE)
        print("[pwm_ext_on] PWM_EXT%d ready" % _n)
