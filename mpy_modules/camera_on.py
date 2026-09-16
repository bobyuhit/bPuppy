# /camera_on.py -- bPuppy boot switch: auto-start web page + camera on power-up.
#
# HOW IT WORKS
#   The firmware's frozen main.py looks for /camera_on.py on the board's
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
# START WITH OR WITHOUT THE CAMERA
#   camera_stream.start(stream=True)   -> web page + camera on  (uses more power)
#   camera_stream.start()              -> web page only; open the camera later
#                                         with the video button on the page
#
# TO TURN IT OFF AGAIN
#   Delete /camera_on.py from the board (ViperIDE file manager, or
#   `mpremote rm :camera_on.py`).  Next boot will not start anything.
#
# MANUAL USE INSTEAD (from the REPL, without this file):
#   import camera_stream; camera_stream.start()
#   camera_stream.stop()

import camera_stream
camera_stream.start(stream=True)
