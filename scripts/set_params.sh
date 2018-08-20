#!/bin/bash

v4l2-ctl -c brightness=-16                   # min=-64 max=64 step=1 default=-16
v4l2-ctl -c contrast=28                      # min=0 max=95 step=1 default=28
v4l2-ctl -c saturation=40                    # min=0 max=100 step=1 default=40
v4l2-ctl -c hue=0                            # min=-180 max=180 step=1 default=0
v4l2-ctl -c white_balance_temperature_auto=0 # default=1
v4l2-ctl -c gamma=100                        # min=48 max=300 step=1 default=100
v4l2-ctl -c power_line_frequency=2           # min=0 max=2 default=2
v4l2-ctl -c white_balance_temperature=4000   # min=2800 max=6500 step=1 default=4600
v4l2-ctl -c sharpness=1                      # min=1 max=7 step=1 default=1
v4l2-ctl -c backlight_compensation=0         # min=0 max=2 step=1 default=0
v4l2-ctl -c exposure_auto=0                  # min=0 max=3 default=3
#v4l2-ctl -c exposure_absolute=              # min=1 max=5000 step=1 default=333
v4l2-ctl -c exposure_auto_priority=0         # default=0
v4l2-ctl -c pan_absolute=0                   # min=-36000 max=36000 step=3600 default=0
v4l2-ctl -c tilt_absolute=0                  # min=-36000 max=36000 step=3600 default=0
v4l2-ctl -c zoom_absolute=0                  # min=0 max=10 step=1 default=0
#v4l2-ctl -c privacy=0                        # default=0
