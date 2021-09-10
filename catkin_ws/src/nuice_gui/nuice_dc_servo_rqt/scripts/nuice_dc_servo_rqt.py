#!/usr/bin/env python

import sys

from nuice_dc_servo_rqt.dc_servo_plugin import DcServoPlugin
from rqt_gui.main import Main

plugin = 'nuice_dc_servo_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))