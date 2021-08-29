#!/usr/bin/env python

import sys

from nuice_dc_motor_rqt.dc_motor_plugin import DcMotorPlugin
from rqt_gui.main import Main

plugin = 'nuice_dc_motor_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))