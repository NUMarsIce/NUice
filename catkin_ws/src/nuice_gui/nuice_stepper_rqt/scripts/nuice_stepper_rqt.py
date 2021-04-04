#!/usr/bin/env python

import sys

from nuice_stepper_rqt.stepper_plugin import StepperPlugin
from rqt_gui.main import Main

plugin = 'nuice_stepper_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))