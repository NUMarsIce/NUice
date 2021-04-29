#!/usr/bin/env python

import sys

from nuice_heater_rqt.heater_plugin import HeaterPlugin
from rqt_gui.main import Main

plugin = 'nuice_heater_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))