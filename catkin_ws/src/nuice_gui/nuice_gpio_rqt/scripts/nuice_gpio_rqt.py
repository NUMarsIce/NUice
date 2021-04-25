#!/usr/bin/env python

import sys

from nuice_gpio_rqt.gpio_plugin import GPIOPlugin
from rqt_gui.main import Main

plugin = 'nuice_gpio_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))