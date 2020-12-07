import os
import rospy
import rospkg

from std_msgs.msg import Empty

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

class MotorWidget(Plugin):

    def __init__(self, context):
        super(MotorWidget, self).__init__(context)
        self.setObjectName('MotorWidget')

        self._widget = QWidget()

        #Load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('motor_widget'), 'resource', 'motor_widget.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MotorWidgetPluginUi')
        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the UI
        context.add_widget(self._widget)

        ####Ros code
        

    ##UI class overrides
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

