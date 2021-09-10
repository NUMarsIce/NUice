import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot
from python_qt_binding.QtGui import QBrush, QGradient

from std_msgs.msg import Empty, Float32, Bool, Int32, UInt16
from nuice_msgs.srv import *

#Plugin
class HeaterPlugin(Plugin):

    temp1 = 0.0
    temp2 = 0.0
    setpoint1 = 0.0
    setpoint2 = 0.0
    
    update_signal = Signal()

    def __init__(self, context):
        super(HeaterPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HeaterPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_heater_rqt'), 'resource', 'HeaterPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('HeaterPluginUi')
        self._widget.setWindowTitle("Probe Heaters")
        # Number if multiple instancess
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        #Init thermo color
        # self._widget.heat1Thermo.setFillBrush(QBrush(QGradient(QGradient.SweetPeriod)))
        # self._widget.heat1Thermo.setAlarmBrush()


        ### RQT signals  
        self.update_signal.connect(self.update_handler) 
        # Buttons
        self._widget.heat1SetBtn.pressed.connect(lambda: self.set_temp1(self._widget.heat1Spin.value()))
        self._widget.stop1Btn.pressed.connect(lambda: self.set_temp1(0.0))
        self._widget.heat2SetBtn.pressed.connect(lambda: self.set_temp2(self._widget.heat2Spin.value()))
        self._widget.stop2Btn.pressed.connect(lambda: self.set_temp2(0.0))

        ### ROS
        self.heat1_srv = rospy.ServiceProxy('set_probe1', FloatCommand)
        self.heat1_sub = rospy.Subscriber("/melt_board/probe_therm1/value", Int32, self.temp1_sub_cb)
        self.heat2_srv = rospy.ServiceProxy('set_probe2', FloatCommand)
        self.heat2_sub = rospy.Subscriber("/melt_board/probe_therm2/value", Int32, self.temp2_sub_cb)



    ### Signal handlers ###########################
    def set_temp1(self, temp_setpoint):
        try:
            self.setpoint1 = temp_setpoint
            self.heat1_srv(float(temp_setpoint))
            self.update_signal.emit()
        except rospy.ServiceException:
            return

    def set_temp2(self, temp_setpoint):
        try:
            self.setpoint2 = temp_setpoint
            self.heat2_srv(float(temp_setpoint))
            self.update_signal.emit()
        except rospy.ServiceException:
            return

    def update_handler(self):
        ## Heat1 colors
        self._widget.heat1Temp.setText("%d / %d" % (self.temp1, self.setpoint1))
        color = int(min(255, max(0, self.temp1/150.0*255)))
        self._widget.heat1Temp.setStyleSheet("background-color: rgb(%d, %d, %d);" % (color,0,255-color))

        if(self.temp1 < self.setpoint1):
            self._widget.heat1Box.setTitle("Heat1 - ON ")
            self._widget.heat1Box.setStyleSheet("QGroupBox {color: rgb(200, 0, 0); font-weight: bold}")
        else:
            self._widget.heat1Box.setTitle("Heat1 - OFF")
            self._widget.heat1Box.setStyleSheet("QGroupBox {color: rgb(0, 0, 0); font-weight: normal}")

        ## Heat2 colors
        self._widget.heat2Temp.setText("%d / %d" % (self.temp2, self.setpoint2))
        color = int(min(255, max(0, self.temp2/150.0*255)))
        self._widget.heat2Temp.setStyleSheet("background-color: rgb(%d, %d, %d);" % (color,0,255-color))

        if(self.temp2 < self.setpoint2):
            self._widget.heat2Box.setTitle("Heat2 - ON ")
            self._widget.heat2Box.setStyleSheet("QGroupBox {color: rgb(200, 0, 0); font-weight: bold}")
        else:
            self._widget.heat2Box.setTitle("Heat2 - OFF")
            self._widget.heat2Box.setStyleSheet("QGroupBox {color: rgb(0, 0, 0); font-weight: normal}")


    ### ROS callbacks
    def temp1_sub_cb(self, msg):
        self.temp1 = msg.data
        self.update_signal.emit()


    def temp2_sub_cb(self, msg):
        self.temp2 = msg.data
        self.update_signal.emit()


    def shutdown_plugin(self):
        pass


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass


    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog