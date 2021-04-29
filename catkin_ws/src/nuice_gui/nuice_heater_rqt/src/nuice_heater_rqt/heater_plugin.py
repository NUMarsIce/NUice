import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot
from python_qt_binding.QtGui import QBrush, QGradient

from std_msgs.msg import Empty, Float32, Bool, Int32, UInt16

#Plugin
class HeaterPlugin(Plugin):

    heaters = []
    temp = 0.0
    setpoint = 0.0
    heating = False

    update_signal = Signal()

    heater_set_temp_pub = None
    heater_temp_sub = None


    def __init__(self, context):
        super(HeaterPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HeaterPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_heater_rqt'), 'resource', 'HeaterPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('HeaterPluginUi')
        # Number if multiple instancess
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        #Init thermo color
        # self._widget.heat1Thermo.setFillBrush(QBrush(QGradient(QGradient.SweetPeriod)))
        # self._widget.heat1Thermo.setAlarmBrush()


        ### RQT signals  
        self.update_signal.connect(self.update_handler) 
        # Name input
        self._widget.reloadBtn.pressed.connect(self.reload)
        self._widget.nameBox.currentIndexChanged.connect(self.heater_selection_changed)
        # Buttons
        self._widget.heat1SetBtn.pressed.connect(lambda: self.set_temp(self._widget.heat1Spin.value()))
        self._widget.stopBtn.pressed.connect(lambda: self.set_temp(0.0))

    ### Signal handlers ###########################
    def set_temp(self, temp_setpoint):
        if self.heater_set_temp_pub is not None:
            self.setpoint = temp_setpoint
            self.heater_set_temp_pub.publish(Float32(temp_setpoint))

    def update_handler(self):#TODO
        self._widget.heat1Temp.setText("%d / %d" % (self.temp, self.setpoint))
        color = int(min(255, max(0, self.temp/100*255)))
        self._widget.heat1Temp.setStyleSheet("background-color: rgb(%d, %d, %d);" % (color,0,255-color))

        if(self.heating):
            self._widget.heatBox.setTitle("Heat - Heating")
            self._widget.heatBox.setStyleSheet("QGroupBox {color: rgb(200, 0, 0); font-weight: bold}")
        else:
            self._widget.heatBox.setTitle("Heat - Not Heating")
            self._widget.heatBox.setStyleSheet("QGroupBox {color: rgb(0, 0, 0); font-weight: normal}")


    def reload(self):
        #refreshes the list of heaters
        
        self.heaters = []
        for name, _ in rospy.get_published_topics():
            if "heating" in name: #define heaters by them having heating
                self.heaters.append(name[:-8]) #get namespace

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.heaters)

    def heater_selection_changed(self, idx):
        if len(self.heaters) == 0:
            return
        
        self.selection = self.heaters[idx]
        self.unsubscribe()

        self.heater_set_temp_pub = rospy.Publisher("{}/set_setpoint".format(self.selection), Float32, queue_size=10)
        self.heater_temp_sub = rospy.Subscriber("{}/current_temp".format(self.selection), Float32, self.temp_sub_cb)
        self.heater_heating_sub = rospy.Subscriber("{}/heating".format(self.selection), Bool, self.heating_sub_cb)

        self.temp = 0.0
    
    ### ROS callbacks
    def temp_sub_cb(self, msg):
        self.temp = msg.data
        self.update_signal.emit()
        
    def heating_sub_cb(self, msg): 
        self.heating = msg.data
        self.update_signal.emit()

    ### Helpers
    def unsubscribe(self):
        if self.heater_set_temp_pub is not None:
            self.heater_set_temp_pub.unregister()
            self.heater_temp_sub.unregister()
        
    def shutdown_plugin(self):
        self.unsubscribe()

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