import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot

from std_msgs.msg import Empty, Float32, Bool, Int32, UInt16

#Plugin
class GPIOPlugin(Plugin):

    gpios = []
    state = False

    update_signal = Signal()

    gpio_set_state_pub = None
    gpio_state_sub = None


    def __init__(self, context):
        super(GPIOPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GPIOPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_gpio_rqt'), 'resource', 'GPIOPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('GPIOPluginUi')
        # Number if multiple instancess
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        ### RQT signals  
        self.update_signal.connect(self.update_handler) 
        # Name input
        self._widget.reloadBtn.pressed.connect(self.reload)
        self._widget.nameBox.currentIndexChanged.connect(self.gpio_selection_changed)
        # Buttons
        self._widget.toggleBtn.pressed.connect(lambda: self.set_state(not self.state))
        self._widget.lowBtn.pressed.connect(lambda: self.set_state(False))
        self._widget.highBtn.pressed.connect(lambda: self.set_state(True))

    ### Signal handlers ###########################
    def set_state(self, state):
        if self.gpio_set_state_pub is not None:
            self.gpio_set_state_pub.publish(Bool(state))

    def update_handler(self):
        self._widget.stateLbl.setText("HIGH" if self.state else "LOW")

        if self.state:
            self._widget.stateLbl.setStyleSheet("background-color: red")
        else:
            self._widget.stateLbl.setStyleSheet("background-color: blue")

    def reload(self):
        #refreshes the list of gpios
        
        self.gpios = []
        for name, _ in rospy.get_published_topics():
            if "current_state" in name: #define gpios by them having current_state
                self.gpios.append(name[:-14]) #get namespace

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.gpios)

    def gpio_selection_changed(self, idx):
        if len(self.gpios) == 0:
            return
        
        self.selection = self.gpios[idx]
        self.unsubscribe()

        self.gpio_set_state_pub = rospy.Publisher("{}/set_state".format(self.selection), Bool, queue_size=10)
        self.gpio_state_sub = rospy.Subscriber("{}/current_state".format(self.selection), Bool, self.state_sub_cb)

        self.state = False
    
    ### ROS callbacks
    def state_sub_cb(self, msg):
        self.state = msg.data
        self.update_signal.emit()
        

    ### Helpers
    def unsubscribe(self):
        if self.gpio_set_state_pub is not None:
            self.gpio_set_state_pub.unregister()
            self.gpio_state_sub.unregister()
        
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