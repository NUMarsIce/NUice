import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot

from std_msgs.msg import Float32, Bool, UInt8

#Plugin
class DcMotorPlugin(Plugin):

    motors = []
    selection = None
    current = float("nan")

    update_signal = Signal()

    set_dir_pub = None
    set_speed_pub = None
    en_pub = None
    current_sub = None


    def __init__(self, context):
        super(DcMotorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DcMotorPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_dc_motor_rqt'), 'resource', 'DcMotorPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('DcMotorPluginUi')
        
        # Number if multiple instancess
        if context.serial_number() > 1:
            self._widget.setWindowTitle('DcMotor (%d)' % context.serial_number())
        context.add_widget(self._widget)

        ### RQT signals  
        self.update_signal.connect(self.update_handler) 
        # Name input
        self._widget.reloadBtn.pressed.connect(self.reload)
        self._widget.nameBox.currentIndexChanged.connect(self.motor_selection_changed)

        self._widget.stopBtn.pressed.connect(self.stop)
        self._widget.enBox.stateChanged.connect(self.set_enable)
        self._widget.revBox.stateChanged.connect(self.set_dir)
        self._widget.posSlider.sliderReleased.connect(lambda: self.set_speed(self._widget.posSlider.value()))

        ### Setup
        self._widget.nameBox.sizeAdjustPolicy = 0 #Make it adjust to size

    def stop(self):
        self.set_speed(0)
        self._widget.posSlider.setValue(0)

    def set_enable(self, state):
        if self.en_pub is not None:
            self.en_pub.publish(Bool(state))

    def set_dir(self, state):
        if self.set_dir_pub is not None:
            self.set_dir_pub.publish(Bool(state))

    def set_speed(self, speed):
        if self.set_speed_pub is not None:
            self.set_speed_pub.publish(UInt8(speed))

    def update_handler(self):
        if(self.current == self.current): # it isnt nan
            self._widget.currentLbl.setText("%.2fA" % self.current)
        else:
            self._widget.currentLbl.setText("N/A")


    def reload(self):
        #refreshes the list of motors

        self.motors = []
        _, _, topic_type = rospy.get_master().getTopicTypes()        
        for name, typ in topic_type: 
            if "current" in name: #define motors by them having current
                self.motors.append(name[:-8]) #get namespace

        self.motors.sort() #sort

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.motors)


    def motor_selection_changed(self, idx):
        if len(self.motors) == 0:
            return
        
        if self.set_speed_pub is not None:
            self.set_speed_pub.publish(UInt8(0))

        if(idx >= 0): self.selection = self.motors[idx]
        self.unsubscribe()

        self.set_dir_pub = rospy.Publisher("{}/set_dir".format(self.selection), Bool, queue_size=10)
        self.set_speed_pub = rospy.Publisher("{}/set_speed".format(self.selection), UInt8, queue_size=10)
        self.en_pub = rospy.Publisher("{}/enable".format(self.selection), Bool, queue_size=10)
        self.current_sub = rospy.Subscriber("{}/current".format(self.selection), Float32, self.current_cb)

        self.current = float("nan")
        self._widget.posSlider.setValue(0)

        # Set name of widget
        if self.selection is not None:
            self._widget.setWindowTitle("Dc Motor: " + self.selection.split('/')[-1])

    
    ### ROS callbacks
    def current_cb(self, msg):
        self.current = msg.data
        self.update_signal.emit()
        

    ### Helpers
    def unsubscribe(self):
        if self.set_dir_pub is not None:
            self.set_dir_pub.unregister()
            self.set_speed_pub.unregister()
            self.en_pub.unregister()
            self.current_sub.unregister()


    ### RQT functions    
    def shutdown_plugin(self):
        self.unsubscribe()

    def save_settings(self, plugin_settings, instance_settings):
        # Save current selection
        if self.selection is not None:
            instance_settings.set_value('_selection', self.selection)

    def restore_settings(self, plugin_settings, instance_settings):
        # Load curent selection
        self.reload()
        if instance_settings.contains('_selection'):
            self.selection = instance_settings.value('_selection')
            self.motor_selection_changed(-1) #setup pub/subs
            if(self.selection in self.motors):
               self._widget.nameBox.setCurrentIndex(self.motors.index(self.selection))
            else:
                self.motors.insert(0, self.selection)
                self._widget.nameBox.setCurrentIndex(0)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog