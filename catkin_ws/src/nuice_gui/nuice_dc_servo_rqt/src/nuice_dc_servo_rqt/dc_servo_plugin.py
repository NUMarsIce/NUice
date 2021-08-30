import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot

from std_msgs.msg import Float32, Bool, UInt8, Int32, Empty

#Plugin
class DcServoPlugin(Plugin):

    motors = []
    selection = None
    current = float("nan")
    cur_pos = float("nan")

    update_signal = Signal()

    set_pos_pub = None
    set_speed_pub = None
    en_pub = None
    current_sub = None
    zero_pub = None
    pos_sub = None

    def __init__(self, context):
        super(DcServoPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DcServoPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_dc_servo_rqt'), 'resource', 'DcServoPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('DcServoPluginUi')
        
        # Number if multiple instancess
        self._widget.setWindowTitle("DcServo")
        if context.serial_number() > 1:
            self._widget.setWindowTitle('DcServo (%d)' % context.serial_number())
        context.add_widget(self._widget)

        ### RQT signals  
        self.update_signal.connect(self.update_handler) 
        # Name input
        self._widget.reloadBtn.pressed.connect(self.reload)
        self._widget.nameBox.currentIndexChanged.connect(self.motor_selection_changed)

        self._widget.haltBtn.pressed.connect(self.halt)
        self._widget.moveBtn.pressed.connect(lambda: self.set_pos(self._widget.posBox.value()))
        self._widget.zeroBtn.pressed.connect(self.zero)
        self._widget.enBox.stateChanged.connect(self.set_enable)
        self._widget.speedBox.valueChanged.connect(self.set_speed)

        ### Setup
        self._widget.nameBox.sizeAdjustPolicy = 0 #Make it adjust to size

    def zero(self):
        if self.zero_pub is not None:
            self.zero_pub.publish(Empty())

    def halt(self):
        if self.cur_pos == self.cur_pos: # is not nan
            self.set_pos(self.cur_pos)

    def set_enable(self, state):
        if self.en_pub is not None:
            self.en_pub.publish(Bool(state))

    def set_speed(self, value):
        if self.set_speed_pub is not None:
            self.set_speed_pub.publish(UInt8(value))

    def set_pos(self, value):
        if self.set_pos_pub is not None:
            self.set_pos_pub.publish(Int32(int(value)))

    def update_handler(self):
        # Current
        if(self.current == self.current): # it isnt nan
            self._widget.currentLbl.setText("%.2fA" % self.current)
        else:
            self._widget.currentLbl.setText("N/A")

        # Position bar
        if self.cur_pos == self.cur_pos: # it isnt nan
            self._widget.posBar.setValue(int(self.cur_pos))
        else:
            self._widget.posBar.setValue(0)

    def reload(self):
        #refreshes the list of motors
        
        self.motors = []
        for name, typ in rospy.get_published_topics():
            if ("current_position" in name) and (typ == "std_msgs/Int32"): #define motors by them having set_pos
                self.motors.append(name[:-17]) #get namespace

        self.motors.sort()

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.motors)


    def motor_selection_changed(self, idx):
        if len(self.motors) == 0:
            return
        
        if self.set_speed_pub is not None:
            self.set_speed_pub.publish(UInt8(0))

        if(idx >= 0): self.selection = self.motors[idx]
        self.unsubscribe()

        self.set_pos_pub = rospy.Publisher("{}/set_pos".format(self.selection), Int32, queue_size=10)
        self.set_speed_pub = rospy.Publisher("{}/set_speed".format(self.selection), UInt8, queue_size=10)
        self.en_pub = rospy.Publisher("{}/enable".format(self.selection), Bool, queue_size=10)
        self.zero_pub = rospy.Publisher("{}/zero".format(self.selection), Empty, queue_size=10)
        self.current_sub = rospy.Subscriber("{}/current".format(self.selection), Float32, self.current_cb)
        self.pos_sub = rospy.Subscriber("{}/current_position".format(self.selection), Int32, self.pos_cb)

        self.current = float("nan")
        self.cur_pos = float("nan")

        # Set name of widget
        if self.selection is not None:
            self._widget.setWindowTitle("Dc Servo: " + self.selection.split('/')[-1])

    
    ### ROS callbacks
    def current_cb(self, msg):
        self.current = msg.data
        self.update_signal.emit()

    def pos_cb(self, msg):
        self.cur_pos = msg.data
        self.update_signal.emit()


    ### Helpers
    def unsubscribe(self):
        if self.set_pos_pub is not None:
            self.set_pos_pub.unregister()
            self.set_speed_pub.unregister()
            self.en_pub.unregister()
            self.zero_pub.unregister()
            self.current_sub.unregister()
            self.pos_sub.unregister()

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