import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QObject, Signal, Slot

from std_msgs.msg import Empty, Float32, Bool, Int32, UInt16

#Plugin
class StepperPlugin(Plugin):

    steppers = []
    selection = None
    target_pos = 0
    last_pos = 0
    current_pos = 0
    reverse_jog = False

    pos_signal = Signal()

    stepper_set_speed_pub = None
    stepper_set_accel_pub = None
    stepper_abs_pos_pub = None
    stepper_rel_pos_pub = None
    stepper_enable_pub = None
    stepper_stop_pub = None
    stepper_pos_sub = None

    def __init__(self, context):
        super(StepperPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StepperPlugin')

        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('nuice_stepper_rqt'), 'resource', 'StepperPlugin.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('StepperPluginUi')
        # Number if multiple instancess
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        ### RQT signals   
        self.pos_signal.connect(self.pos_sig_handler)
        # Name input
        self._widget.reloadBtn.pressed.connect(self.reload)
        self._widget.nameBox.currentIndexChanged.connect(self.stepper_selection_changed)
        # Enable
        self._widget.enableBox.stateChanged.connect(self.set_enable)
        # Target Position
        self._widget.goBtn.pressed.connect(self.set_position)
        # Stop
        self._widget.stopBtn.pressed.connect(self.quick_stop)
        # Jogs
        self._widget.upUpBtn.pressed.connect(lambda: self.nudge_relative(-1.0))
        self._widget.upBtn.pressed.connect(lambda: self.nudge_relative(-0.25))
        self._widget.downBtn.pressed.connect(lambda: self.nudge_relative(0.25))
        self._widget.downDownBtn.pressed.connect(lambda: self.nudge_relative(1.0))
        self._widget.reverseBox.stateChanged.connect(self.set_reverse)

        ### Setup
        self._widget.nameBox.sizeAdjustPolicy = 0 #Make it adjust to size


    ### Signal handlers ###########################
    def nudge_relative(self, scale=1.0):
        self.update_speed_accel(scale)
        if self.reverse_jog:
            scale *= -1

        pos = int(self._widget.speedInput.value()*scale/3) #go for 3rd of a second
        if self.stepper_rel_pos_pub is not None:
            self.stepper_rel_pos_pub.publish(Int32(pos))

        self.set_jog()


    def quick_stop(self):
        if self.stepper_stop_pub is not None:
            self.stepper_stop_pub.publish(Empty())
        self.set_jog()


    def set_position(self):
        self.update_speed_accel()
        if self.stepper_abs_pos_pub is not None:
            self.last_pos = self.current_pos
            self.target_pos = self._widget.positionInput.value()
            self.stepper_abs_pos_pub.publish(Int32(self.target_pos))


    def set_enable(self, state):
        if self.stepper_enable_pub is not None:
            self.stepper_enable_pub.publish(Bool(state == 2)) 


    def set_reverse(self, state):
        self.reverse_jog = (state == 2) 


    def reload(self):
        #refreshes the list of steppers
        
        self.steppers = []
        for name, _ in rospy.get_published_topics():
            if "current_position" in name: #define steppers by them having quick_stop
                self.steppers.append(name[:-17]) #get namespace

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.steppers)


    def stepper_selection_changed(self, idx):
        if len(self.steppers) == 0:
            return
        
        if(idx >= 0): self.selection = self.steppers[idx]
        self.unsubscribe()

        self.stepper_set_speed_pub = rospy.Publisher("{}/set_max_speed".format(self.selection), UInt16, queue_size=10)
        self.stepper_set_accel_pub = rospy.Publisher("{}/set_accel".format(self.selection), UInt16, queue_size=10)
        self.stepper_abs_pos_pub = rospy.Publisher("{}/set_abs_pos".format(self.selection), Int32, queue_size=10)
        self.stepper_rel_pos_pub = rospy.Publisher("{}/set_rel_pos".format(self.selection), Int32, queue_size=10)
        self.stepper_enable_pub = rospy.Publisher("{}/set_enabled".format(self.selection), Bool, queue_size=10)
        self.stepper_stop_pub = rospy.Publisher("{}/quick_stop".format(self.selection), Empty, queue_size=10)
        self.stepper_pos_sub = rospy.Subscriber("{}/current_position".format(self.selection), Int32, self.position_cb)

        self.set_enable(False)
        self._widget.enableBox.setChecked(False)

        self.target_pos = 0
        self.last_pos = 0
        self.current_pos = 0
        self._widget.accelInput.setValue(400)
        self._widget.speedInput.setValue(400)
        self.set_jog()

        # Set name of widget
        self._widget.setWindowTitle("Stepper: " + self.selection.split('/')[-1])


    def pos_sig_handler(self):
        # set bar
        if self.last_pos < self.target_pos:
            self._widget.positionBar.setMinimum(self.last_pos)
            self._widget.positionBar.setMaximum(self.target_pos)
            self._widget.positionBar.setValue(self.current_pos)
        else:
            self._widget.positionBar.setMinimum(self.target_pos)
            self._widget.positionBar.setMaximum(self.last_pos)
            self._widget.positionBar.setValue(self.current_pos)
        self._widget.positionBar.setFormat(str(self.current_pos))


    ### ROS callbacks
    def position_cb(self, msg):
        self.current_pos = msg.data
        self.pos_signal.emit()


    ### Helpers
    def set_jog(self):
        self.target_pos = self.current_pos
        self.last_pos = self.current_pos
        self.pos_signal.emit()

    
    def update_speed_accel(self, speed_scale=1.0):
        if None not in {self.stepper_set_speed_pub, self.stepper_set_accel_pub}:
            self.stepper_set_speed_pub.publish(UInt16(int(abs(self._widget.speedInput.value()*speed_scale))))
            self.stepper_set_accel_pub.publish(UInt16(int(abs(self._widget.accelInput.value()))))

    
    def unsubscribe(self):
        if self.stepper_set_speed_pub is not None:
            #Stop current motor
            self.stepper_stop_pub.publish(Empty())

            #unregister stuff
            self.stepper_set_speed_pub.unregister()
            self.stepper_set_accel_pub.unregister()
            self.stepper_abs_pos_pub.unregister()
            self.stepper_rel_pos_pub.unregister()
            self.stepper_enable_pub.unregister()
            self.stepper_stop_pub.unregister()
            self.stepper_pos_sub.unregister()


    def shutdown_plugin(self):
        self.unsubscribe()


    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('_selection', self.selection)
        instance_settings.set_value('_reverse', str(self.reverse_jog))
        instance_settings.set_value('_speed', str(self._widget.speedInput.value()))
        instance_settings.set_value('_accel', str(self._widget.accelInput.value()))


    def restore_settings(self, plugin_settings, instance_settings):
        # Load curent selection
        if instance_settings.contains('_selection'):
            self.reload()
            self.selection = instance_settings.value('_selection')
            self.stepper_selection_changed(-1) #setup pub/subs
            if(self.selection in self.steppers):
               self._widget.nameBox.setCurrentIndex(self.steppers.index(self.selection))
            else:
                self.steppers.insert(0, self.selection)
                self._widget.nameBox.setCurrentIndex(0)

            # Load other
            self._widget.accelInput.setValue(int(instance_settings.value('_accel')))
            self._widget.speedInput.setValue(int(instance_settings.value('_speed')))
            self._widget.reverseBox.setChecked(instance_settings.value('_reverse')=='True')
            self.reverse_jog = instance_settings.value('_reverse')=='True'

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog