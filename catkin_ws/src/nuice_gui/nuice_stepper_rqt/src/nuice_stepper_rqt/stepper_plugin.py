import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from std_msgs.msg import Empty, Float32, Bool, Int32, UInt16

class StepperPlugin(Plugin):

    steppers = []
    selection = None
    target_pos = 0
    last_pos = 0
    current_pos = 0
    jog_mode = True

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

    ### Signal handlers ###########################
    def nudge_relative(self, scale=1.0):
        self.update_speed_accel(scale)

        pos = int(self._widget.speedInput.value()*scale/5) #go for 5th of a second
        if self.stepper_rel_pos_pub is not None:
            self.stepper_rel_pos_pub.publish(Int32(pos))

        self.jog_mode = True


    def quick_stop(self):
        if self.stepper_stop_pub is not None:
            self.stepper_stop_pub.publish(Empty())
        self.jog_mode = True

    def set_position(self):
        self.update_speed_accel()
        if self.stepper_abs_pos_pub is not None:
            self.last_pos = self.current_pos
            self.target_pos = self._widget.positionInput.value()
            self.stepper_abs_pos_pub.publish(Int32(self.target_pos))
        self.jog_mode = False

    def set_enable(self, state):
        if self.stepper_enable_pub is not None:
            self.stepper_enable_pub.publish(Bool(state == 2)) 

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
        
        self.selection = self.steppers[idx]
        self.unsubscribe()

        self.stepper_set_speed_pub = rospy.Publisher("{}/set_max_speed".format(self.selection), UInt16, queue_size=10)
        self.stepper_set_accel_pub = rospy.Publisher("{}/set_accel".format(self.selection), UInt16, queue_size=10)
        self.stepper_abs_pos_pub = rospy.Publisher("{}/set_abs_pos".format(self.selection), Int32, queue_size=10)
        self.stepper_rel_pos_pub = rospy.Publisher("{}/set_rel_pos".format(self.selection), Int32, queue_size=10)
        self.stepper_enable_pub = rospy.Publisher("{}/set_enabled".format(self.selection), Bool, queue_size=10)
        self.stepper_stop_pub = rospy.Publisher("{}/quick_stop".format(self.selection), Empty, queue_size=10)
        self.stepper_pos_sub = rospy.Subscriber("{}/current_position".format(self.selection), Int32, self.position_cb)

        self.set_enable(True)
        self._widget.enableBox.setChecked(True)

        self.target_pos = 0
        self.last_pos = 0
        self.current_pos = 0
        self._widget.accelInput.setValue(200)
        self._widget.speedInput.setValue(400)
        self.jog_mode = True

    ### ROS callbacks
    def position_cb(self, msg):
        self.current_pos = msg.data
        # hide bar if jogging or data is invald
        if self.jog_mode or not (self.target_pos > self.current_pos > self.last_pos or self.target_pos < self.current_pos < self.last_pos):
            if not self._widget.positionBar.isHidden():
                self._widget.positionBar.hide()
            return
        elif self._widget.positionBar.isHidden():
           self._widget.positionBar.show()

        # set bar
        if self.target_pos > self.last_pos:
            self._widget.positionBar.setMaximum(self.target_pos)
            self._widget.positionBar.setMinimum(self.last_pos)
            self._widget.positionBar.setValue(self.current_pos)
        elif self.target_pos < self.last_pos:
            self._widget.positionBar.setMaximum(self.last_pos)
            self._widget.positionBar.setMinimum(self.target_pos)
            self._widget.positionBar.setValue(self.current_pos)
        else:
            self._widget.positionBar.setMaximum(0)
            self._widget.positionBar.setMinimum(0)
            self._widget.positionBar.setValue(0)
    

    ### Helpers
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