import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class StepperPlugin(Plugin):

    steppers = []

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

        ### Publishers and Subscribers

        ### Name input
        self._widget.reloadBtn.clicked[bool].connect(self.reload)
        

    ### Signal handlers ###########################
    def reload(self, var):
        #refreshes the list of steppers

        self.steppers = []
        for name, datatype in rospy.get_published_topics():
            if "quick_stop" in name: #deffine steppers by having quick_stop
                self.steppers.append(name[:-11]) #get namespace

        self._widget.nameBox.clear()
        self._widget.nameBox.addItems(self.steppers)

        print("YEEET")



    def pos_sub_sig_cb(self, pos):
        pass #TODO

    ### RQT callbacks

    ### Helpers
    #Unsubscribes ros stuff
    def unsubscribe(self):
        pass #TODO

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