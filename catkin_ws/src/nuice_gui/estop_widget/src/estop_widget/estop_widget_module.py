import os
import rospy
import rospkg

from std_msgs.msg import Empty

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

class EstopWidget(Plugin):

    state_sig = QtCore.pyqtSignal(int)
    restart_sig = QtCore.pyqtSignal(int)
    resume_sig = QtCore.pyqtSignal(int)

    def __init__(self, context):
        super(EstopWidget, self).__init__(context)
        self.setObjectName('EstopWidget')

        self._widget = QWidget()

        #Load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('estop_widget'), 'resource', 'estop_widget.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('EstopWidgetPluginUi')
        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the UI
        context.add_widget(self._widget)

        ####subscriers
        
        ####buttons
        self.estop_pub = rospy.Publisher("estop", Empty, queue_size=10)
        self._widget.estopBtn.clicked[bool].connect(lambda:self.estop_pub.publish(Empty()))

        self.reset_pub = rospy.Publisher("reset", Empty, queue_size=10)
        self._widget.resetBtn.clicked[bool].connect(lambda:self.restart_pub.publish(Empty()))

        self.resume_pub = rospy.Publisher("resume", Empty, queue_size=10)
        self._widget.resumeBtn.clicked[bool].connect(lambda:self.resume_pub.publish(Empty()))


    ##UI class overrides
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

