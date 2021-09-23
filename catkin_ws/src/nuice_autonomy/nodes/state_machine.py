#!/usr/bin/env python

from pysm import StateMachine, State, Event
import drill_machine
import melt_machine
import threading
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import UInt16
from nuice_msgs.srv import FloatCommand, FloatCommandResponse




class Carosel(StateMachine):

    def __init__(self, name):
        super(Carosel,self).__init__(name)
        self.worker_thread = threading.Thread(target=self.run)
        self.goal = 0
        self.repos_goal = 0
        self.carosel_position = 0
        self.repositioning = False
        drill_motion_pub = rospy.Publisher("/central_board/drill_stp/set_abs_pos", Int32, queue_size = 10)
        drill_rel_motion_pub = rospy.Publisher("/central_board/drill_stp/set_rel_pos", Int32, queue_size = 10)
        drill_speed_pub = rospy.Publisher("/central_board/drill_stp/set_max_speed", UInt16, queue_size = 10)
        drill_accel_pub = rospy.Publisher("/central_board/drill_stp/set_accel", UInt16, queue_size = 10)
        drill_stop_pub = rospy.Publisher("/central_board/drill_stp/quick_stop", Empty, queue_size = 10)
        drill_pub = rospy.Publisher("/central_board/drill_relay/set_state", Bool, queue_size = 10)
        melt_motion_pub = rospy.Publisher("/central_board/probe_stp/set_abs_pos", Int32, queue_size = 10)
        melt_rel_motion_pub = rospy.Publisher("/central_board/probe_stp/set_rel_pos", Int32, queue_size = 10)
        melt_speed_pub = rospy.Publisher("/central_board/probe_stp/set_max_speed", UInt16, queue_size = 10)
        melt_accel_pub = rospy.Publisher("/central_board/probe_stp/set_accel", UInt16, queue_size = 10)
        melt_stop_pub = rospy.Publisher("/central_board/probe_stp/quick_stop", Empty, queue_size = 10)
        rospy.wait_for_service('set_probe1')
        probe_1_service = rospy.ServiceProxy('set_probe1', FloatCommand)
        rospy.wait_for_service('set_probe2')
        probe_2_service = rospy.ServiceProxy('set_probe2', FloatCommand)
        self.caroselPub = rospy.Publisher("/movement_board/carosel/set_pos", Int32, queue_size = 10)

        
        
        # Children state machines
        self.drill = drill_machine.Drill("drilling", drill_motion_pub, drill_rel_motion_pub, drill_speed_pub, drill_accel_pub, drill_stop_pub, drill_pub)
        self.melt = melt_machine.Melt("melting", melt_motion_pub, melt_rel_motion_pub, melt_speed_pub, melt_accel_pub, melt_stop_pub, probe_1_service, probe_2_service)

        rospy.Subscriber('/central_board/drill_stp/current_position', Int32, self.drill.drillPositionCallback)
        rospy.Subscriber('/central_board/drill_limit/current_state', Bool, self.drill.drillLimitCallback)
        rospy.Subscriber('/central_board/probe_stp/current_position', Int32, self.melt.meltPositionCallback)
        rospy.Subscriber('/central_board/probe_limit/current_state', Bool, self.melt.meltLimitCallback)
        self.caroselSub = rospy.Subscriber("/movement_board/carosel/current_position", Int32, self.caroselPositionCallback)
        rospy.Subscriber('/ac/goal', Int32, self.goalCallback)
        rospy.Subscriber('/ac/events', String, self.dispatchEvent)
        
        # Main states
        steady = State("steady")
        raising_tools = State("raising_tools")
        repositioning = State("repositioning")
        braking = State("braking")
        
        
        self.add_state(steady, initial=True)
        self.add_state(raising_tools)
        self.add_state(repositioning)
        self.add_state(braking)

        # State transitions
        self.add_transition(steady, raising_tools, events=['repos'])
        self.add_transition(raising_tools, repositioning, events=['done_raising_tools'])
        self.add_transition(repositioning, braking, events=['steady'])
        self.add_transition(braking, steady, events=['done_braking'])

        #Event handlers
        steady.handlers = {'drill_idle': lambda state, event: self.drill.dispatch(event),
                           'drill_drill': lambda state, event: self.drill.dispatch(event),
                           'drill_stop': lambda state, event: self.drill.dispatch(event),
                           'melt_idle': lambda state, event: self.melt.dispatch(event),
                           'melt_melt': lambda state, event: self.melt.dispatch(event),
                           'melt_stop': lambda state, event: self.melt.dispatch(event),
                           'melt_probe_1' : lambda state, event: self.melt.dispatch(event),
                           'melt_probe_2' : lambda state, event: self.melt.dispatch(event)}
        raising_tools.handlers = {'enter': self.raiseTools}
        repositioning.handlers = {'enter' : self.enterRepositioning,
                                  'repos' : self.updateRepositioning,
                                  'exit' : self.exitRepositioning}
        braking.handlers = {'enter' : self.enterBraking,
                            'exit' : self.exitBraking}
        self.rate = rospy.Rate(20)
        self.worker_thread.start()

    #Subscriber callbacks
    def goalCallback(self, goal_data):
        self.goal = goal_data.data

    def dispatchEvent(self, event_data):
        self.dispatch(Event(event_data.data, goal=self.goal))

    #Action functions
    def caroselPositionCallback(self, carosel_position_data):
        self.carosel_position = carosel_position_data.data

    def raiseTools(self, state, event):
        print "raising tools"
        self.drill.dispatch(Event('drill_idle'))
        self.melt.dispatch(Event('melt_idle'))
        while (not self.drill.atZero) and (not self.melt.atZero):
            sleep(1)
        self.repos_goal = event.cargo['source_event'].cargo['goal']
        print "raised tools"

    def enterRepositioning(self, state, event):
        self.repositioning = True
        print "entered repositioning"

    def updateRepositioning(self, state, event):
        self.repos_goal = event.cargo['goal']
        print "updated repositioning"

    def exitRepositioning(self, state, event):
        self.repositioning = False
        print "exited repositioning"

    def enterBraking(self, state, event):
        print "entered braking"

    def exitBraking(self, state, event):
        print "exited braking"

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.repositioning and (not (self.carosel_position == self.repos_goal)):
                self.caroselPub.publish(self.repos_goal)







if __name__ == '__main__':
    
    rospy.init_node('autonomy_core')
    state_machine = Carosel("carosel")
    state_machine.initialize()
    
    while not rospy.is_shutdown():
        rospy.spin()    




