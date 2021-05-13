from pysm import StateMachine, State, Event
class Melt(StateMachine):
    
    worker_thread = threading.Thread(target=self.run)

    def __init__(self, name):
        super().__init__(name)

        idle = State("idle")
        descending = State("descending")
        retracting = State("retracting")
        rockwell = State("rockwell")
        bowl = State("bowl")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(descending)
        self.add_state(rockwell)
        self.add_state(bowl)
        self.add_state(retracting)
        self.add_state(stopped)

        self.add_transition(idle, descending, event='descend')
        self.add_transition(retracting, descending, event='descend')
        self.add_transition(stopped, descending, event='descend')
        self.add_transition(rockwell, descending, event='descend')
        self.add_transition(bowl, descending, event='descend')

        self.add_transition(idle, stopped, event='stop')
        self.add_transition(descending, stopped, event='stop')
        self.add_transition(retracting, stopped, event='stop')
        self.add_transition(rockwell, stopped, event='stop')
        self.add_transition(bowl, stopped, event='stop')

        self.add_transition(descending, retracting, event='retract')
        self.add_transition(stopped, retracting, event='retract')
        self.add_transition(rockwell, retracting, event='retract')
        self.add_transition(bowl, retracting, event='retract')

        self.add_transition(stopped, idle, event='idle')
        self.add_transition(drilling, idle, event='idle')
        self.add_transition(retracting, idle, event='idle')
        self.add_transition(rockwell, idle, event='idle')
        self.add_transition(bowl, idle, event='idle')

        self.add_transition(descending, rockwell, event='rockwell')
        self.add_transition(retracting, rockwell, event='rockwell')
        self.add_transition(stopped, rockwell, event='rockwell')
        self.add_transition(bowl, rockwell, event='rockwell')

        self.add_transition(stopped, bowl, event='bowl')

        self.idle.handlers = {'enter' : self.idleOnEnter}
        self.stopped.handlers = {'stopped' : self.stoppedOnEnter}
        self.descending.handlers = {'descend' : self.descendUpdateSpeed}
        self.retracting.handlers = {'retract' : self.retractUpdateSpeed}
        self.bowl.handers = { 'enter' : self.bowlOnEnter,
                              'exit' : self.bowlOnExit}
        self.rockwell.handlers = { 'enter' : self.rockwellOnEnter,
                                   'exit' : self.rockwellOnExit}        
        
        worker_thread.start()

    def run(self): #TODO
        while True:
            pass
