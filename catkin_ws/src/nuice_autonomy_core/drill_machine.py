from pysm import StateMachine, State, Event
class Drill(StateMachine):

    worker_thread = threading.Thread(target=self.run)

    def __init__(self, name):
        super().__init__(name)

        idle = State("idle")
        descending = State("descending")
        retracting = State("retracting")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(descending)
        self.add_state(retracting)
        self.add_state(stopped)

        self.add_transition(self.descending, self.idle, event='idle')
        self.add_transition(self.retracting, self.idle, event='idle')
        self.add_transition(self.stopped, self.idle, event='idle')

        self.add_transition(self.idle, self.descending, event='descend')
        self.add_transition(self.retracting, self.descending, event='descend')
        self.add_transition(self.stopped, self.descending, event='descend')

        self.add_transition(self.descending, self.retracting, event='retracting')
        self.add_transition(self.stopped, self.retracting, event='retracting')
        
        self.add_transition(self.idle, self.stopped, event='stopped')
        self.add_transition(self.descending, self.stopped, event='stopped')
        self.add_transition(self.retracting, self.stopped, event='stopped')

        self.idle.handlers = {'enter' : self.idleOnEnter}
        self.descending.handlers = {'descend' : self.descendUpdateSpeed}
        self.ascending.handlers = {'ascend' : self.ascendUpdateSpeed}
        self.stop.handlers = {'enter' : self.stopOnEnter}

        worker_thread.start()

    def run(self): #TODO
        while True:
            pass