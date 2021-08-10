from pysm import StateMachine, State, Event
import drill_machine
import melt_machine
import threading

class Carosel(StateMachine):

    worker_thread = threading.Thread(target=self.run)

    def __init__(self, name):
        super().__init__(name)
        
        # Children state machines
        self.drilling = Drill("drilling")
        self.melting = Melt("melting")
        
        # Main states
        init = State("init")
        manual = State("manual")
        repos = State("repos")
        steady = State("steady")
        
        # Sub-States
        self.add_state(init, initial=True)
        self.add_state(manual)
        self.add_state(repos)
        self.add_state(steady)

        # Sub-state transitions
        self.add_transition(self.init, self.manual, event='manual')
        self.add_transition(self.repos, self.manual, event='manual')
        self.add_transition(self.steady, self.manual, event='manual')
        self.add_transition(self.manual, self.repos, event='reposition')
        self.add_transition(self.steady, self.repos, event='reposition')
        self.add_transition(self.init, self.repos, event='initialized')
        self.add_transition(self.repos, self.steady, event='steady')

        self.init.handlers = {'enter': self.initOnEnter}
        self.repos.handlers = {'turn': self.turn,
                               'exit': self.reposExit}
        self.steady.handlers = {'exit': self.exitSteady,
                                'drill_idle': lambda state, event: self.drilling.dispatch(Event('idle')),
                                'drill_descend': lambda state, event: self.drilling.dispatch(Event('descend')),
                                'drill_retract': lambda state, event: self.drilling.dispatch(Event('retract')),
                                'drill_idle': lambda state, event: self.drilling.dispatch(Event('stopped')),
                                'melt_idle': lambda state, event: self.melt.dispatch(Event('idle')),
                                'melt_descend': lambda state, event: self.melt.dispatch(Event('descend')),
                                'melt_retract': lambda state, event: self.melt.dispatch(Event('retract')),
                                'melt_bowl': lambda state, event: self.melt.dispatch(Event('bowl')),
                                'melt_rockwell': lambda state, event: self.melt.dispatch(Event('rockwell')),
                                'melt_stop': lambda state, event: self.melt.dispatch(Event('stop'))}

        worker_thread.start()

    def turn(self, state, event):

    def reposExit(self, state, event):

    def exitSteady(self, state, event):
        


    def run(self): #TODO
        while True:
            pass





if __name__ == '__main__':
    pass