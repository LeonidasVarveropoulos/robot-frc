class StateMachine:

    def __init__(self):
        self.state = None
        self.initial_state = None 
        self.shutdown_state = None

    def set_initial(self, initial_state):
        self.initial_state = initial_state

    def set_shutdown(self, shutdown_state):
        self.shutdown_state = shutdown_state

    # Starts the State Machine
    def start(self):
        self.state = self.initial_state
    
    # Safely shuts down the state machine
    def shutdown(self):
        self.state = self.shutdown_state

    # Main loop
    def tick(self):
        """ This is the main state machine loop """
        if (self.state is not None):
            self.state = self.state.update()
