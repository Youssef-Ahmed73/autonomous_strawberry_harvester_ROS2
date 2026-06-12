class StateBase:
    def __init__(self, name):
        self.name = name

    def enter(self, context):
        pass

    def execute(self, context):
        raise NotImplementedError("Every state must implement the execute method.")

    def exit(self, context):
        pass