class Phase:
    def __init__(self, phase_name, phase_time, phase_thrusters):
        self.name = phase_name
        self.time = phase_time
        self.thruster_control = []

        self.populate_efforts(phase_thrusters)

    def populate_efforts(self, phase_thrusters):
        for thruster in phase_thrusters:
            self.thruster_control.append(thruster["effort"])