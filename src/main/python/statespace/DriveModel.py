import frccontrol as frccnt

class DriveModel:
    def setConstants(self, isHighGear):
        # The motor used
        motor = frccnt.models.MOTOR_CIM
        # Number of motors per side
        num_motors = 2.0

        # Gearing constants and stuff
        self.motor = frccnt.models.gearbox(motor, num_motors)

        # High and low gear ratios of differential drive
        Glow = 11.832
        Ghigh = 6.095
        if(isHighGear):
            self.G = Ghigh
        else:
            self.G = Glow

        # Drivetrain mass in kg
        self.m = 130 * 0.45
        # Radius of wheels in meters
        self.r = 4.0 * 0.0254
        # Radius of robot in meters
        self.rb = 26.0 * 0.0254 / 2.0
        # Moment of inertia of the differential drive in kg-m^2
        self.J = 6.0

        self.C1 = -G ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
        self.C2 = G * motor.Kt / (motor.R * r)
        self.C3 = -G ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
        self.C4 = G * motor.Kt / (motor.R * r)
