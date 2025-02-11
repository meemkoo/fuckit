import wpilib.simulation
import wpilib, wpimath, math, commands2
import wpimath.controller
import wpimath.system
import wpimath.system.plant
import wpimath.trajectory
import phoenix6.hardware
from dumbdashboard import DumbDashboard


class Elevator(commands2.Subsystem):
    def __init__(self):
        self.m0 = phoenix6.hardware.TalonFX(14)
        self.m1 = phoenix6.hardware.TalonFX(15)

        self.upperSoftLimit = 0

        self.gearRatio = 7.75
        self.sprocketRadius = 0.0444/2
        self.carriageRatio = 3

        self.motor_configs = phoenix6.configs.TalonFXConfiguration() \
            .with_audio(phoenix6.configs.AudioConfigs().with_beep_on_boot(True).with_beep_on_config(True)) \
            .with_current_limits(phoenix6.configs.CurrentLimitsConfigs() \
                .with_supply_current_limit(40) \
                .with_supply_current_limit_enable(True)) \
            .with_motor_output(phoenix6.configs.MotorOutputConfigs() \
                .with_inverted(phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE) \
                .with_neutral_mode(phoenix6.signals.NeutralModeValue.BRAKE)) \
            .with_slot0(phoenix6.configs.Slot0Configs() \
                .with_k_p(0.5) \
                .with_k_i(0.5) \
                .with_k_d(0) \
                .with_k_g(0) \
                .with_k_s(0) \
                .with_k_v(0) \
                .with_k_a(0) \
                .with_gravity_type(phoenix6.signals.GravityTypeValue.ELEVATOR_STATIC)) \
            .with_software_limit_switch(phoenix6.configs.SoftwareLimitSwitchConfigs() \
                .with_reverse_soft_limit_threshold(0) \
                .with_reverse_soft_limit_enable(True) \
                .with_forward_soft_limit_threshold(self.upperSoftLimit) \
                .with_forward_soft_limit_enable(False)) \
            .with_feedback(phoenix6.configs.FeedbackConfigs() \
                .with_sensor_to_mechanism_ratio(self.gearRatio)) \
            .with_motion_magic(phoenix6.configs.MotionMagicConfigs() \
                .with_motion_magic_acceleration(10))

        self.m0.configurator.apply(self.motor_configs)
        self.m1.configurator.apply(self.motor_configs)

        self.m1.set_control(phoenix6.controls.Follower(14, False))

        self.setpoint = 0
        self.at_setpoint = commands2.button.Trigger(lambda: abs(self.m0.get_position().value_as_double - self.setpoint) < 0.1)

        self.elevatorSim = wpilib.simulation.ElevatorSim(   
            wpimath.system.plant.DCMotor.krakenX60(2),
            self.gearRatio,
            10, self.sprocketRadius, 0, 1.87, True, 0)
        
        self.refresh_motors(True)

    def periodic(self):
        self.m1.set_control(phoenix6.controls.Follower(14, False))

        # DumbDashboard.put("elevator setpoint", self.setpoint)
        # DumbDashboard.put("elevator1 pos", self.m0.get_position().value_as_double)
        # DumbDashboard.put("elevator2 pos", self.m1.get_position().value_as_double)
        # DumbDashboard.put("elevator1 output", self.m0.get_position().value_as_double)
        # DumbDashboard.put("elevator2 output", self.m1.get_position().value_as_double)

    def simulationPeriodic(self):
        talonFXSim = self.m0.sim_state

        talonFXSim.set_supply_voltage(wpilib.RobotController.getBatteryVoltage())

        motorVoltage = talonFXSim.motor_voltage

        self.elevatorSim.setInputVoltage(motorVoltage)
        self.elevatorSim.update(0.020)

        talonFXSim.set_raw_rotor_position(self.elevatorSim.getPosition()*self.gearRatio* self.carriageRatio /(self.sprocketRadius *2*math.pi))
        talonFXSim.set_rotor_velocity(self.elevatorSim.getVelocity()*self.gearRatio* self.carriageRatio /(self.sprocketRadius *2*math.pi))

    def set(self, speed):
        return self.run(lambda: self.m0.set(speed))

    def goToPos(self, pos):
        def inner():
            rotations = pos / (math.pi * 2 * self.sprocketRadius * self.carriageRatio)
            self.setpoint = rotations
            self.m0.set_control(phoenix6.controls.MotionMagicExpoVoltage(rotations))
        return self.run(inner).until(self.at_setpoint).finallyDo(lambda _: print("The elevator should be where its supposed to NOW! :)))"))
    
    def getHeight(self):
        return self.m0.get_position().value_as_double* self.sprocketRadius *2*math.pi
    


    def refresh_motors(self, set=False):
        if set:
            self.motor_configs.slot0.k_p = DumbDashboard.put('k_p', 0, float)
            self.motor_configs.slot0.k_i = DumbDashboard.put('k_i', 0, float)
            self.motor_configs.slot0.k_d = DumbDashboard.put('k_d', 0, float)
            self.motor_configs.slot0.k_g = DumbDashboard.put('k_g', 0, float)


        self.motor_configs.slot0.k_p = DumbDashboard.grab('k_p', 0, float)
        self.motor_configs.slot0.k_i = DumbDashboard.grab('k_i', 0, float)
        self.motor_configs.slot0.k_d = DumbDashboard.grab('k_d', 0, float)
        self.motor_configs.slot0.k_g = DumbDashboard.grab('k_g', 0, float)


class Robot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()
        self.elevator = Elevator()
        DumbDashboard.put('Slag', 31, float)
        # commands2.button.Trigger()
        xctl = commands2.button.CommandXboxController(0)
        xctl.a().onTrue(self.elevator.goToPos(0.4))
        xctl.b().onTrue(commands2.cmd.run(lambda: self.elevator.refresh_motors(False)))

    def teleopPeriodic(self):
        print(DumbDashboard.grab('Slag', 12, float))
