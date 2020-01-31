import wpilib
from wpilib.drive import DifferentialDrive
from robotpy_ext.control.toggle import Toggle
from ctre import *
from sensors.rev_color_sensor import REV_Color_Sensor_V3

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.frontLeft = wpilib.Spark(1)
        self.rearLeft = wpilib.Spark(2)
        self.frontRight = wpilib.Spark(0)
        self.rearRight = wpilib.Spark(3)

        self.motor1 = WPI_TalonSRX(4)
        self.motor2 = WPI_TalonSRX(5)
        self.motor3 = WPI_TalonSRX(6)
        self.motor4 = WPI_TalonSRX(7)

        self.shooter = wpilib.SpeedControllerGroup(self.motor1, self.motor2, self.motor3, self.motor4)

        self.leftSide = wpilib.SpeedControllerGroup(self.frontLeft, self.rearLeft)
        self.rightSide = wpilib.SpeedControllerGroup(self.frontRight, self.rearRight)

        self.drive = DifferentialDrive(self.leftSide, self.rightSide)
        self.timer = wpilib.Timer()
        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)

        self.driveButtonStatus = Toggle(self.leftJoystick, 2)

        self.ultrasonic = wpilib.AnalogInput(0)

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidGear = wpilib.DoubleSolenoid(0, 1)  # gear shifting
        self.Compressor.start()  # starts compressor to intake air
        
        self.colorsensor = REV_Color_Sensor_V3(wpilib.I2C.Port.kOnboard)

    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        if self.ultrasonic.getVoltage() > 0.250:
            self.drive.tankDrive(0.5, 0.45)
        else:
            self.drive.tankDrive(0, 0)

    def teleopInit(self):
        ''' function that is run at the beginning of the tele-operated phase '''
        self.colorsensor.enable()
        pass

    def teleopPeriodic(self):

        if self.xbox.getRawButton(1):
            self.Compressor.start()
        elif self.xbox.getRawButton(2):
            self.Compressor.stop()

            # gear shift toggle - press trigger on joystick to toggle
        if self.xbox.getRawButton(3):
            self.DoubleSolenoidGear.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(4):
            self.DoubleSolenoidGear.set(wpilib.DoubleSolenoid.Value.kReverse)

        self.leftAxis = self.leftJoystick.getRawAxis(1)
        self.rightAxis = self.rightJoystick.getRawAxis(1)
        self.rotateAxis = self.leftJoystick.getRawAxis(2)

        if self.driveButtonStatus.on:
            self.drive.tankDrive(-self.rightAxis * 0.65, -self.leftAxis * 0.65)
        elif self.driveButtonStatus.off:
            self.drive.arcadeDrive(-self.leftAxis * 0.65, self.rotateAxis * 0.65)

if __name__ == '__main__':
  wpilib.run(MyRobot)
