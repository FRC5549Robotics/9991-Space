import wpilib
from wpilib import DriverStation
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from networktables import NetworkTables
from navx import AHRS
import logging


class MyRobot(wpilib.TimedRobot):

    # values for navx
    kP = 0.03
    kI = 0.00
    kD = 0.00
    kF = 0.00

    kToleranceDegrees = 2.0

    def robotInit(self):
        """Robot initialization function"""
        self.ahrs = AHRS.create_spi()

        turnController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.ahrs, output=self)
        turnController.setInputRange(-180, 180)
        turnController.setOutputRange(-0.6, 0.6)
        turnController.setAbsoluteTolerance(self.kToleranceDegrees)
        turnController.setContinuous(True)

        self.turnController = turnController
        self.rotateToAngleRate = 0

        # object that handles basic drive operations
        self.rearLeftMotor = wpilib.Spark(0)
        self.frontLeftMotor = wpilib.Spark(1)
        self.frontRightMotor = wpilib.Spark(3)
        self.rearRightMotor = wpilib.Spark(2)

        # defining motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        # defines timer for autonomous
        self.timer = wpilib.Timer()

        # joystick 0, 1, and 2 on the driver station
        self.left_stick = wpilib.Joystick(0)
        self.right_stick = wpilib.Joystick(1)
        self.joystick = wpilib.Joystick(2)

        # initialization of the FMS
        self.DS = DriverStation.getInstance()

        # pneumatics init
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoid = wpilib.DoubleSolenoid(0, 1)
        self.Compressor.start()

        # logging and smart dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.99.91.2')

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.ahrs.reset()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # gets randomization of field elements
        gameData = self.DS.getGameSpecificMessage()
        # gets location of robot on the field
        position = self.DS.getLocation()

        # SmartDashboard modules
        self.sd.putNumber("Gyro angle: ", self.ahrs.getAngle())

        def gearTest():
            if self.timer.get() <= 1800:
                self.Compressor.start()
                self.rearLeftMotor.set(0.85)
                self.frontLeftMotor.set(0.85)
                self.frontRightMotor.set(0.85)
                self.rearRightMotor.set(0.85)
            else:
                self.Compressor.stop()
                self.rearLeftMotor.set(0)
                self.frontLeftMotor.set(0)
                self.frontRightMotor.set(0)
                self.rearRightMotor.set(0)

        def navxTest():
            if self.ahrs.getAngle() < 90.0:
                self.drive.tankDrive(0.6, -0.6)
            else:
                self.drive.tankDrive(0, 0)

        if gameData == "RRR":  # and position == 3:
            navxTest()
        else:
            navxTest()


    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.toggle = 0
        self.speed = 0.5

        self.ahrs.reset()

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        # smart dashboard
        self.sd.putNumber("Gyro angle: ", self.ahrs.getAngle())

        leftAxis = self.left_stick.getRawAxis(1)
        rightAxis = self.right_stick.getRawAxis(1)

        if self.joystick.getRawButton(3):
            self.right.set(0.5)
        else:
            self.right.set(0)

        rotateToAngle = False
        if self.joystick.getRawButton(7):
            self.ahrs.reset()

        if self.joystick.getRawButton(4):
            self.turnController.setSetpoint(0.0)
            rotateToAngle = True
        elif self.joystick.getRawButton(2):
            self.turnController.setSetpoint(90.0)
            rotateToAngle = True
        elif self.joystick.getRawButton(3):
            self.turnController.setSetpoint(180.0)
            rotateToAngle = True
        elif self.joystick.getRawButton(1):
            self.turnController.setSetpoint(-90.0)
            rotateToAngle = True

        if rotateToAngle:
            self.turnController.enable()
            currentRotationRate = self.rotateToAngleRate
        else:
            self.turnController.disable()
            currentRotationRate = self.joystick.getX()
        

        # pneumatics control
        if self.right_stick.getRawButton(1):      # open claw
            self.DoubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.left_stick.getRawButton(1):    # close claw
            self.DoubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self.joystick.getRawButton(9):  # turn on compressor
            self.Compressor.stop()
        elif self.joystick.getRawButton(10):  # turn off compressor
            self.Compressor.start()

        self.drive.tankDrive(-leftAxis/1.25, -rightAxis/1.25)
        self.drive.arcadeDrive(self.joystick.getY(), currentRotationRate)

    def pidWrite(self, output):
        self.rotateToAngleRate = output


if __name__ == '__main__':
    wpilib.run(MyRobot)