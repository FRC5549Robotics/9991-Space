import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
import logging
from wpilib import DriverStation
from math import *

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """Robot initialization function"""

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
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)

        # Button box
        self.buttonBox = wpilib.Joystick(3)
        self.buttonStatus = False

        # pneumatics init
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoid = wpilib.DoubleSolenoid(0, 1)
        self.Compressor.start()

        self.sensor = wpilib.AnalogInput(0)
        self.sensor.getVoltage()

        '''Smart Dashboard'''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.99.91.2')

        self.ds = DriverStation.getInstance()
        self.sd.putString("TX2State: ", "Enable")

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.Compressor.stop()

        self.buttonStatus = False

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

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

        def toggleTest():
            if self.timer.get() <= 3:
                self.drive.tankDrive(0.5, 0.5)
            elif self.timer.get() > 3:
                self.buttonStatus = False
                self.timer.reset()

        if self.buttonBox.getRawButtonPressed(7):
            self.buttonStatus = not self.buttonStatus

        if self.buttonStatus is True:
            toggleTest()

        self.sd.putBoolean("Button Status: ", self.buttonStatus)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.driveAxis = self.rightStick.getRawAxis(1)
        self.rotateAxis = self.rightStick.getRawAxis(2)

        # # drives drive system using tank steering
        # if self.DoubleSolenoidOne.get() == 1:  # if on high gear
        #     self.divisor = 1.2  # 90% of high speed
        # elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
        #     self.divisor = 1.2  # normal slow speed
        # else:
        #     self.divisor = 1.0

        if self.driveAxis != 0:
            self.leftSign = self.driveAxis / fabs(self.driveAxis)
        else:
            self.leftSign = 0

        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        self.drive.arcadeDrive(-self.driveAxis / 1.25, self.rotateAxis / 1.25)


if __name__ == '__main__':
    wpilib.run(MyRobot)