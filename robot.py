import wpilib
from networktables import NetworkTables
from ctre import *
from robotpy_ext.control.toggle import Toggle
from math import *


class MyRobot(wpilib.TimedRobot):
    ''' robot program starts here '''

    def robotInit(self):
        ''' function that is run at the beginning of the match '''
        self.bottomShooterEncoder = WPI_TalonSRX(7)
        self.topShooter = WPI_TalonSRX(6)
        self.bottomShooter = WPI_TalonSRX(4)
        self.topShooterEncoder = WPI_TalonSRX(5)

        self.xbox = wpilib.Joystick(2) # controller for shooter

        self.topShooters = wpilib.SpeedControllerGroup(self.topShooter, self.topShooterEncoder)
        self.bottomShooters = wpilib.SpeedControllerGroup(self.bottomShooter, self.bottomShooterEncoder)

        self.dash = NetworkTables.getTable("limelight")
        self.dashboard = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.99.91.2')

        self.topButtonStatus = Toggle(self.xbox, 1)
        self.bottomButtonStatus = Toggle(self.xbox, 4)

        # PID settings
        kP = 0.0
        kI = 0.0
        kD = 0.0

    def autonomousInit(self):
        ''' function that is run at the beginning of the autonomous phase '''
        pass

    def autonomousPeriodic(self):
        ''' function that is run periodically during the autonomous phase '''
        pass

    def teleopInit(self):
        ''' function that is run at the beginning of the tele-operated phase '''
        pass

    def teleopPeriodic(self):
        ''' function that is run periodically during the tele-operated phase '''
        if self.topButtonStatus.on:
            self.topShooters.set(0.1)
        elif self.topButtonStatus.off:
            self.topShooters.set(-0.1)
        elif self.xbox.getRawButton(3):
            self.topShooters.set(0)

        if self.bottomButtonStatus.on:
            self.bottomShooters.set(0.1)
        elif self.bottomButtonStatus.off:
            self.bottomShooters.set(-0.1)
        elif self.xbox.getRawButton(2):
            self.bottomShooters.set(0)

        self.topValue = fabs((self.topShooterEncoder.getSelectedSensorVelocity()) * (600 / 4096))
        self.bottomValue = fabs((self.bottomShooterEncoder.getSelectedSensorVelocity()) * (600 / 4096))

        self.dashboard.putNumber("Top Encoder", self.topValue)
        self.dashboard.putNumber("Bottom Encoder", self.bottomValue)

        self.angle = self.dash.getNumber('ty', 0)   # should get the angle from the limelight

        self.angleTwo = self.angle + 0.00000001  # plus 45 because that is what it will be mounted at I believe
        self.distance = 12.5 / (tan(radians(self.angleTwo)))  # (tan(radians(self.angleTwo)))  # the math to calculate distance in python, also the 96.19 is from the real target height and about where the limelight is mounted on the CAD file.

        self.dashboard.putNumber("Dist:", self.distance)  # writes to dashboard what the distance is


if __name__ == '__main__':
    ''' running the entire robot program '''
    wpilib.run(MyRobot)