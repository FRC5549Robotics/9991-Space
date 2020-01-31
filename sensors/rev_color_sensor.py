import wpilib
import hal
from hal_impl import i2c_helpers, data
import typing

hal_data = data.hal_data


class ColorSensorBase(wpilib.SensorBase):

    def __init__(self):
        super().__init__()
        self.redChannel = 0
        self.blueChannel = 0
        self.greenChannel = 0

    def getColor(self, *args, **kwargs) -> int:
        raise NotImplementedError(
            "Implement 'getColor' in your class!"
        )

    def getRed(self, *args, **kwargs) -> int:
        raise NotImplementedError(
            "Implement 'getRed' in your class!"
        )

    def getBlue(self, *args, **kwargs) -> int:
        raise NotImplementedError(
            "Implement 'getBlue' in your class!"
        )

    def getGreen(self, *args, **kwargs) -> int:
        raise NotImplementedError(
            "Implement 'getGreen' in your class!"
        )

    def initSendable(self, builder: wpilib.SendableBuilder) -> None:
        builder.setSmartDashboardType("ColorSensor")
        builder.addDoubleProperty("Red", self.getRed, None)
        builder.addDoubleProperty("Blue", self.getBlue, None)
        builder.addDoubleProperty("Green", self.getGreen, None)

class REV_Color_Sensor_V3(ColorSensorBase):
    """Considering ramifications."""

    ADDRESS = 0x52

    def __init__(self, port: wpilib.I2C.Port):
        super().__init__()

        if port is None:
            port = wpilib.I2C.Port.kOnboard

        simPort = None
        if hal.HALIsSimulation():
            simPort = REV_Color_V3_Sim(self)

        self.i2c = wpilib.I2C(port, self.ADDRESS, simPort=simPort)
        self.i2c.write(0x04, 0x22)
        self.i2c.write(0x05, 0x01)
        self.setName("REV_Robotics_Color_Sensor_V3", port)

    def enable(self) -> None:
        self.i2c.write(0x00, 0x06)

    def getColor(self) -> typing.List[float]:
        red = self.getRed()
        green = self.getGreen()
        blue = self.getBlue()
        mag = red + green + blue
        color = [red / mag, green / mag, blue / mag]
        return color

    def getRed(self):
        redRegData = self.readRegister(0x13, 3)# << 8\
        #           + self.readRegister(0x14, 1) << 8\
        #           + self.readRegister(0x15, 1)
        return redRegData

    def getGreen(self):
        greenRegData = self.readRegister(0x0D, 3)# << 8\
        #             + self.readRegister(0x0E, 1) << 8\
        #             + self.readRegister(0x0F, 1)
        return greenRegData

    def getBlue(self):
        blueRegData = self.readRegister(0x10, 3)# << 8\
        #            + self.readRegister(0x11, 1) << 8\
        #            + self.readRegister(0x12, 1)
        return blueRegData

    def readRegister(self, register: int, bytes: int) -> int:
        return int.from_bytes(self.i2c.read(register, bytes), byteorder="big")


class REV_Color_V3_Sim(i2c_helpers.I2CSimBase):

    def __init__(self, sensor: REV_Color_Sensor_V3):
        super().__init__()
        self.sensor = sensor
        self.red = 0
        self.green = 0
        self.blue = 0
        self.clear = 0

    def initializeI2C(self, port, status):
        self.color_key = "rev_color_sensor_v2_%d_color" % port

    def transactionI2C(
            self, port, deviceAddress, dataToSend, sendSize, dataReceived, receiveSize
    ):
        deviceAddress = REV_Color_Sensor_V3.ADDRESS
        sendSize = 0xFF
        receiveSize = 0xFF
        dataReceived[0] = 0xFF
        port = wpilib.I2C.Port.kOnboard

        return 0

    def readI2C(self, port, deviceAddress, buffer, count):
        color = hal_data["robot"].get(self.color_key, 0)
        port = wpilib.I2C.Port.kOnboard
        if count is 2:
            buffer[1] = (0xFF).to_bytes(1, "big")
            buffer[0] = (0xFF).to_bytes(1, "big")
        elif count is 1:
            buffer[0] = (0xFF).to_bytes(1, "big")

        return count