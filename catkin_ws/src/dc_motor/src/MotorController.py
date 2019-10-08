import pigpio
import time
import struct
from math import sin,cos,radians
# Import for hardware I2C (unused)
'''
import smbus
import io
import fcntl
'''

# Register addresses for functions
DC_Firmware = 0x26
Set_EXP_ID = 0x24
Battery_Voltage = 0x53
WDT_STOP = 0x23
Controller_Enable = 0x25
Controller_Reset = 0x27
Motor1_Power = 0x40
Motor2_Power = 0x41
Motor_Powers = 0x42
Motor1_Speed = 0x43
Motor2_Speed = 0x44
Motor_Speeds = 0x45
Motor1_Target = 0x46
Motor2_Target = 0x47
Motor_Targets = 0x48
Motor1_Degree = 0x58
Motor2_Degree = 0x59
Motor_Degrees = 0x5A
Motor1_Invert = 0x51
Motor2_Invert = 0x52
Motor1_Busy = 0x4F
Motor2_Busy = 0x50
Motor1_Current = 0x54
Motor2_Current = 0x55
Encoder1_Count = 0x49
Encoder2_Count = 0x4A
Encoder1_Degrees = 0x5B
Encoder2_Degrees = 0x5C
Reset_Encoder1 = 0x4C
Reset_Encoder2 = 0x4D
Reset_Encoders = 0x4E
Speed_PID = 0x56
Target_PID = 0x57

# Software I2C functions
class PigpioI2CBitBang(object):
    """
    PIGPIO Bitbang I2C Helper Class
    Provides device specific I2C interface
    instructions to PIGPIO daemon.
    """
    # GPIO configuration and frequency constants
    SDA = 2
    SCL = 3
    CF = 100000
    # PIGPIO bitbang constants
    BB_END = 0
    BB_ESCAPE = 1
    BB_START = 2
    BB_STOP = 3
    BB_ADDRESS = 4
    BB_FLAGS = 5
    BB_READ = 6
    BB_WRITE = 7

    def __init__(self):
        """
        Initalize the pigpio library.
        """
        self.pi = pigpio.pi()

    def stop(self):
        """
        Stop pigpio connection
        """
        self.pi.stop()

    def open_bus(self):
        """
        Creates a bitbang handle with a given SDA/SCL GPIO
        pair, and a I2C freqency set by CF.
        """
        self.handle = self.pi.bb_i2c_open(self.SDA, self.SCL, self.CF)

    def close_bus(self):
        """
        Closes an open bitbang handle.
        """
        self.pi.bb_i2c_close(self.SDA)

    def read(self, address, pointer_reg, read_bytes):
        """
        Performs a standard I2C read operation on a
        particular address. Requires the 7-bit I2C address,
        the pointer register to write to, and the number
        of bytes to read from the pointer register.
        """
        #read_bytes += 1
        arg_list = [self.BB_ADDRESS, address, self.BB_START,
                    self.BB_WRITE, pointer_reg,
                    self.BB_START, self.BB_READ, read_bytes,
                    self.BB_STOP, self.BB_END]
        count, data = self.pi.bb_i2c_zip(self.SDA, arg_list)
        return [count,data]

    def justread(self, address, read_bytes):
        """
        Performs a standard I2C read operation on a
        particular address. Requires the 7-bit I2C address, the number
        of bytes to read from the pointer register.
        """
        #read_bytes += 1
        arg_list = [self.BB_ADDRESS, address,
                    self.BB_START, self.BB_READ, read_bytes,
                    self.BB_STOP, self.BB_END]
        count, data = self.pi.bb_i2c_zip(self.SDA, arg_list)
        return [count,data]

    def write(self, address, pointer_reg, data_array):
        """
        Performs a standard I2C write opreation on a particular
        address. Requires the 7-bit I2C address, the pointer
        register, and a data array of the data package to send.
        """
        if type(data_array) is int:
            assert data_array < 256
            data_array = [data_array]
        arg_list = [self.BB_ADDRESS, address, self.BB_START,
                    self.BB_WRITE, len(data_array)+1, pointer_reg
                    ] + data_array + [self.BB_STOP, self.BB_END]
        count, data = self.pi.bb_i2c_zip(self.SDA, arg_list)
        return [count,data]

# Hardware I2C functions, unused due to stability issues
'''
class i2c:
    def __init__(self, device, bus):
        # Open I2C device for r/w
        self.fr = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
        self.fw = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)
        # Set device address
        fcntl.ioctl(self.fr, I2C_SLAVE, device)
        fcntl.ioctl(self.fw, I2C_SLAVE, device)

    def write(self, bytes):
        self.fw.write(bytes)

    def read(self, bytes):
        return self.fr.read(bytes)

    def close(self):
        self.fw.close()
        self.fr.close()
'''

class MotorController:
    def __init__(self,address):
        self.address = address
        self.bus = PigpioI2CBitBang()

    def openI2Cbus(self):
        try:
            self.bus.close_bus()
        except:
            pass
        self.bus.open_bus()
    
    def controllerEnable(self):
        self.bus.write(self.address,Controller_Enable,[])
        time.sleep(0.05)
        
    def controllerReset(self):
        self.bus.write(self.address,Controller_Reset,[])
        time.sleep(0.05)
        
    def WDT_STOP(self):
        self.bus.write(self.address,WDT_STOP,[])
        time.sleep(0.05)
    
    def setMotorPowers(self,power1,power2):
        self.bus.write(self.address,Motor_Powers,[power1,power2])
        time.sleep(0.01)
    
    def setMotorPower(self,channel,power):
        if channel==1:
            channel = Motor1_Power
        if channel==2:
            channel = Motor2_Power    
        self.bus.write(self.address,channel,[power])
        time.sleep(0.01)
        
    def setMotorSpeed(self,channel,speed):
        if channel==1:
            channel = Motor1_Speed
        if channel==2:
            channel = Motor2_Speed
        bytes = struct.pack(">h", speed)
        hibyte = struct.unpack('>h', b'\x00' + bytes[0])[0]
        lobyte = struct.unpack('>h', b'\x00' + bytes[1])[0]
        self.bus.write(self.address,channel,[hibyte,lobyte])
        time.sleep(0.01)
        
    def setMotorSpeeds(self,speed1,speed2):
        bytes1 = struct.pack(">h", speed1)
        hibyte1 = struct.unpack('>h', b'\x00' + bytes1[0])[0]
        lobyte1 = struct.unpack('>h', b'\x00' + bytes1[1])[0]
        bytes2 = struct.pack(">h", speed2)
        hibyte2 = struct.unpack('>h', b'\x00' + bytes2[0])[0]
        lobyte2 = struct.unpack('>h', b'\x00' + bytes2[1])[0]
        self.bus.write(self.address,Motor_Speeds,[hibyte1, lobyte1, hibyte2, lobyte2] )
        #time.sleep(0.005)
        
    def setMotorTarget(self,channel,speed,target):
        if channel==1:
            channel = Motor1_Target
        if channel==2:
            channel = Motor2_Target
        sbytes = struct.pack(">h", speed)
        shibyte = struct.unpack('>h', b'\x00' + sbytes[0])[0]
        slobyte = struct.unpack('>h', b'\x00' + sbytes[1])[0]
        tbytes = struct.pack(">l", target)
        tbyte0 = struct.unpack('>h', b'\x00' + tbytes[0])[0]
        tbyte1 = struct.unpack('>h', b'\x00' + tbytes[1])[0]
        tbyte2 = struct.unpack('>h', b'\x00' + tbytes[2])[0]
        tbyte3 = struct.unpack('>h', b'\x00' + tbytes[3])[0]
        self.bus.write(self.address,channel,[shibyte,slobyte,tbyte0,tbyte1,tbyte2,tbyte3])
        time.sleep(0.01)
        
    def setMotorTargets(self,speed1,target1,speed2,target2):
        sbytes1 = struct.pack(">h", speed1)
        shibyte1 = struct.unpack('>h', b'\x00' + sbytes1[0])[0]
        slobyte1 = struct.unpack('>h', b'\x00' + sbytes1[1])[0]
        tbytes1 = struct.pack(">l", target1)
        tbyte01 = struct.unpack('>h', b'\x00' + tbytes1[0])[0]
        tbyte11 = struct.unpack('>h', b'\x00' + tbytes1[1])[0]
        tbyte21 = struct.unpack('>h', b'\x00' + tbytes1[2])[0]
        tbyte31 = struct.unpack('>h', b'\x00' + tbytes1[3])[0]
        sbytes2 = struct.pack(">h", speed2)
        shibyte2 = struct.unpack('>h', b'\x00' + sbytes2[0])[0]
        slobyte2 = struct.unpack('>h', b'\x00' + sbytes2[1])[0]
        tbytes2 = struct.pack(">l", target2)
        tbyte02 = struct.unpack('>h', b'\x00' + tbytes2[0])[0]
        tbyte12 = struct.unpack('>h', b'\x00' + tbytes2[1])[0]
        tbyte22 = struct.unpack('>h', b'\x00' + tbytes2[2])[0]
        tbyte32 = struct.unpack('>h', b'\x00' + tbytes2[3])[0]
        self.bus.write(self.address,Motor_Targets,[shibyte1,slobyte1,tbyte01,tbyte11,tbyte21,tbyte31,shibyte2,slobyte2,tbyte02,tbyte12,tbyte22,tbyte32])
        time.sleep(0.01)
        
    def setMotorDegree(self,channel,speed,target):
        if channel==1:
            channel = Motor1_Degree
        if channel==2:
            channel = Motor2_Degree
        sbytes = struct.pack(">h", speed)
        shibyte = struct.unpack('>h', b'\x00' + sbytes[0])[0]
        slobyte = struct.unpack('>h', b'\x00' + sbytes[1])[0]
        tbytes = struct.pack(">l", target)
        tbyte0 = struct.unpack('>h', b'\x00' + tbytes[0])[0]
        tbyte1 = struct.unpack('>h', b'\x00' + tbytes[1])[0]
        tbyte2 = struct.unpack('>h', b'\x00' + tbytes[2])[0]
        tbyte3 = struct.unpack('>h', b'\x00' + tbytes[3])[0]
        self.bus.write(self.address,channel,[shibyte,slobyte,tbyte0,tbyte1,tbyte2,tbyte3])
        time.sleep(0.01)
        
    def setMotorDegrees(self,speed1,target1,speed2,target2):
        sbytes1 = struct.pack(">h", speed1)
        shibyte1 = struct.unpack('>h', b'\x00' + sbytes1[0])[0]
        slobyte1 = struct.unpack('>h', b'\x00' + sbytes1[1])[0]
        tbytes1 = struct.pack(">l", target1)
        tbyte01 = struct.unpack('>h', b'\x00' + tbytes1[0])[0]
        tbyte11 = struct.unpack('>h', b'\x00' + tbytes1[1])[0]
        tbyte21 = struct.unpack('>h', b'\x00' + tbytes1[2])[0]
        tbyte31 = struct.unpack('>h', b'\x00' + tbytes1[3])[0]
        sbytes2 = struct.pack(">h", speed2)
        shibyte2 = struct.unpack('>h', b'\x00' + sbytes2[0])[0]
        slobyte2 = struct.unpack('>h', b'\x00' + sbytes2[1])[0]
        tbytes2 = struct.pack(">l", target2)
        tbyte02 = struct.unpack('>h', b'\x00' + tbytes2[0])[0]
        tbyte12 = struct.unpack('>h', b'\x00' + tbytes2[1])[0]
        tbyte22 = struct.unpack('>h', b'\x00' + tbytes2[2])[0]
        tbyte32 = struct.unpack('>h', b'\x00' + tbytes2[3])[0]
        self.bus.write(self.address,Motor_Degrees,[shibyte1,slobyte1,tbyte01,tbyte11,tbyte21,tbyte31,shibyte2,slobyte2,tbyte02,tbyte12,tbyte22,tbyte32])
        time.sleep(0.01)
        
    def readBatteryVoltage(self):
        self.bus.write(self.address,Battery_Voltage,[])
        [count,data] = self.bus.justread(self.address, 2)
        Bvoltagetemp = struct.unpack('<bB',data)
        Bvoltage = (Bvoltagetemp[0] *256 +Bvoltagetemp[1])/100.0
        time.sleep(0.01)
        return Bvoltage

    def readDCFirmware(self):
        self.bus.write(self.address,DC_Firmware,[])
        [count,data] = self.bus.justread(self.address, 1)
        DCFirmware = struct.unpack('<b', data)
        time.sleep(0.01)
        return DCFirmware[0]

    def readMotor1Current(self):
        self.bus.write(self.address,Motor1_Current,[])
        [count, data] = self.bus.justread(self.address, 2)
        temp = struct.unpack('<bB', data)
        Motor1Current = (temp[0] *256 +temp[1])/1000.0
        #time.sleep(0.01)
        return Motor1Current
    
    def readMotor2Current(self):
        self.bus.write(self.address,Motor2_Current,[])
        [count, data] = self.bus.justread(self.address, 2)
        temp = struct.unpack('<bB', data)
        Motor1Current = (temp[0] *256 +temp[1])/1000.0
        #time.sleep(0.01)
        return Motor2Current
        
    def readEncoderCount(self,channel):
        if channel==1:
            channel = Encoder1_Count
        if channel==2:
            channel = Encoder2_Count
        self.bus.write(self.address,channel,[])
        [count,result] = self.bus.justread(self.address,4)
        temp = struct.unpack('<bBBB', result)
        EncoderCount = temp[0]
        EncoderCount = EncoderCount*256 + temp[1]
        EncoderCount = EncoderCount*256 + temp[2]
        EncoderCount = EncoderCount*256 + temp[3]
        return EncoderCount
    
    def readEncoderDegrees(self,channel):
        if channel==1:
            channel = Encoder1_Degrees
        if channel==2:
            channel = Encoder2_Degrees
        self.bus.write(self.address,channel,[])
        [count, result] = self.bus.justread(self.address, 4)
        temp = struct.unpack('<bBBB', result)
        EncoderDegree = temp[0]
        EncoderDegree = EncoderDegree*256 + temp[1]
        EncoderDegree = EncoderDegree*256 + temp[2]
        EncoderDegree = EncoderDegree*256 + temp[3]
        time.sleep(0.01)
        return EncoderDegree
        
    def readEncodersCount(self):
        Encoder1Count = self.readEncoderCount(1)
        Encoder2Count = self.readEncoderCount(2)
        return [Encoder1Count,Encoder2Count]
    
    def readEncodersDegrees(self):
        Encoder1Deg = self.readEncoderDegrees(1)
        Encoder2Deg = self.readEncoderDegrees(2)
        return [Encoder1Deg,Encoder2Deg]
    
    def readMotor1Busy(self):
        self.bus.write(self.address,Motor1_Busy,[])
        [count,data] = self.bus.justread(self.address, 1)
        Motor1Busy = struct.unpack('<b', data)
        return Motor1Busy[0]
    
    def readMotor2Busy(self):
        self.bus.write(self.address,Motor2_Busy,[])
        [count,data] = self.bus.justread(self.address, 1)
        Motor2Busy = struct.unpack('<b', data)
        return Motor2Busy[0]
    
    def setMotorInvert(self,channel,invert):
        if channel==1:
            channel = Motor1_Invert
        if channel==2:
            channel = Motor2_Invert
        self.bus.write(self.address,channel,[invert])
        time.sleep(0.01)
        
    def resetEncoders(self):
        self.bus.write(self.address,Reset_Encoders,[])
        time.sleep(0.01)
    
    def resetEncoder(self,channel):
        if channel==1:
            channel = Reset_Encoder1
        if channel==2:
            channel = Reset_Encoder2
        self.bus.write(self.address,channel)
        time.sleep(0.01)
        
    def readMotorsBusy(self):
        Motor1Busy = self.readMotor1Busy()
        time.sleep(0.01)
        Motor2Busy = self.readMotor2Busy()
        time.sleep(0.01)
        if  Motor1Busy or Motor2Busy:
            return True
        else:
            return False
        
    def stopMotors(self):
        self.setMotorPowers(125,125)
    
    def waitTarget(self):
        while self.readMotorsBusy():
            time.sleep(0.02)
        self.stopMotors()
        
def VxVyWtoMotorSpeeds(vx,vy,w,theta):
    rradius = 0.17324 # Robot radius
    mstods = 1127.87 # m/s to deg/s
    thetarad = radians(theta)
    # Kinematic equations
    v = vx * cos(thetarad) + vy * sin(thetarad)
    vn = -vx * sin(thetarad) + vy *  cos(thetarad)
    # Wheel speeds in m/s
    v0ms = vn + rradius * w
    v1ms = -v + rradius * w
    v2ms = -vn + rradius * w
    v3ms = v + rradius * w
    # Wheel speeds in deg/s (converted to int)
    v0ds = int(v0ms * mstods)
    v1ds = int(v1ms * mstods)
    v2ds = int(v2ms * mstods)
    v3ds = int(v3ms * mstods)
    return [v0ds,v1ds,v2ds,v3ds]

