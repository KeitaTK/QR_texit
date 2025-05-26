#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import serial.tools.list_ports
import time

class Dynamixel:
    ver = '23.10.27'
    #
    ## Control table address
    # Addr, Size, Name, RW, Default, Range, Unit
    # 8	1	Baud Rate	RW	1	0 ~ 7	-
    ADDR_RETURN_DELAY_TIME  = 9 # 1 RW	250	0 ~ 254	2 [μsec]
    ADDR_DRIVE_MODE = 10    # 1 RW	0	0 ~ 5	-
    #     1: Reverse mode
    ADDR_OPERATING_MODE     = 11    # 1 RW	3	0 ~ 16	-
    #     0: Current Control Mode
    #     1: Velocity Control Mode
    #     3: Position Control Mode
    #     16: PWM Control Mode (Voltage Control Mode)
    # 36	2	PWM Limit	RW	885	0 ~ 885	0.113 [%]
    # 38	2	Current Limit	RW	1,193	0 ~ 1,193	2.69 [mA]
    # 44	4	Velocity Limit	RW	200	0 ~ 1,023	0.229 [rev/min]
    # 48	4	Max Position Limit	RW	4,095	0 ~ 4,095	1 [pulse]
    # 52	4	Min Position Limit	RW	0	0 ~ 4,095	1 [pulse]
    ADDR_TORQUE_ENABLE      = 64    # 1 RW	0	0 ~ 1	-
    ADDR_LED = 65   # 1 RW	0	0 ~ 1	-
    ADDR_STATUS_RETURN_LEVEL = 68   # 1 RW	2	0 ~ 2	-
    #     0: PING Instruction
    #     1: PING Instruction, READ Instruction
    #     2: All Instructions	
    ADDR_GOAL_PWM           = 100   # 0 RW	-	-PWM Limit(36) ~ PWM Limit(36)	-
    ADDR_GOAL_CURRENT       = 102   # 2 RW	-	-Current Limit(38) ~ Current Limit(38)	2.69 [mA]
    ADDR_GOAL_VELOCITY      = 104  # 4 RW	-	-Velocity Limit(44) ~ Velocity Limit(44)	0.229 [rev/min]
    ADDR_GOAL_POSITION      = 116   # 4 RW	-	Min Position Limit(52) ~ Max Position Limit(48)	1 [pulse]
    ADDR_PRESENT_PWM     = 124      # 2 R	-	-	-
    ADDR_PRESENT_CURRENT = 126      # 2 R	-	-	2.69 [mA]
    ADDR_PRESENT_VELOCITY = 128     # 4	R	-	-	0.229 [rev/min]
    ADDR_PRESENT_POSITION   = 132   # 4	R	-	-	1 [pulse]

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
    # COMM_SUCCESS                = 0;            % Communication Success result value
    # COMM_TX_FAIL                = -1001;        % Communication Tx Failed

    def __init__(self, device_name, baud=57600):
        self.DEVICENAME = device_name
        self.BAUDRATE = baud
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(Dynamixel.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            self.portHandler.closePort()
            quit()

        # Set port baudrate
        if  not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to change the baudrate")
            self.portHandler.closePort()
            quit()
        # print(len(self.DXL_IDs))

    def __del__(self):
        # Close port
        if hasattr(self, 'portHandler'):
            self.portHandler.closePort()

    def setRecommendedValue(self, DXL_IDs):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
        self.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        self.writeReturnDelayTime(DXL_IDs, [2] * len(DXL_IDs))
        self.writeStatusReturnLevel(DXL_IDs, [1] * len(DXL_IDs))
        # self.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

    def readReturnDelayTime(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_RETURN_DELAY_TIME, 1, DXL_IDs)
    def readDriveMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_DRIVE_MODE, 1, DXL_IDs)
    def readOperatingMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_OPERATING_MODE, 1, DXL_IDs)
    def readTorqueEnable(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_TORQUE_ENABLE, 1, DXL_IDs)
    def readLED(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_LED, 1, DXL_IDs)
    def readStatusReturnLevel(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_STATUS_RETURN_LEVEL, 1, DXL_IDs)

    def writeReturnDelayTime(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_RETURN_DELAY_TIME, 1, DXL_IDs, data)
    def writeDriveMode(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_DRIVE_MODE, 1, DXL_IDs, data)
    def writeOperatingMode(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_OPERATING_MODE, 1, DXL_IDs, data)
    def writeTorqueEnable(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_TORQUE_ENABLE, 1, DXL_IDs, data)
    def writeLED(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_LED, 1, DXL_IDs, data)
    def writeStatusReturnLevel(self, DXL_IDs, data):
        self.groupAsyncWrite(Dynamixel.ADDR_STATUS_RETURN_LEVEL, 1, DXL_IDs, data)

    def readPresentPWM(self, DXL_IDs, data):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_PWM, 2, DXL_IDs)
    def readPresentVelocity(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_VELOCITY, 4, DXL_IDs)
    def readPresentPosition(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_POSITION, 4, DXL_IDs)
    def readPresentCurrent(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_CURRENT, 2, DXL_IDs)

    def writeGoalPWM(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_PWM, 2, DXL_IDs, data)
    def writeGoalCurrent(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_CURRENT, 2, DXL_IDs, data)
    def writeGoalVelocity(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_VELOCITY, 4, DXL_IDs, data)
    def writeGoalPosition(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_POSITION, 4, DXL_IDs, data)

    def groupAsyncWrite(self, ADDR, byte, DXL_IDs, data):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
            data = [data]
        for i, DXL_ID in enumerate(DXL_IDs):
            if byte == 1:
                dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            elif byte == 2:
                dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            elif byte == 4:
                dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, DXL_ID, ADDR, int(data[i]))
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)
    def groupAsyncRead(self, ADDR, byte, DXL_IDs):
        data = [0]*len(DXL_IDs)
        for i, DXL_ID in enumerate(DXL_IDs):
            if byte == 1:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, ADDR)
            elif byte == 2:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR)
            elif byte == 4:
                data[i], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
        return data
    def groupSyncWrite(self, ADDR, byte, DXL_IDs, data):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
            data = [data]
        groupSyncWritePacket = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR, byte)
        # self.groupSyncWriteGoalPosition.clearParam()
        for DXL_ID in DXL_IDs:
            iData = int(data[DXL_IDs.index(DXL_ID)])
            if byte == 4:
                param = [ iData&0xFF, (iData>>8)&0xFF, (iData>>16)&0xFF, (iData>>24)&0xFF ]
            elif byte == 2:
                param = [ iData&0xFF, (iData>>8)&0xFF ]
            else:
                param = iData&0xFF
            # dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, self.signed_hex2int(param, 8))
            dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, param)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
        dxl_comm_result = groupSyncWritePacket.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    
    def groupSyncRead(self, ADDR, byte, DXL_IDs):
        if not hasattr(DXL_IDs, "__iter__"):
            DXL_IDs = [DXL_IDs]
        data = [0]*len(DXL_IDs)
        groupSyncReadPacket = GroupSyncRead(self.portHandler, self.packetHandler,ADDR, byte)
        # groupSyncReadPacket.clearParam()
        for DXL_ID in DXL_IDs:
            dxl_addparam_result = groupSyncReadPacket.addParam(DXL_ID)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID)
        dxl_comm_result = groupSyncReadPacket.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for num,DXL_ID in enumerate(DXL_IDs):
            dxl_getdata_result = groupSyncReadPacket.isAvailable(DXL_ID, ADDR, byte)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
            data[num] = groupSyncReadPacket.getData(DXL_ID, ADDR, byte)
            if byte == 4:
                data[num] = self.signed_hex2int(data[num], 32)
                # data[num] = float(data[num]+(data[num]>>31)*(1-0xFFFFFFFF))
            elif byte == 2:
                data[num] = self.signed_hex2int(data[num], 16)
            else:
                data[num] = data[num]&0xFF
                # data[num] = self.signed_hex2int(data[num], 8)
        return data

    @classmethod
    def open_ports_by_serial_number(cls, serial_number, baud=57600):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return cls(port.device, baud)
    @staticmethod
    def device_name_from_serial_number(serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
    @staticmethod
    def show_list_ports():
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is None:
                port.serial_number = "None"
                port.vid = 0
                port.pid = 0
            print('Name:%s, Serial Number:%s, VID:PID:%04X:%04X, Manufacturer:%s'%(
                port.device,
                port.serial_number,
                port.vid,
                port.pid,
                port.manufacturer) )
    #         print("-----------")
    #         print(port.device)
    #         print(port.name)
    #         print(port.description)
    #         print(port.hwid)
    #         print(port.vid)
    #         print(port.pid)
    #         print(port.serial_number)
    #         print(port.location)
    #         print(port.manufacturer)
    #         print(port.product)
    #         print(port.interface)
    @staticmethod
    def signed_hex2int( signed_hex, digit ):
        signed = 0x01 << (digit-1)
        mask = 0x00
        for num in range(digit):
            mask = mask | (0x01<<num)
        signed_int = (int(signed_hex^mask)*-1)-1  if (signed_hex & signed) else int(signed_hex)
        return signed_int

def timeMesurement( func, num, arg1=None, arg2=None, arg3=None ):
    start = time.time()
    if num==0:
        ret = func()
    elif num==1:
        ret = func(arg1)
    elif num==2:
        ret = func(arg1, arg2)
    else:
        ret = func(arg1, arg2, arg3)        
    elapsed_time = time.time() - start
    # print("Spent time = %.3f(msec)" % (elapsed_time*1000))
    return ret, elapsed_time

def testReadWritePosition(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])
    dyn.writeOperatingMode(DXL_IDs, [3, 3])
    dyn.writeTorqueEnable(DXL_IDs, [1, 1])

    start = time.time()
    dyn.writeGoalPosition(DXL_IDs, [90/0.088,90/0.088])
    elapsed_time = time.time() - start
    print ("elapsed_time(write):{0}".format(elapsed_time*1000) + "[msec]")
    time.sleep(1.0)
    start = time.time()
    pos = dyn.readPresentPosition(DXL_IDs)
    elapsed_time = time.time() - start
    print(pos)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeGoalPosition(DXL_IDs, [0,0])
    time.sleep(1.0)
    start = time.time()
    pos = dyn.readPresentPosition(DXL_IDs)
    elapsed_time = time.time() - start
    print(pos)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])
    # ret, elapsed_time = timeMesurement(dyn.writeGoalPosition, 2, DXL_IDs, [0,0])
    # print("Spent time = %.3f(msec)" % (elapsed_time*1000))

def testReadWriteVelocity(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0] * 2)
    dyn.writeOperatingMode(DXL_IDs, [1] * 2)
    dyn.writeTorqueEnable(DXL_IDs, [1] * 2)

    start = time.time()
    dyn.writeGoalVelocity(DXL_IDs, [200] * 2)
    elapsed_time = time.time() - start
    print ("elapsed_time(write):{0}".format(elapsed_time*1000) + "[msec]")
    time.sleep(1.0)
    start = time.time()
    vel = dyn.readPresentVelocity(DXL_IDs)
    elapsed_time = time.time() - start
    print(vel)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeGoalVelocity(DXL_IDs, [0,0])
    time.sleep(1.0)
    start = time.time()
    vel = dyn.readPresentVelocity(DXL_IDs)
    elapsed_time = time.time() - start
    print(vel)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])

def testReadWriteCurrent(dyn, DXL_IDs):
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])
    dyn.writeOperatingMode(DXL_IDs, [0] * 2)
    dyn.writeTorqueEnable(DXL_IDs, [1, 1])

    start = time.time()
    dyn.writeGoalCurrent(DXL_IDs, [-100] * 2)
    elapsed_time = time.time() - start
    print ("elapsed_time:{0}".format(elapsed_time*1000) + "[msec]")
    time.sleep(1.0)
    start = time.time()
    vel = dyn.readPresentCurrent(DXL_IDs)
    elapsed_time = time.time() - start
    print(vel)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeGoalCurrent(DXL_IDs, [100] * 2)
    time.sleep(1.0)
    start = time.time()
    vel = dyn.readPresentCurrent(DXL_IDs)
    elapsed_time = time.time() - start
    print(vel)
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")
    dyn.writeGoalCurrent(DXL_IDs, [0] * 2)
    time.sleep(0.5)
    dyn.writeTorqueEnable(DXL_IDs, [0, 0])

def testReadSettings(dyn, DXL_IDs):
    start = time.time()
    print(dyn.readReturnDelayTime(DXL_IDs))
    print(dyn.readDriveMode(DXL_IDs))
    print(dyn.readOperatingMode(DXL_IDs))
    print(dyn.readTorqueEnable(DXL_IDs))
    print(dyn.readLED(DXL_IDs))
    print(dyn.readStatusReturnLevel(DXL_IDs))
    elapsed_time = time.time() - start
    print ("elapsed_time(read):{0}".format(elapsed_time*1000) + "[msec]")

if __name__ == '__main__':
    # DXL_IDs                      = 1
    DXL_IDs                      = [1, 2]
    BAUDRATE                    = 1e6             # Dynamixel default baudrate : 57600
    # Check which port is being used on your controller
    # DEVICENAME                  = 'COM5' # for Win
    # DEVICENAME                  = '/dev/tty.usbserial-FT6Z5XDE' # for macOS
    # dyn = Dynamixel(DEVICENAME, BAUDRATE)
    Dynamixel.show_list_ports()
    dyn = Dynamixel.open_ports_by_serial_number("FT6Z5XDEA", BAUDRATE)
    dyn.setRecommendedValue(DXL_IDs)

    testReadWritePosition(dyn, DXL_IDs)
    testReadWriteVelocity(dyn, DXL_IDs)
    testReadWriteCurrent(dyn, DXL_IDs)
    testReadSettings(dyn, DXL_IDs)
