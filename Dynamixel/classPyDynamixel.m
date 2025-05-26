classdef classPyDynamixel < handle
    properties (Hidden)
    end
    
    properties (SetAccess = private)
        pyObj;
    end
    
    properties (Constant)
        ver = '23.10.27';
%         # Addr, Size, Name, RW, Default, Range, Unit
%         # 8	1	Baud Rate	RW	1	0 ~ 7	-
        ADDR_RETURN_DELAY_TIME  = uint8(9); %# 1 RW	250	0 ~ 254	2 [μsec]
        ADDR_DRIVE_MODE = uint8(10);    %# 1 RW	0	0 ~ 5	-
%         #     1: Reverse mode
        ADDR_OPERATING_MODE     = uint8(11);    %# 1 RW	3	0 ~ 16	-
%         #     0: Current Control Mode
%         #     1: Velocity Control Mode
%         #     3: Position Control Mode
%         #     16: PWM Control Mode (Voltage Control Mode)
%         # 36	2	PWM Limit	RW	885	0 ~ 885	0.113 [%]
%         # 38	2	Current Limit	RW	1,193	0 ~ 1,193	2.69 [mA]
%         # 44	4	Velocity Limit	RW	200	0 ~ 1,023	0.229 [rev/min]
%         # 48	4	Max Position Limit	RW	4,095	0 ~ 4,095	1 [pulse]
%         # 52	4	Min Position Limit	RW	0	0 ~ 4,095	1 [pulse]
        ADDR_TORQUE_ENABLE      = uint8(64);    %# 1 RW	0	0 ~ 1	-
        ADDR_LED = uint8(65)   %# 1 RW	0	0 ~ 1	-
        ADDR_STATUS_RETURN_LEVEL = uint8(68);   %# 1 RW	2	0 ~ 2	-
%         #     0: PING Instruction
%         #     1: PING Instruction, READ Instruction
%         #     2: All Instructions	
        ADDR_GOAL_PWM           = uint8(100);   %# 0 RW	-	-PWM Limit(36) ~ PWM Limit(36)	-
        ADDR_GOAL_CURRENT       = uint8(102);   %# 2 RW	-	-Current Limit(38) ~ Current Limit(38)	2.69 [mA]
        ADDR_GOAL_VELOCITY      = uint8(104);  %# 4 RW	-	-Velocity Limit(44) ~ Velocity Limit(44)	0.229 [rev/min]
        ADDR_GOAL_POSITION      = uint8(116);   %# 4 RW	-	Min Position Limit(52) ~ Max Position Limit(48)	1 [pulse]
        ADDR_PRESENT_PWM     = uint8(124);      %# 2 R	-	-	-
        ADDR_PRESENT_CURRENT = uint8(126);      %# 2 R	-	-	2.69 [mA]
        ADDR_PRESENT_VELOCITY = uint8(128);     %# 4	R	-	-	0.229 [rev/min]
        ADDR_PRESENT_POSITION   = uint8(132);   %# 4	R	-	-	1 [pulse]
    
%         # Protocol version
        PROTOCOL_VERSION            = 2.0;               %# See which protocol version is used in the Dynamixel
%         # COMM_SUCCESS                = 0;            % Communication Success result value
%         # COMM_TX_FAIL                = -1001;        % Communication Tx Failed
    end
%{
    % Define an event called InsufficientFunds
    events
        InsufficientFunds 
    end
%}
    methods
        function self = classPyDynamixel(PORT, BAUD)
            self.pyObj = py.Dynamixel.Dynamixel(PORT, int32(BAUD) );
        end
        
        function delete (self)
            self.pyObj.portHandler.closePort();
        end

        function setRecommendedValue (self, DXL_IDs)
            self.pyObj.setRecommendedValue(self.mat2py( uint8(DXL_IDs) ));
        end

        function ret = readReturnDelayTime(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_RETURN_DELAY_TIME, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readDriveMode(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_DRIVE_MODE, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readOperatingMode(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_OPERATING_MODE, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readTorqueEnable(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_TORQUE_ENABLE, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readLED(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_LED, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readStatusReturnLevel(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_STATUS_RETURN_LEVEL, uint8(1), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end


        function writeReturnDelayTime(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_RETURN_DELAY_TIME, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end
        function writeDriveMode(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_DRIVE_MODE, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end
        function writeOperatingMode(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_OPERATING_MODE, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end
        function writeTorqueEnable(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_TORQUE_ENABLE, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end
        function writeLED(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_LED, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end
        function writeStatusReturnLevel(self, DXL_IDs, data)
            self.pyObj.groupAsyncWrite(self.ADDR_STATUS_RETURN_LEVEL, 1, self.mat2py( uint8(DXL_IDs) ), self.mat2py( uint8(data) ));
        end

        
        function ret = readPresentPWM(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_PRESENT_PWM, uint8(2), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readPresentVelocity(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_PRESENT_VELOCITY, uint8(4), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readPresentPosition(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_PRESENT_POSITION, uint8(4), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end
        function ret = readPresentCurrent(self, DXL_IDs)
            ret = self.pyObj.groupSyncRead(self.ADDR_PRESENT_CURRENT, uint8(2), self.mat2py( uint8(DXL_IDs) ));
            ret = self.py2mat(ret);
        end


        
        function writeGoalPWM(self, DXL_IDs, data)
            self.pyObj.groupSyncWrite(self.ADDR_GOAL_PWM, uint8(2), self.mat2py( uint8(DXL_IDs) ), self.mat2py( int16(data) ));
        end
        function writeGoalVelocity(self, DXL_IDs, data)
            self.pyObj.groupSyncWrite(self.ADDR_GOAL_VELOCITY, uint8(4), self.mat2py( uint8(DXL_IDs) ), self.mat2py( int16(data) ));
        end
        function writeGoalPosition(self, DXL_IDs, data)
            self.pyObj.groupSyncWrite(self.ADDR_GOAL_POSITION, uint8(4), self.mat2py( uint8(DXL_IDs) ), self.mat2py( int32(data) ));
        end
        function writeGoalCurrent(self, DXL_IDs, data)
            self.pyObj.groupSyncWrite(self.ADDR_GOAL_CURRENT, uint8(2), self.mat2py( uint8(DXL_IDs) ), self.mat2py( int16(data) ));
        end
        
    end % methods
    methods (Static)
        function ret = open_ports_by_serial_number(ser, BAUD)
            dev_name = string(py.Dynamixel.Dynamixel.device_name_from_serial_number(ser));
            ret = classPyDynamixel(dev_name, BAUD);
        end
        function show_list_ports ()
            py.Dynamixel.Dynamixel.show_list_ports();
        end
    end % methods (Static)
    methods (Access = protected, Static)
        function ret = mat2py( arg )
            if length(arg)==1
                ret = arg;
            else
                ret = py.list(arg');
            end
        end
        function ret = py2mat( arg )
            arg = cell(arg);
            n = length(arg);
            ret = zeros(n, 1);
            for ii=1:n
                ret(ii) = arg{ii};
            end
        end
    end % methods (Access = protected, Static)
    methods(Static)
    end % methods(Static)
end % classdef
