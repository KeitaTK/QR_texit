clear;
classPyDynamixel.show_list_ports();
dyn = classPyDynamixel.open_ports_by_serial_number("FT6Z5XDEA", 1e6);
% dyn = classPyDynamixel('COM5', 1e6); % (COM port, Baud)
DXL_IDs = [1; 2];
% DXL_IDs = 2;
dyn.setRecommendedValue(DXL_IDs);
% 
testReadWritePosition(dyn, DXL_IDs);
testReadWriteVelocity(dyn, DXL_IDs);
testReadWriteCurrent(dyn, DXL_IDs);
testReadSettings(dyn, DXL_IDs);

dyn.delete();

function testReadWritePosition(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 3*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    dyn.writeGoalPosition(DXL_IDs, (90/0.088)*ones(len,1));
    pause(1.0);
    tic;
    a = dyn.readPresentPosition(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    tic;
    dyn.writeGoalPosition(DXL_IDs, 0*ones(len,1));
    fprintf('write: %.3f(ms)\n', toc*1e3);
    pause(1.0);
    tic;
    a = dyn.readPresentPosition(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);

    dyn.writeTorqueEnable(DXL_IDs, zeros(len,1));
end

function testReadWriteVelocity(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 1*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    dyn.writeGoalVelocity(DXL_IDs, 200*ones(len,1));
    pause(1.0);
    tic;
    a = dyn.readPresentVelocity(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    tic;
    dyn.writeGoalVelocity(DXL_IDs, -200*ones(len,1));
    fprintf('write: %.3f(ms)\n', toc*1e3);
    pause(1.0);
    tic;
    a = dyn.readPresentVelocity(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    
    dyn.writeTorqueEnable(DXL_IDs, zeros(len,1));
end

function testReadWriteCurrent(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 0*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    dyn.writeGoalCurrent(DXL_IDs, 100*ones(len,1));
    pause(1.0);
    tic;
    a = dyn.readPresentCurrent(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    tic;
    dyn.writeGoalCurrent(DXL_IDs, -100*ones(len,1));
    fprintf('write: %.3f(ms)\n', toc*1e3);
    pause(1.0);
    tic;
    a = dyn.readPresentCurrent(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    
    dyn.writeTorqueEnable(DXL_IDs, zeros(len,1));
end

function testReadSettings(dyn, DXL_IDs)
    disp(dyn.readReturnDelayTime(DXL_IDs)');
    disp(dyn.readDriveMode(DXL_IDs)');
    disp(dyn.readOperatingMode(DXL_IDs)');
    disp(dyn.readTorqueEnable(DXL_IDs)');
    disp(dyn.readLED(DXL_IDs)');
    disp(dyn.readStatusReturnLevel(DXL_IDs)');
end
