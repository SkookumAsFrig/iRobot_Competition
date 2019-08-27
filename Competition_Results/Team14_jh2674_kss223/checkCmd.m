function [cmdV,cmdW] = checkCmd(fwdVel,angVel,maxV,wheel2Center)
% scale forward and angular velocity commands to avoid
% saturating motors.
[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
% check NaN
if isnan(cmdV)
    cmdV = 0;
end
if isnan(cmdW)
    cmdW = 0;
end
end