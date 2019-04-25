function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
%
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the
%   forward and angular velocity commands to drive a differential-drive
%   robot whose wheel motors saturate at +/- maxV.
%
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
%
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SHI, KOWIN

L = wheel2Center*2;
vrminusvl = angVel*L;
vrplusvl = 2*fwdVel;
%find values for expressions needed to calculate vr and vl from V and w
vr = (vrplusvl + vrminusvl)/2;
vl = vrplusvl-vr;
%get desired vr and vl
if abs(vr)<=maxV && abs(vl)<=maxV
    %return same if meets max velocity requirement
    cmdV = fwdVel;
    cmdW = angVel;
else
    %finds larger of the two wheel speeds
    [biggerone,I] = max([abs(vr) abs(vl)]);
    %scales it down to 0.5
    scale = maxV/biggerone;
    %outputs the scaled results
    cmdW = vrminusvl*scale/L;
    cmdV = vrplusvl*scale/2;
end

end
