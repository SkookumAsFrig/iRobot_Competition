function [mu_t, sig_t, kt] = extendedKalmanFilter(h_handle,g_handle,Hjac_handle,Gjac_handle,mubar,sigbar,u,R,Q,zt)
% hGPS: predict the depth measurements for a robot operating
% in a known map, given the expected range measurements.
%
%   depth = depthPredict(angles, range) returns
%   the expected depth measurements for a robot given range measurements.
%
%   INPUTS
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
%       range           K-by-1 vector of ranges (meters)
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   SHI, KOWIN

Gt = feval(Gjac_handle,mubar,u);
%prediction step
mu_tbar = feval(g_handle,mubar,u);
sig_tbar = Gt*sigbar*Gt' + R;
%prediction step end
Ht = feval(Hjac_handle,mu_tbar);
meas_pred = feval(h_handle,mu_tbar);
%kalman gain
kt = sig_tbar*Ht'/(Ht*sig_tbar*Ht'+Q);
[M,~] = size(kt);
%update step
mu_t = mu_tbar + kt*(zt-meas_pred);
sig_t = (eye(M) - kt*Ht)*sig_tbar;

end
