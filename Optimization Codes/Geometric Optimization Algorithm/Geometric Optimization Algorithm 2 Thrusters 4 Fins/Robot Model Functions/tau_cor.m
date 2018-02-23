function [CRB,CA]=tau_cor(veh,v,vr)

% [CRB,CA]=tau_cor(veh,v,vr) calculates coriolis forces from 
% vehicle variables and generalized velocities v and vr

% this is verified by the similiar funtion in Fossen's book p.56

Crb=[ zeros(3,3),           -vp(veh.Mrb(1:3,:)*v);
     -vp(veh.Mrb(1:3,:)*v), -vp(veh.Mrb(4:6,:)*v)];

Ca= [ zeros(3,3),           -vp(veh.Ma(1:3,:)*vr);
     -vp(veh.Ma(1:3,:)*vr), -vp(veh.Ma(4:6,:)*vr)];

CRB=Crb*v;
CA=Ca*vr;

