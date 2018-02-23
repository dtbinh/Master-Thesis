function CRB = tau_RB_cor(veh,v)

% CRB=tau_RB_cor(veh,v) calculates the rigid body Coriolis matrix from the 
% global vehicle parameters and the generalised velocity

Crb=[ zeros(3,3),           -vp(veh.Mrb(1:3,:)*v);
     -vp(veh.Mrb(1:3,:)*v), -vp(veh.Mrb(4:6,:)*v)];
 
CRB=Crb*v;


end

