function CA = tau_A_cor(veh,vr)

% CRB=tau_RB_cor(veh,v) calculates the rigid body Coriolis matrix from the 
% global vehicle parameters and the relative velocity

Ca= [ zeros(3,3),           -vp(veh.Ma(1:3,:)*vr);
     -vp(veh.Ma(1:3,:)*vr), -vp(veh.Ma(4:6,:)*vr)];
 
CA=Ca*vr; 

end

