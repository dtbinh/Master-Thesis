function Mrb = RigidBodyInertiaMatrix(m,I_cg,r_G)
% This function calculate the rigid body inertia matrix 
% input: 1. m: the total mass of all modules
%        2. r_G: the distance betweenn combinational center of gravity and
%        the origin point (geometric center of the hull)
%        3. I_cg: the inertial tensor of the rigid body with respect to the
%        origin point at the center of the gravity


Mrb=[m*eye(3),-m*vp(r_G);m*vp(r_G),I_cg-m*vp(r_G)*vp(r_G)];


end

