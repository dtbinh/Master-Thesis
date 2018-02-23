
M=veh.Mrb+veh.Ma;
%%  The operating inputs should be calculated according to the dynamic equation of the robots
v=[linear_velocity_x(1);linear_velocity_y(1);linear_velocity_z(1);angular_velocity_roll(1);angular_velocity_pitch(1);angular_velocity_yaw(1)];
u0=tau_d(:,:,1);
%% specify the linearization point
x0=v; 
%% linearize the model about the point (x0,u0)
[A1,B1,C1,D1]=linmod('mlinear',x0,u0);
      

