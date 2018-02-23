%% under the initial geometric configuration we should calculate the intial desired input for all trim trajectory segments

% in order to initialise 
% update the rigid body Coriolis matrix

C_RB1_init=tau_RB_cor(veh,v_d_1');


C_RB2_init=tau_RB_cor(veh,v_d_2');



% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1_init=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,veh.G_b,veh.B_b);

G2_init=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,veh.G_b,veh.B_b);


%% now let us calculate the new desired input 

% The desired input has four components tau_d=

tau_d_1=C_RB1_init+C_A1+D1+G1_init;

tau_d_2=C_RB2_init+C_A2+D2+G2_init;

