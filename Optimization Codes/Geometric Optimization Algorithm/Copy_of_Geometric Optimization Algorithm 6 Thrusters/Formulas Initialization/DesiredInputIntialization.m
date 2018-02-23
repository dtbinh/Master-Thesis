%% under the initial geometric configuration we should calculate the intial desired input for all trim trajectory segments

% in order to initialise 
% update the rigid body Coriolis matrix

C_RB1_init=tau_RB_cor(veh,v_d_1');


C_RB2_init=tau_RB_cor(veh,v_d_2');


C_RB3_init=tau_RB_cor(veh,v_d_3');


C_RB4_init=tau_RB_cor(veh,v_d_4');


C_RB5_init=tau_RB_cor(veh,v_d_5');


C_RB6_init=tau_RB_cor(veh,v_d_6');


C_RB7_init=tau_RB_cor(veh,v_d_7');

% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1_init=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,veh.G_b,veh.B_b);

G2_init=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,veh.G_b,veh.B_b);

G3_init=gvect(veh.G,veh.B,pitch_d_3,roll_d_3,veh.G_b,veh.B_b);

G4_init=gvect(veh.G,veh.B,pitch_d_4,roll_d_4,veh.G_b,veh.B_b);

G5_init=gvect(veh.G,veh.B,pitch_d_5,roll_d_5,veh.G_b,veh.B_b);

G6_init=gvect(veh.G,veh.B,pitch_d_6,roll_d_6,veh.G_b,veh.B_b);

G7_init=gvect(veh.G,veh.B,pitch_d_7,roll_d_7,veh.G_b,veh.B_b);

%% now let us calculate the new desired input 

% The desired input has four components tau_d=

tau_d_1=C_RB1_init+C_A1+D1+G1_init;

tau_d_2=C_RB2_init+C_A2+D2+G2_init;

tau_d_3=C_RB3_init+C_A3+D3+G3_init;

tau_d_4=C_RB4_init+C_A4+D4+G4_init;

tau_d_5=C_RB5_init+C_A5+D5+G5_init;

tau_d_6=C_RB6_init+C_A6+D6+G6_init;

tau_d_7=C_RB7_init+C_A7+D7+G7_init;
