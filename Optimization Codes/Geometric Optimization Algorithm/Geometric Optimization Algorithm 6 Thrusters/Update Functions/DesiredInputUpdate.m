%% After the configuration is changed, we must update the desired input 


%% we should update the centre of mass first: it is important %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate the center of mass CG 

% since it will influence the rigid body Coriolis matrix
% for calculation we need to update the veh.Mrb in the global variable veh
 
% use the function CenterofMass to update the center of mass of mass which
% is located in "Modular Modelling of Underwater Robots"


% 
% The input should be in the format {r_Gi,[x_Gi,y_Gi,z_Gi]}

% The functionas are located in "Modular Modelling of Underwater Robots/MassMomentinertiaCalculation"

% use the function "CenterofMass" to update the center of mass CG 

%

% use the current geometric center of 6 thrusters to update the
% geometric center, and their center of mass 




 

% update the rigid body Coriolis matrix

C_RB1=tau_RB_cor(veh,v_d_1');


C_RB2=tau_RB_cor(veh,v_d_2');


C_RB3=tau_RB_cor(veh,v_d_3');


C_RB4=tau_RB_cor(veh,v_d_4');


C_RB5=tau_RB_cor(veh,v_d_5');


C_RB6=tau_RB_cor(veh,v_d_6');


C_RB7=tau_RB_cor(veh,v_d_7');

% after the restoring force term

% remember the structure of the 

% function g = gvect(W,B,theta,phi,r_g,r_b)

% actually the only parameter that should be changed is the fifth parameter
% veh.G_b
% The center of mass CG will inluence by different geometric configurations



G1=gvect(veh.G,veh.B,pitch_d_1,roll_d_1,veh.G_b,veh.B_b);

G2=gvect(veh.G,veh.B,pitch_d_2,roll_d_2,veh.G_b,veh.B_b);

G3=gvect(veh.G,veh.B,pitch_d_3,roll_d_3,veh.G_b,veh.B_b);

G4=gvect(veh.G,veh.B,pitch_d_4,roll_d_4,veh.G_b,veh.B_b);

G5=gvect(veh.G,veh.B,pitch_d_5,roll_d_5,veh.G_b,veh.B_b);

G6=gvect(veh.G,veh.B,pitch_d_6,roll_d_6,veh.G_b,veh.B_b);

G7=gvect(veh.G,veh.B,pitch_d_7,roll_d_7,veh.G_b,veh.B_b);

%% now let us calculate the new desired input 

% The desired input has four components tau_d=

tau_d_1=C_RB1+C_A1+D1+G1;

tau_d_2=C_RB2+C_A2+D2+G2;

tau_d_3=C_RB3+C_A3+D3+G3;

tau_d_4=C_RB4+C_A4+D4+G4;

tau_d_5=C_RB5+C_A5+D5+G5;

tau_d_6=C_RB6+C_A6+D6+G6;

tau_d_7=C_RB7+C_A7+D7+G7;

