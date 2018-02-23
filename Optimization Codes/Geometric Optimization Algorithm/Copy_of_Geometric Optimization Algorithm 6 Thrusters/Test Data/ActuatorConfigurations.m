%% For different configurations, we have different input configuration matrix and different input matrix 
% we put them together so that we can just choose from one of them 

%%  generalised forces con

% syms X_l Y_l Z_l K_l M_l N_l

% BH=inv(M)*[X_l;Y_l;Z_l;K_l;M_l;N_l];  

% B_v_l=jacobian(BH_v,[X_l Y_l Z_l K_l M_l N_l]);

% B_w_l=jacobian(BH_w,[X_l Y_l Z_l K_l M_l N_l]);

%% Configuration of 4 fins and two thrusters

% thruster force

% syms T1 T2 

% deflection angle

% syms delta1 delta2 delta3 delta4

% BH=inv(M)*B*[T1;T2;delta1;delta2;delta3;delta4];

% B_v_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);

% B_w_l=jacobian(BH_w,[T1 T2 delta1 delta2 delta3 delta4]);


