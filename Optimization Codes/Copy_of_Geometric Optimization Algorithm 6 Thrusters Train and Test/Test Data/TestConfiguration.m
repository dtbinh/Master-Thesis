%% We use the Snookie Configuration to test the Optimization Algorithm 

% The parameters should be adapted from the Snookie

% The hull size is fixed during the optimization phase 

% The parameters of 

m_H=25;
I_cg_H=[0.23,0,0;0,1.68,0;0,0,1.70];


% The following parameters should be regarded as initialization parameters 
% fin parameters: four fins


a_F1=0.104;
b_F1=0.25;
m_F1=1.2;
C_F1=[-0.367;0.05;0];

a_F2=0.104;
b_F2=0.25;
m_F2=1.2;
C_F2=[-0.367;0;0.05];

a_F3=0.104;
b_F3=0.25;
m_F3=1.2;
C_F3=[-0.367;-0.05;0];

a_F4=0.104;
b_F4=0.25;
m_F4=1.2;
C_F4=[-0.393;0;-0.05];

% paras for thrusters:

r_T5=[0.00276;-0.1799;0];
m_T5=0.12;

r_T6=[0.00276;0.1799;0];
m_T6=0.12;




