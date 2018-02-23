%% Snookie Configuration Parameter used to test the geometric optimization 

% in this situation we do not consider the result drom the first
% optimization phase

% total mass

m=32.23;

I_cg=[0.23,0,0;0,1.68,0;0,0,1.70];

% we can assume the mass of the hull
% is 25 kg

m_H=25;

%% 4 fins 

% for the first fin 

a_F1=0.104;

b_F1=0.25;

% center of the fin 

C_F1=[-0.367;0.05;0];

% mass of the fin


% for the second fin 

a_F2=0.104;

b_F2=0.25;

% center of the fin 

C_F2=[-0.367;0;0.05];

% for the third fin

a_F3=0.104;

b_F3=0.25;

% center of the fin

C_F3=[-0.367;-0.05;0];


% for the fourth fin 

a_F4=0.104;

b_F4=0.25;

% center of the fin 

C_F4=[-0.393;0;-0.05];

%% Hull Parameter

a_H=0.42;

b_H=0.136;


%% Snookie thruster Configuration 

% for all thrusters assume they have the  

% we have totally six thrusters

% left bottom thruster location

r_T1=[-0.24078;-0.12327;0.12131];

e_T1=[1;-1;1];

% left top thruster location

r_T2=[-0.24078;-0.12327;-0.12131];

e_T2=[1;-1;-1];

% right bottom thruster location

r_T3=[-0.24078;0.12327;0.12131];

e_T3=[1;1;1];

% right top thruster location 

r_T4=[-0.24078;0.12327;-0.12131];

e_T4=[1;1;-1];

% midright thruster location

r_T5=[0.00276;-0.1799;0];

e_T5=[0;0;-1];

% midleft thruster location

r_T6=[0.00276;0.1799;0];

e_T6=[0;0;-1];










