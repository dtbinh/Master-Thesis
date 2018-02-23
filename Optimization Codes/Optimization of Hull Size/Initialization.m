clear all;
clc;
%% parameter initialization for the specification "Buoyancy Neural"
g=9.81; % gravity acceleration
rho_f=1033; % density of surrounding water
rho_h=1050; % density of robot body part, aprroximately equal to water density 
rho_air=1.225; % density of air from Wiki
t_h=0.003; % the average thickness of the hull material is 0.003
l_constant=0.40;
a_constant=0.15;
b_constant=0.15;
V_constant=l_constant*a_constant*b_constant; % volume including battery, IMU and so on 60 cm * 20 cm * 20cm (length * width * height)
% define a third decision variable for ballast material
rho_ballast=11342; 
% calculate the total mass of battery and IMU and so on for present Snookie
S_h=4*pi*(0.25/2)^2+2*pi*0.125*0.74; % total hull area of Snookie
V_h=S_h*t_h; % total volume of the hull 
m_h=V_h*rho_h;
m_constant=32.234-m_h+30;
%% parameters for thrusters
% T100 Thruster Specification
m_T100_air=0.378; % weight in Air(with 1m cable)(with BlueESC)
m_T100_water=0.167; % weight in Water  (with 1m cable)(with BlueESC)
u_max_f_T100=ForceConversion(5.2,1);
u_max_r_T100=ForceConversion(4.1,1);
u_min_T100=ForceConversion(0.02,1);
B_T100=(m_T100_air-m_T100_water)*g;
% T200 Thruster Specification
m_T200_air=0.422; % weight in air(with 1m cable)(with BlueESC)    
m_T200_water=0.21; % weight in water (with 1m cable)(with BlueESC)
% @ 16V 
u_max_f_T200_16V=ForceConversion(11.2,1);
u_max_r_T200_16V=ForceConversion(9.0,1);
% @ 12V
u_max_f_T200_12V=ForceConversion(7.8,1);
u_max_r_T200_12V=ForceConversion(6.6,1);

%
u_min_T200=ForceConversion(0.02,1);


%% parameters for battery
% information from BlueRobotics http://docs.bluerobotics.com/batteries/
% electrical parameters:
U_B=14.8; % Nominal Voltage unit: volt V
W_B=266.4; % Nominal Capacity unit: Wh 18.0Ah 266.4Wh=18.0Ah*14.8V
% physical parameters:
D_B=0.075; % Diameter: m
l_B=0.141; % Length: m
m_B=1.152; % Weight: kg

