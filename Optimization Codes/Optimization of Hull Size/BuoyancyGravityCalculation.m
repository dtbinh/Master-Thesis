function [B W Differ] = BuoyancyGravityCalculation(x1,x2,x3)
%% calculate the total mass of battery and IMU and so on 
S_h=4*pi*(0.25/2)^2+2*pi*0.125*0.74; % total hull area of Snookie
V_h=S_h*t_h;
m_h=V_h*rho_h;
m_constant=32.234-m_h-5;
B=rho_f*g*(pi*x1^2*x2+4/3*pi*x1^3);
W_air=(pi*x1^2*x2+4/3*pi*x1^3-V_constant-x3-(2*pi*x1*x2+4*pi*x1^2)*t_h)*rho_air*g;
W_constant=m_constant*g;
W_hull=(2*pi*x1*x2+4*pi*x1^2)*t_h*rho_h*g;
W_ballast=x3*rho_ballast*g;
W=W_air+W_constant+W_hull+W_ballast;
% val=(rho_f*g*pi*x1^2*x2-(pi*x1^2*x2-V_constant)*rho_air*g-(2*pi*x1*x2+4*pi*x1^2+4*pi*x1^2)*t_h*rho_h*g-m_constant*g)^2;
Differ=(B-W)^2;
end

