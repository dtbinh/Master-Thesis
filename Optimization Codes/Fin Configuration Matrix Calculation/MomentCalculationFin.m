function M_F = MomentCalculationFin(C_L,a_F,b_F,u,gamma_F,d_H)

% This function calculates the desired moment for each trim trajectory
 
rho_f=1033;

M_F=[-(C_L*a_F*b_F*rho_f*u^2*(a_F + d_H))/4,0,0;
         0,(C_L*a_F*b_F*rho_f*u^2*sin(gamma_F))/2,0;
         0,0,(C_L*a_F*b_F*rho_f*u^2*cos(gamma_F))/2];


end

