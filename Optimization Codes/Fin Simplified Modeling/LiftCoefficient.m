function k_L = LiftCoefficient(rho,A,u_f,C_L)
% Calculaze the lift coefficient
% Input: 
% 1.rho----density
% 2.A -----area
% 3.u_f----fin velocity
% 4.C_L---- the lift coefficient

k_L=0.5*rho*A*u_f^2*C_L;

end

