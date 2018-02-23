function k_D = DragCoefficient(rho,A,u_f,C_D)
% Calculate the drag coefficient
% Input:
% 1.rho----density
% 2.A ---- area
% 3.u_f--- fin velocity
% 4.C_D--- the lift coefficient

k_D=0.5*rho*A*u_f^2*C_D;

end

