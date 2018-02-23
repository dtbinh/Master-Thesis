function B_F = FinConfigurationMatrix(varargin)
% B_F calculate the fin configuration matrix for arbitrary number of fins
% input should be in the format :

B_F=[];


% d_H is the diameter of the hull 

d_H=0.5;

% the fluid density

rho_f=1033;

for i=1:nargin
    % varargin{i}{1}: the lift coefficient C_L
    % varargin{i}{2}: the fin size: a_F and b_F
    % varargin{i}{3}: the surge velocity u
    % varargin{i}{4}: the position of the robot is characterized by two
    %                  parameters: x_F and gamma_F
    
    C_L=varargin{i}{1};
    a_F=varargin{i}{2}(1);
    b_F=varargin{i}{2}(2);
    u=varargin{i}{3};
    x_F=varargin{i}{4}(1);
    gamma=varargin{i}{4}(2);
    
    
    B_F_i=[0;
           (C_L*a_F*b_F*rho_f*u^2*cos(gamma))/2;
          -(C_L*a_F*b_F*rho_f*u^2*sin(gamma))/2;
          -(C_L*a_F*b_F*rho_f*u^2*(a_F + d_H))/4;
         (C_L*a_F*b_F*rho_f*u^2*x_F*sin(gamma))/2;
         (C_L*a_F*b_F*rho_f*u^2*x_F*cos(gamma))/2];

    B_F=horzcat(B_F,B_F_i);
    
    
    
end


end

