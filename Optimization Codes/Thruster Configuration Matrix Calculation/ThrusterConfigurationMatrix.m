function B_T = ThrusterConfigurationMatrix(varargin)
% B_T calculate the thruster configuration matrix for arbitrary number of
% thrusters
% input should be in the format : 
% r_i: the position of thrusters in body frame [r_xi;r_yi;r_zi]
% b_i: motor spin direction, -1 means counterclockwise, 1 means clockwise
% d_i: the orientation of thrusters, unit sphere: [d_xi;d_yi;d_zi]
lambda=0.5;
B_T=[];
for i=1:nargin
    % varargin{i}{1}: the position vector r_i
    % varargin{i}{2}: the motor spin direction b_i
    % varargin{i}{3}: the orientation vector d_i
    F=varargin{i}{3};
    M=varargin{i}{2}*lambda*varargin{i}{3}+vp(varargin{i}{1},varargin{i}{3});
    B_i=vertcat(F,M);
    B_T=horzcat(B_T,B_i);
end

end

