function r_G = CenterofMass(varargin)

% Calculate the centre of mass for all modules
% The number of input is variable, 
% All variables are in the format: cell:{m_Gi,[x_Gi,y_Gi,z_Gi]}

% The location of each subsubmodules r_Gi=[x_Gi,y_Gi,z_Gi]

% the centre of mass is calculated as follows: Laetitia Papaxanthos
 

% r_G=sum(m_Gi*r_Gi)/sum(m_Gi)

num=[0;0;0]; %: sum(m_Gi*r_Gi)

den=0;       %: sum(m_Gi)
for i=1:nargin
    num=varargin{i}{1}*varargin{i}{2}+num;
    den=den+varargin{i}{1};
end
r_G=num/den;
end

