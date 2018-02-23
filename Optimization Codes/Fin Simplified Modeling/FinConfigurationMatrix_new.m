function B_F = FinConfigurationMatrix_new(varargin)

for i=1:nargin
    % varargin{i}{1}: the position of the fin in body frame {b}
    % varargin{i}{4}: char to identify the type of the fin:
    if strcmp(varargin{i}{4},'top')==1
    elseif strcmp(varargin{4},'bottom')==1
    elseif strcmp(varargin{4},'port')==1
    elseif strcmp(varargin{4},'starboard')==1
    end
                      
end

end

