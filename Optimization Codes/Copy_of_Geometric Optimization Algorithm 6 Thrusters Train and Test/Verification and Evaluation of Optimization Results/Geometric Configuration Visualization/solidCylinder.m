function [x,y,z] = solidCylinder(varargin)

    %// Basic checks
    assert(nargin >= 1, 'Not enough input arguments.');
    assert(nargin <= 3, 'Too many input arguments.');
    assert(nargout <= 3, 'Too many output arguments.');

    %// Parse input
    N  = 20;
    Ax = [];
    switch nargin
        case 1 %// R
            R  = varargin{1};
        case 2  %// Ax, R  or  R, N
            if ishandle(varargin{1})
                Ax = varargin{1};
                R  = varargin{2};                
            else
                R  = varargin{1};
                N  = varargin{2};
            end

        case 3 %// Ax, R, N
            Ax = varargin{1};
            R  = varargin{2};
            N  = varargin{3};
    end

    %// Check input arguments
    if ~isempty(Ax)
        assert(ishandle(Ax) && strcmp(get(Ax, 'type'), 'axes'),...
            'Argument ''Ax'' must be a valid axis handle.');        
    else
        Ax = gca;
    end

    assert(isnumeric(R) && isvector(R) && all(isfinite(R)) && all(imag(R)==0) && all(R>0),...
        'Argument ''R'' must be a vector containing finite, positive, real values.');    
    assert(isnumeric(N) && isscalar(N) && isfinite(N) && imag(N)==0 && N>0 && round(N)==N,...
        'Argument ''N'' must be a finite, postive, real, scalar integer.');

    %// Compute cylinder coords (mostly borrowed from builtin 'cylinder')   
    theta         = 2*pi*(0:N)/N;
    sintheta      = sin(theta); 
    sintheta(N+1) = 0;

    M = length(R);
    if M==1 
        R = [R;R]; M = 2; end

    x = R(:) * cos(theta);
    y = R(:) * sintheta;
    z = (0:M-1).'/(M-1) * ones(1,N+1);  %'

    if nargout == 0                
        oldNextPlot = get(Ax, 'NextPlot');         
        set(Ax, 'NextPlot', 'add');

        %// The side of the cylinder
        surf(x,y,z, 'parent',Ax); 
        %// The bottom 
        patch(x(1,:)  , y(1,:)  , z(1,:)  , z(1,:)  );
        %// The top
        patch(x(end,:), y(end,:), z(end,:), z(end,:));

        set(Ax, 'NextPlot', oldNextPlot);
    end

end