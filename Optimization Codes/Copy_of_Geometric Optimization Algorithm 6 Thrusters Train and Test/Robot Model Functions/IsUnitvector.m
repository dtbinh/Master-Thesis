function unitvec = IsUnitvector(x)
% in this function, we check if the input 3-dimensional vector is unit
% vector

if((x(1)^2+x(2)^2+x(3)^2)-1<1e-5)
    
    unitvec=1;
else
    
    unitvec=0;


end

