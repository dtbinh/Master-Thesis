function Cl= alpha2cl(alpha)
%% This function calculates the lift coefficient approximately
       p1 =      0.4653;
       p2=0;
       % p2 =   5.294e-15;
       p3 =      -3.219;
       % p4 =  -2.628e-14;
       p4=0;
       p5 =       8.356;
       p6=0;
       % p6 =   3.998e-14;
       p7 =       -9.75;
       p8=0;
       % p8 =  -1.913e-14;
       p9 =       3.729;
       p10=0;
       % p10 =   1.534e-15;
     Cl = p1*alpha^9 + p2*alpha^8 + p3*alpha^7 + p4*alpha^6 + p5*alpha^5 + p6*alpha^4 + p7*alpha^3 + p8*alpha^2 + p9*alpha + p10;

end

