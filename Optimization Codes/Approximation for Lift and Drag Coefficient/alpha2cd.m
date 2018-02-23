function Cd = alpha2cd(alpha)
%% The function calculate the drag coefficient approximately wrt. alpha
       p1=0;
       % p1 =  -1.471e-15;
       p2 =     -0.1463;
       p3=0;
       % p3 =   8.682e-15;
       p4 =       1.234;
       p5=0;
       % p5 =  -1.682e-14;
       p6 =       -3.61;
       p7=0;
       % p7 =   1.196e-14;
       p8 =       3.604;
       p9=0;
       % p9 =  -2.004e-15;
       p10 =     0.09403;
      
Cd=p1*alpha^9 + p2*alpha^8 + p3*alpha^7 + p4*alpha^6 + p5*alpha^5 + p6*alpha^4 + p7*alpha^3 + p8*alpha^2 + p9*alpha + p10;
end

