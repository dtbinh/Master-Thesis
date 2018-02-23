function alpha = AttackAngleCalculation(u,w)
v=sqrt(u*u+w*w);
alpha=asin(w/v);
end

