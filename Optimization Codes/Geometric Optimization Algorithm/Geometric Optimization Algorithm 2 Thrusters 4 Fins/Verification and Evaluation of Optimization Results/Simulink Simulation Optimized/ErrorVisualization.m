% Fin4
figure;
plot(cell2mat(input_error_c),'LineWidth',3);
title('Control Input Optimization');
xlabel('iterations');
ylabel('value of obejective function');
legend('fval_{input}');
% 
figure;
plot(cell2mat(fval_pos_c),'LineWidth',3);
title('Position Optimization');
xlabel('iterations');
ylabel('value of obejective function');
legend('fval_{pos}');
% 