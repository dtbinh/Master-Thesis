model = LTISystem('A', [ -0.79  -0.3 -0.1; 0.5  0.82 1.23; 0.52 -0.3 -0.5], 'B', [ -2.04 -0.21; -1.28  2.75; 0.29 -1.41]);
model.x.min = [-10; -9; -8];
model.x.max = [10; 9; 8];
model.u.min = [-2; -3];
model.u.max = [2; 3];
model.x.with('reference');
model.x.reference = 'free';
model.x.penalty = OneNormFunction( diag([0.5, 1, 0]) );
model.u.penalty = OneNormFunction( eye(2) )
ctrl = MPCController(model, 4)
loop = ClosedLoop(ctrl, model);
x0 = [0; 0; 0];
Nsim = 30;
xref = [0; 1; 0];
data = loop.simulate(x0, Nsim, 'x.reference', xref)
subplot(2, 1, 1);
plot(1:Nsim, data.X(:,1:Nsim), 'linewidth', 2);
axis([1, Nsim, -2, 2]);
grid on;  title('states');
subplot(2, 1, 2); plot(1:Nsim, data.U, 'linewidth', 2);
axis([1, Nsim, -1, 1]);
grid on; title('inputs');