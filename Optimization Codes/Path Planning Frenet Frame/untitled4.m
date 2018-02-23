t = -pi/2:pi/50:2*pi;

ct = cos(t);
st = -sin(t);

figure
plot3(ct,st,-(t+pi/2))

hold on;

r=1
teta=-pi:0.01:pi
x=r*cos(teta);
y=r*sin(teta)
plot3(x,y,zeros(1,numel(x)),'--')

hold on;

plot3(x,y,zeros(1,numel(x))-2*pi,'--')

