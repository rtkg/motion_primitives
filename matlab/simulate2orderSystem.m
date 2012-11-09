function [y,t,X]=simulate2orderSystem(p,u,t,x0,data)

A=[0 1;-p(1)/p(3) -p(2)/p(3)];
B=[0;1/p(3)];
C=[1 0];
D=0;

sys=ss(A,B,C,D);

[y,t,X]=lsim(sys,u,t,x0);

%differentiate to get accelerations
dt=mean(diff(t));
ddq=[0; diff(X(:,2))/dt];
X=[X ddq];

figure;
subplot(3,1,1);
plot(t,X(:,1),'k--');grid on; hold on;
plot(data(:,1),data(:,2),'r');
title('Position');

subplot(3,1,2);
plot(t,X(:,2),'k--');grid on; hold on;
plot(data(:,1),data(:,3),'r');
title('Velocity');

subplot(3,1,3);
plot(t,X(:,3),'k--');grid on; hold on;
plot(data(:,1),data(:,4),'r');
title('Acceleration');