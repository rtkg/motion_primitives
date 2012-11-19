function [X]=simulate2OrderSystemMakkar(p,u,t,x0,data)

% A=[0 1;-p(1)/p(3) -p(2)/p(3)];
% B=[0;1/p(3)];
% C=[1 0];
% D=0;
x0=x0(:);
X=x0';
n=length(t);
dt=mean(diff(t));

for i=1:n
 X(i,3)=1/p(3)*(-p(2)*X(i,2)-p(1)*X(i,1)-makkarFrictionForce(X(i,2),p(4:9)) +u(i));
 X(i+1,1)=X(i,1)+X(i,2)*dt;
 X(i+1,2)=X(i,2)+X(i,3)*dt;
end
X(end,:)=[];

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