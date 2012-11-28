clear all; close all; clc;

Td=0.01;

M=0.8;
T=5;
P=5;
t1=0:Td:T+2*Td;
X1(:,1)=M*sin(t1*pi/P);
X1(:,2)=[diff(X1(:,1))/Td; 0];
X1(:,3)=[diff(X1(:,2))/Td; 0];
X1(end-1:end,:)=[];
X1=[X1; [X1(end,1) 0 0];[X1(end,1) 0 0]]; %make sure vel & acc are 0 at the end

%t1(end-1:end)=[];


X=X1;
t=t1;



% M=0.25;
% T=6;
% P=2;
% t2=0:Td:T+2*Td;
% X2(:,1)=X1(end,1)-M*sin(t2*pi/P);
% X2(:,2)=[diff(X2(:,1))/Td; 0];
% X2(:,3)=[diff(X2(:,2))/Td; 0];
% X2(end-1:end,:)=[];
% t2(end-1:end)=[];


% X3=flipud(X1);

% M=0.5;
% T=3;
% P=6;
% t3=0:Td:T+2*Td;
% X3(:,1)=X2(end,1)-M*sin(t3*pi/P);
% X3(:,2)=[diff(X3(:,1))/Td; 0];
% X3(:,3)=[diff(X3(:,2))/Td; 0];
% X3(end-1:end,:)=[];
% t3(end-1:end)=[];

% X=[X1;X2;X3];
% t=0:Td:t1(end)+t2(end)+t1(end)+2*Td;



% X=[0 0 0];
% for i=1:length(t)
%     X(i,3)=M*sin(t(i)*pi/P);
%     X(i+1,2)=X(i,2)+Td*X(i,3);
%     X(i+1,1)=X(i,1)+Td*X(i,2);
% end 
% X(end,:)=[];

figure;
subplot(3,1,1);
plot(t,X(:,1)); grid on;
title('Position');

subplot(3,1,2);
plot(t,X(:,2)); grid on;
title('Velocity');

subplot(3,1,3);
plot(t,X(:,3)); grid on;
title('Acceleration');

fileID = fopen('./trajectory.txt','w','n');
fclose(fileID);
dlmwrite('./trajectory.txt',X,'-append','delimiter',' ','precision',12,'newline','pc');

