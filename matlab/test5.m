clear all; close all; clc;
%INVERSE DYNAMICS CONTROL

%PARAMETERS
load p.mat;
Td=1e-3;
T=5;
M=1;
U_m= Inf; %maximum control input
noise=1000;
x0=[1;0;0];
xi=0.9;
w0=25;



%POLE PLACEMENT OF THE ERROR DYNAMICS



eP=[complex(-w0*xi,w0*sqrt(1-xi^2)),complex(-w0*xi,-w0*sqrt(1-xi^2))];
A_e=[0 1; -1 -1];
B_e=[0;1];
K_e=place(A_e,B_e,eP);
A_e=A_e-B_e*K_e;
%PID gains
Kp=-A_e(2,1);
Kd=-A_e(2,2); 
Ki=1; 

%REFERENCE TRAJECTORY
t=(0:Td:T)';
step=ones(size(t))*M;
sine=sin(t*pi/T)*M;
X_ref=sine;
X_ref=[X_ref [0; diff(X_ref)/Td] [0; 0; diff((diff(X_ref)/Td))/Td]];

%MODEL
sys.Td=Td;
sys.A=zeros(2,2); sys.A(1,2)=1; sys.A(2,1)=-p(1)/p(3); sys.A(2,2)=-p(2)/p(3);
sys.B=zeros(2,1); sys.B(2,1)=1/p(3);
sys.C=zeros(1,2); sys.C(1,1)=1;
sys.D=0;
sys.Ad=expm(sys.A*Td);
sys.Bd=inv(sys.A)*(sys.Ad-eye(size(sys.A,1)))*sys.B;

X=x0';
e=0;
for i=1:length(t)
   e=e+X_ref(i,1)-X(i,1);
   aq=X_ref(i,3)+Kp*(X_ref(i,1)-X(i,1))+Kd*(X_ref(i,2)-X(i,2))+Ki*e*sys.Td;
   U(i)=pinv(sys.B)*([X(i,2);aq]-sys.A*X(i,1:2)');
  
   U(i)=min(U_m,U(i));
   U(i)=max(-U_m,U(i));

   X(i+1,1:2)=(sys.Ad*X(i,1:2)'+sys.Bd*U(i))'; 
   X(i+1,3)=(X(i+1,2)-X(i,2))/sys.Td;
   X(i+1,:)=X(i+1,:)+awgn([0 0 0],noise);
end  
X(end,:)=[];

subplot(3,1,1);
plot(t,X_ref(:,1),'r'); grid on; hold on;
plot(t,X(:,1),'k--'); 
title('Position');

subplot(3,1,2);
plot(t,X_ref(:,2),'r'); grid on; hold on;
plot(t,X(:,2),'k--');
title('velocity');

subplot(3,1,3);
plot(t,U,'b'); grid on; hold on;
title('Control');
