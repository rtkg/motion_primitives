clear all; close all; clc;
%LQR CONTROL

load p.mat;
Td=1e-3;
T=5;
M=1;
U_m= Inf;
noise=1000;
x0=[0;0];
rho=1e5;

t=(0:Td:T)';
step=ones(size(t))*M;
sine=sin(t*pi/T)*M;
X_ref=sine;
X_ref=[X_ref [0; (diff(X_ref)/Td)]];

sys.Td=Td;
sys.A=zeros(2,2); sys.A(1,2)=1; sys.A(2,1)=-p(1)/p(3); sys.A(2,2)=-p(2)/p(3);
sys.B=zeros(2,1); sys.B(2,1)=1/p(3);
sys.C=zeros(1,2); sys.C(1,1)=1;
sys.D=0;
sys.Ad=expm(sys.A*Td);
sys.Bd=inv(sys.A)*(sys.Ad-eye(size(sys.A,1)))*sys.B;
dsys=ss(sys.A,sys.B,sys.C,sys.D);

%controler parameters
R=1; Q=rho*sys.C'*sys.C; N=zeros(size(sys.B)); 
K=lqr(dsys,Q,R,N)
A_=[sys.A sys.B; sys.C sys.D];
N_=inv(A_)*[zeros(size(sys.A,1),1); 1];
N=N_(end)+K*N_(1:end-1);


X=x0';
for i=1:length(t)
   U(i)=-K*X(i,:)'+N*X_ref(i,1);
    
   U(i)=min(U_m,U(i));
   U(i)=max(-U_m,U(i));

   X(i+1,:)=(sys.Ad*X(i,:)'+sys.Bd*U(i))'; 
   X(i+1,:)=X(i+1,:)+awgn([0 0],noise);
end  
X(end,:)=[];

subplot(3,1,1);
plot(t,X(:,1),'k--'); grid on; hold on;
plot(t,X_ref(:,1),'r');
title('Position');

subplot(3,1,2);
plot(t,X(:,2),'k--'); grid on; hold on;
plot(t,X_ref(:,2),'r');
title('velocity');

subplot(3,1,3);
plot(t,U,'b'); grid on; hold on;
title('Control');
