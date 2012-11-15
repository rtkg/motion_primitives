clear all; close all; clc;

load ./data/ffj0_sine_5_5.txt;
load ./data/ffj3_sine_5_5.txt;
load ./data/ffj3_step_10_5.txt;
load ./data/ffj4_sine_5_5.txt;
load ./data/lfj5_sine_5_5.txt;
load ./data/thj1_sine_5_5.txt;
load ./data/thj2_sine_5_5.txt;
load ./data/thj3_sine_5_5.txt;
load ./data/thj4_sine_5_5.txt;
load ./data/thj5_sine_5_5.txt;
load ./data/wrj1_sine_5_5.txt;
load ./data/wrj2_sine_5_5.txt;
load ./data/ffj3_sweep_30_20.txt;

data=ffj3_sweep_30_20;

X=data(:,2:4);
f=data(:,5);

del_s=0;
p=fit2orderSystem(X,f,del_s);

dt=1e-2;
t=0:dt:data(end,1);
u=interp1(0:mean(diff(data(:,1))):data(end,1),data(:,5),t);
x0=data(1,2:3)';

[y,t,X]=simulate2orderSystem(p,u,t,x0,data);
l2_e=norm(interp1(0:mean(diff(data(:,1))):data(end,1),data(:,2),t)-X(:,1));
disp(l2_e);

save('p','p');

ss_model=[0, 1, -p(1)/p(3), -p(2)/p(3), 0, 1/p(3), 1 , 0 , 0];
disp(ss_model);

%id_data=iddata(data(:,2),data(:,5),mean(diff(data(:,1))));
%sys = n4sid(id_data,1:5); 
% A=[ 0.9953   -0.0202;   -0.2414   -0.0373];
% B=[65.1;3341.3];
% C=[0.3862 -0.0075];
% D=0;
% sys=ss(A,B,C,D);
% [y_,t_,X_]=lsim(sys,u,t,x0);


