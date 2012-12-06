clear all; close all; clc;

load 'LFJ3_sine_0.8_5_p_-3200_i_-200_d_-800_iclamp_32.0_2012-11-29-18-57-39.txt';
load 'LFJ3_sine_0.8_5_p_-800_i_-40_d_-180_iclamp_4.0_2012-11-29-18-41-42.txt';

data=LFJ3_sine_0_8_5_p__3200_i__200_d__800_iclamp_32_0_2012_11_29_18;
%data=LFJ3_sine_0_8_5_p__800_i__40_d__180_iclamp_4_0_2012_11_29_18_41;


t=data(:,1)*1e-9; t=t-t(1); dt=mean(diff(t));
q_r=data(:,4);
dq_r=[diff(q_r)/dt; 0];
ddq_r=[diff(dq_r)/dt; 0];
q=data(:,5);
dq=data(:,6);
q_f=data(:,7);
dq_f=data(:,8);
dq_r=data(:,9);
e_p=data(:,10);
e_v=dq_r-dq_f;
f=data(:,12);
clear data




subplot(2,2,1);
plot(t,q_f,'--k'); grid on; hold on;
plot(t,q_r,'r');
title('Position');
legend('filtered position','reference position');

subplot(2,2,2);
plot(t,dq_f,'--k'); grid on; hold on;
plot(t,dq_r,'r');
title('Velocity');
legend('filtered velocity','reference velocity');

subplot(2,2,3);
plot(t,e_p,'b'); grid on; hold on;
plot(t,e_v,'r');
title('errors');
legend('error position','error velocity');

subplot(2,2,4);
plot(t,f,'b'); grid on; 
title('Effort');
