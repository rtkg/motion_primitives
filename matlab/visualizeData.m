function visualizeData(data,name)

figure;
subplot(2,2,1); 
plot(data(:,1),data(:,2)); grid on;
title(strcat(name,' Position'));
xlabel('t [s]'); ylabel('q [rad]');

subplot(2,2,2); 
plot(data(:,1),data(:,3)); grid on;
title(strcat(name,' Velocity'));
xlabel('t [s]'); ylabel('dq [rad/s]');

subplot(2,2,3); 
plot(data(:,1),data(:,4)); grid on;
title(strcat(name,' Acceleration'));
xlabel('t [s]'); ylabel('ddq [rad/s^2]');

subplot(2,2,4); 
plot(data(:,1),data(:,5)); grid on;
title(strcat(name,' Force'));
xlabel('t [s]'); ylabel('f');

