function [p o]=fit2OrderSystemMakkar(X,f,lb,ub,pI)

% X     ... columns holding q,dq,ddq respectively
% f     ... control input

path = strcat(pwd,'/data/');
args = path;

%Write X to a file to be read by ACADO
fileID = fopen(strcat(path,'kinematics.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'kinematics.dat'),X,'-append','delimiter',' ','precision',12,'newline','pc');

%Write f to a file to be read by ACADO
fileID = fopen(strcat(path,'force.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'force.dat'),f,'-append','delimiter',' ','precision',12,'newline','pc');

%Write lb to a file to be read by ACADO
fileID = fopen(strcat(path,'lb.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'lb.dat'),lb,'-append','delimiter',' ','precision',12,'newline','pc');

%Write ub to a file to be read by ACADO
fileID = fopen(strcat(path,'ub.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'ub.dat'),ub,'-append','delimiter',' ','precision',12,'newline','pc');

%Write pI to a file to be read by ACADO
fileID = fopen(strcat(path,'pI.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'pI.dat'),pI,'-append','delimiter',' ','precision',12,'newline','pc');

executable = 'fit2OrderSystemMakkar';

disp(' ');
disp([' Execute command:  ', executable, ' ', args]);
disp(' ');

out = system([strcat(pwd,'/bin/'), executable, ' ', args]);

p=load(strcat(path,'solution.dat'));
o=p(end); p(end)=[];
