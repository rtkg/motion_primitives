clear all; close all; clc;
%fit 2nd order linear system + Makkar friction with genetic algorithm
%system identification for the lfj3 joint on the real UPMC hand

path='./2012-11-23_SysIdent_LFJ3/';


%pI=[53.0982 60.5957 0.1200 54.4336 5.4780 0.3819 47.9656 12.0178 0.0]'; %lfj3_sine_5.0_150.txt

%pI=[23.2414 40.4836 0.2388 49.5209 1.8260 0.3214 28.0423 11.9166 0]'; %lfj3_sweep_10.0_150at45deg_0.5_5.0.txt

pI=[59.3302 25.7325 0.0500 48.1827 7.9387 0.0852 48.5341 6.8736 0]'; %combined lfj3_sine_5.0_150.txt,lfj3_sweep_10.0_150at45deg_0.5_5.0.txt

 
         

pI=[       0.0010   61.3451    0.7010  138.6009    3.8127    0.8693    7.8032    1.9592  1e-6 ]';
experiments{1}.file='lfj3_sine_5.0_-100at45deg.txt';
experiments{2}.file='lfj3_sine_5.0_100.txt';	      
experiments{3}.file='lfj3_sine_5.0_-150at45deg.txt';
experiments{4}.file='lfj3_sine_5.0_150at45deg.txt';
experiments{5}.file='lfj3_sine_5.0_150.txt';
experiments{6}.file='lfj3_sine_5.0_-180at45deg.txt';
experiments{7}.file='lfj3_sine_5.0_180at45deg.txt';
experiments{8}.file='lfj3_sine_5.0_200at45deg.txt';
experiments{9}.file='lfj3_step_0.3_350.txt';    
experiments{10}.file='lfj3_step_0.5_200.txt';	      
experiments{11}.file='lfj3_step_0.5_220.txt';	      
experiments{12}.file='lfj3_step_0.5_250.txt';	      
experiments{13}.file='lfj3_step_0.5_280.txt';	      
experiments{14}.file='lfj3_step_0.5_300.txt';	      
experiments{15}.file='lfj3_step_1.0_-100at45deg.txt';
experiments{16}.file='lfj3_step_1.0_100at45deg.txt';
experiments{17}.file='lfj3_step_1.0_-150at45deg.txt';
experiments{18}.file='lfj3_step_1.0_150at45deg.txt';
experiments{19}.file='lfj3_step_1.0_160at45deg.txt';
experiments{20}.file='lfj3_step_1.0_170at45deg.txt'; 
experiments{21}.file='lfj3_step_1.0_-180at45deg.txt'; 
experiments{22}.file='lfj3_step_1.0_180at45deg.txt';
experiments{23}.file='lfj3_step_1_100.txt';
experiments{24}.file='lfj3_step_1_120.txt';
experiments{25}.file='lfj3_step_1_140.txt';
experiments{26}.file='lfj3_step_1_150.txt';
experiments{27}.file='lfj3_step_1_160.txt';
experiments{28}.file='lfj3_step_1_170.txt';
experiments{29}.file='lfj3_step_1_180bis.txt';
experiments{30}.file='lfj3_step_1_180.txt';
experiments{31}.file='lfj3_step_1_200.txt';
experiments{32}.file='lfj3_sweep_10.0_100at45deg_0.5_5.0.txt';
experiments{33}.file='lfj3_sweep_10.0_120at45deg_0.5_5.0.txt';
experiments{34}.file='lfj3_sweep_10.0_150at45deg_0.5_5.0.txt';
experiments{35}.file='lfj3_sweep_10.0_180at45deg_0.5_5.0.txt';
experiments{36}.file='lfj3_sweep_30.0_120at45deg_0.5_5.0.txt';
for i=1:length(experiments)
  experiments{i}.data=load(strcat(path,experiments{i}.file));
end

% for i=1:length(experiments)
% visualizeData(experiments{i}.data,experiments{i}.file);
% keyboard;
% close;
% end

% i=34;
% visualizeData(experiments{i}.data,experiments{i}.file);

%training data
data=experiments{34}.data; data(1:2000,:)=[]; data(2000:end,:)=[];   % data=[data; experiments{5}.data];
                                                                     %data=experiments{5}.data;

%%%%%%%%%%%%%%%%%%%%%%%%%%%

%data to compare simulation
testdata=experiments{34}.data;
dt=1e-3;
t=testdata(1,1):dt:testdata(end,1);
u=interp1(testdata(1,1):mean(diff(testdata(:,1))):testdata(end,1),testdata(:,5),t);
x0=testdata(1,2:3)';
%%%%%%%%%%%%%%%%%%%%%%

X=data(:,2:4);
f=data(:,5);

%%%%CONSTRAINTS%%%%%%%%%%%%
lb=ones(9,1)*1e-3; % lb(3)=0.05;
ub=ones(9,1)*1e3;  %ub(1)=10;

ub(1)=1e-2; %keep stiffness small
ub(3)=0.5; %keep inertia small

%bounds for friction model
lb(4)=100; ub(4)=500;
lb(5)=1e-7; ub(5)=200;
lb(6)=1e-7; ub(6)=50;
lb(7)=1e-7; ub(7)=500;
lb(8)=1e-7; ub(8)=500;
lb(9)=0; ub(9)=0.0004;

%inequality constraints
A=zeros(9,9); b=zeros(9,1);
A(5,5)=-1; A(5,6)=1; %g(2) > g(3)
                     

% S=diag(1./pI);
% S=eye(9);
%equality constraints
% Aeq=zeros(9,9); beq=zeros(9,1);
% Aeq(1,1)=1; beq(1)=1e-3;
% Aeq(3,3)=1; beq(3)=1e-2;

%lb(1:3)=pI(1:3); ub(1:3)=pI(1:3);
%lb(4:end)=pI(4:end); ub(4:end)=pI(4:end);

%pI=ub; pI(4:end)=0;

%S=diag(1./max(abs(pI)));
obj = @(p) objectiveFunction(p,X,f);

    options=gaoptimset('InitialPopulation',pI','UseParallel','always', ...
                       'PopulationSize',500,'Generations',4000,'StallGenLimit',1000,'PlotFcns', ...
                       {@gaplotbestf,@gaplotstopping,@gaplotbestindiv});
    [p,o,exitflag] = ga(obj,9,A,b,[],[],lb,ub,[],options);

    
% options = saoptimset('MaxFunEvals',1e6,'PlotFcns',{@saplotbestf,@saplotbestx});%,'InitialPopulation',pI');
% [p,o,exitflag] = simulannealbnd(obj,pI,lb,ub,options);

[X_s l2]=simulate2OrderSystemMakkar(p,u,t,x0,testdata);

p_LFJ3=p;

%save('./2012-11-23_SysIdent_LFJ3/p_LFJ3','p_LFJ3');