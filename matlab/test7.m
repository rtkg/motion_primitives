clear all; close all; clc;
%fit 2nd order linear system + Makkar friction with genetic algorithm

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

%number of optimizations
nO=2;

%training data
data=[ffj3_sweep_30_20];
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%data to compare simulation
testdata=ffj3_step_10_5;
dt=1e-3;
t=0:dt:testdata(end,1);
u=interp1(0:mean(diff(testdata(:,1))):testdata(end,1),testdata(:,5),t);
x0=testdata(1,2:3)';
%%%%%%%%%%%%%%%%%%%%%%

X=data(:,2:4);
f=data(:,5);

%%%%CONSTRAINTS%%%%%%%%%%%%
lb=zeros(9,1);
ub=ones(9,1)*1e3;

%keep g_6 = 0
lb(end)=0; ub(end)=0;
%%%%%%%%%%%%%%%%%%%%%%%

% [pI o]=fit2OrderSystem(X,f,lb,ub); %fit 2nd order system only
% pI=[pI; zeros(6,1)];

obj = @(p) objectiveFunction(p,X,f);
 options = gaoptimset('UseParallel','always','PopulationSize',500,'Generations',4000,'StallGenLimit',1000,'PlotFcns',{@gaplotbestf,@gaplotstopping,@gaplotbestindiv});%,'InitialPopulation',pI');
[p,o,exitflag] = ga(obj,9,[],[],[],[],lb,ub,[],options);


[X_s]=simulate2OrderSystemMakkar(p,u,t,x0,testdata);


