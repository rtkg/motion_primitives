function p=fit2orderSystem(X,f,del_s)
% X     ... columns holding q,dq,ddq respectively
% f     ... control input
% del_s ... number of samples giving the delay between f and X
% p     ... [k, d, J]

f(1:del_s)=[];
X(end-del_s+1:end,:)=[];

QPO_options=qpOASES_options('maxIter',10000,'epsRegularisation',1e-8 ,'printLevel',0);

H=X'*X;
g=(-f'*X)';
ub=ones(3,1)*Inf;
lb=ones(3,1)*1e-1; lb(1)=1e-3;

[T,p,fval,exitflag,iter,lambda] = evalc('qpOASES(H,g,lb,ub,[],QPO_options)');

if (exitflag ~= 0)
    if (exitflag == 1)
        disp('Warning maximum number of iterations exceeded in QP');
    else
        error('No valid solution for QP found');
    end
end
    

