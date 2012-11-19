function [p fval]=fit2OrderSystem(X,f,lb,ub)
% X ... columns holding q,dq,ddq respectively
% f ... control input
% p ... [k, d, J]

QPO_options=qpOASES_options('maxIter',10000,'epsRegularisation',1e-8 ,'printLevel',0);

H=X'*X;
g=(-f'*X)';


[T,p,fval,exitflag,iter,lambda] = evalc('qpOASES(H,g,lb(1:3),ub(1:3),[],QPO_options)');

if (exitflag ~= 0)
    if (exitflag == 1)
        disp('Warning maximum number of iterations exceeded in QP');
    else
        error('No valid solution for QP found');
    end
end
