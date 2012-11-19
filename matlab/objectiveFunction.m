function o=objectiveFunction(p,X,f)

nS=length(f);
B=zeros(nS,9);
p=p(:);

B(:,1:3)=X;
B(:,4)=tanh(p(4)*X(:,2))-tanh(p(5)*X(:,2));
B(:,7)=tanh(p(7)*X(:,2));
B(:,9)=X(:,2);

o=0.5*(B*p-f)'*(B*p-f);




