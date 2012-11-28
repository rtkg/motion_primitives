function o=objectiveFunction(p,X,f)

nS=length(f);
B=zeros(nS,9);
p=p(:);

B(:,1)=X(:,1)-mean(X(:,1));
B(:,2:3)=X(:,2:3);
B(:,4)=tanh(p(5)*X(:,2))-tanh(p(6)*X(:,2));
B(:,7)=tanh(p(8)*X(:,2));
B(:,9)=X(:,2);

o=0.5*(B*p-f)'*(B*p-f);

%o=max(abs(B*p-f));

