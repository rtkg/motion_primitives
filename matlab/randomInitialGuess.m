function pI=randomInitialGuess(lb,ub)

for i=1:length(ub)
   pI(i)= lb(i) + (ub(i)-lb(i)).*rand(1,1);
end    

pI=pI(:);