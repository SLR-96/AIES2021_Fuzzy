function y=Mutate(x,mu,VarMin,VarMax)

    nVar=numel(x);
    
    nmu=ceil(mu*nVar);
    
    j=randsample(nVar,nmu);
    
    sigma=0.1*(VarMax(j)-VarMin(j));
    
    y=x;
    y(j)=x(j)+sigma.*(randn(size(j)))';
%   y(j) = VarMin(j)+rand(1,nmu).*(VarMax(j)-VarMin(j));
    
    y=max(y,VarMin);
    y=min(y,VarMax);
end