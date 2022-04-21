function m=tricurveonce(s,h,f0)   
%输入参数为s路径点，h为时间间隔，f0为第一点的导数值
u=0.5 % μ=0.5
lambda=0.5 % λ=0.5
g=zeros(5,1)%初始化g向量 
n=2*eye(5)%初始化系数矩阵
for i=2:4
    n(i,i-1)=u  %系数矩阵μ的设置
    n(i,i+1)=lambda  %系数矩阵λ的设置
end
%边界条件处理
n(1,2)=1  % λ1=1 边界条件（1）
n(5,4)=1  % μn=1 边界条件（2）

nn=zeros(5,1)
nn(1)=f0  
for i=2:5
    nn(i)=(s(i)-s(i-1))/h  
end
g(1)=6*(nn(2)-nn(1))/h  % g0
for i=2:4
    g(i)=3*(nn(i+1)-nn(i))/h  % g(i)计算
end  
g(5)=-6*nn(5)/h  
% m=n\g  
m=thomas(n,g)  %调用Thomas法函数得到方程的解
end   %函数结束，返回方程的解
