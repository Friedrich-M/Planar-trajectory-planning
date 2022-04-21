function m=tricurveonce(s,h,f0)   
%�������Ϊs·���㣬hΪʱ������f0Ϊ��һ��ĵ���ֵ
u=0.5 % ��=0.5
lambda=0.5 % ��=0.5
g=zeros(5,1)%��ʼ��g���� 
n=2*eye(5)%��ʼ��ϵ������
for i=2:4
    n(i,i-1)=u  %ϵ������̵�����
    n(i,i+1)=lambda  %ϵ������˵�����
end
%�߽���������
n(1,2)=1  % ��1=1 �߽�������1��
n(5,4)=1  % ��n=1 �߽�������2��

nn=zeros(5,1)
nn(1)=f0  
for i=2:5
    nn(i)=(s(i)-s(i-1))/h  
end
g(1)=6*(nn(2)-nn(1))/h  % g0
for i=2:4
    g(i)=3*(nn(i+1)-nn(i))/h  % g(i)����
end  
g(5)=-6*nn(5)/h  
% m=n\g  
m=thomas(n,g)  %����Thomas�������õ����̵Ľ�
end   %�������������ط��̵Ľ�
