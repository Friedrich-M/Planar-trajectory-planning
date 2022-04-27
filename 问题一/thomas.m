function x=thomas(n,g) 
    %输入参数为矩阵方程的系数矩阵A和g矩阵
    for i=1:4
        lambda(i)=n(i,i+1);
        mu(i)=n(i+1, i);
    end
    b=zeros(4,1); %β向量初始化
    b(1)=lambda(1)/2; 
    for i=2:4
        b(i)=lambda(i)/(2-mu(i)*b(i-1)); %β向量计算
    end

    %追的过程
    y=zeros(5,1); %y向量初始化
    y(1)=g(1)/2;
    for i=2:5
        y(i)=(g(i)-mu(i-1)*y(i-1))/(2-mu(i-1)*b(i-1));
    end 

    x=zeros(5,1); %解向量初始化
    x(5)=y(5); %解向量计算，起始点Mn赋值
    for i=4:-1:1
        x(i)=y(i)-b(i)*x(i+1); %向前代入，解向量计算
    end %赶的过程
end 

