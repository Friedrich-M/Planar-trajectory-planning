function x=thomas(n,g) 
    %�������Ϊ���󷽳̵�ϵ������A��g����
    for i=1:4
        lambda(i)=n(i,i+1);
        mu(i)=n(i+1, i);
    end
    b=zeros(4,1); %��������ʼ��
    b(1)=lambda(1)/2; 
    for i=2:4
        b(i)=lambda(i)/(2-mu(i)*b(i-1)); %����������
    end

    %׷�Ĺ���
    y=zeros(5,1); %y������ʼ��
    y(1)=g(1)/2;
    for i=2:5
        y(i)=(g(i)-mu(i-1)*y(i-1))/(2-mu(i-1)*b(i-1));
    end 

    x=zeros(5,1); %��������ʼ��
    x(5)=y(5); %���������㣬��ʼ��Mn��ֵ
    for i=4:-1:1
        x(i)=y(i)-b(i)*x(i+1); %��ǰ���룬����������
    end %�ϵĹ���
end 

