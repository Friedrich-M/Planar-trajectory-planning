function y=planone(sx,sy,gx,gy,theta)
%�������Ϊ��ʼ�����ꡢĿ������ꡢ��ʼ�㷽λ
scatter(sx,sy);  %������ʼ��
hold on
scatter(gx,gy);  %����Ŀ���
hold on
a=zeros(3,2);
a(3,2)=1;  %��ʼ���������˶�ģ�͵�ϵ������
while abs(gx-sx)>0.05||abs(gy-sy)>0.05||abs(theta)>0.05  %�����
    [v,w]=planonce(sx,sy,gx,gy,theta);  %�滮һ�εõ��ٶȿ���ָ��
    a(1,1)=cos(theta);
    a(2,1)=sin(theta);  %���»������˶�ģ�͵�ϵ������
    b=a*[v;w];  %�õ�x��y����λ�ǵı仯������
    sx=sx+b(1)*0.01;  %����x����
    sy=sy+b(2)*0.01  %����y����
    theta=theta+b(3)*0.01  %���·�λ��
    scatter(sx,sy)  %������ǰ��
    hold on
end  %����Ŀ��㸽�����滮����
end
