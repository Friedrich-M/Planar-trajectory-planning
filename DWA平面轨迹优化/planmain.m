clc;
clear;
close all;
h=0.5;  %Ԥ��ʱ�䲽��
t=0:h:7;  %����ʱ������

% ���ⲻ����㷨������Ϊ�˱�֤�õ��Ĺ켣ƽ������Ԥ��켣�������ȵõ�
x=[1 4 6 9 11 7 3 1 4 5 3 6 11 13 14];  %·���滮��ĺ���������
y=[-1 3 4 6 8 11 12 15 14 10 3 2 1 -1 -4]
for i=1:length(x)
    scatter(x(i),y(i),150,'black');  %·���滮�����ĵ�
    hold on
end

s=zeros(5);  %����ֵ���г�ʼ�����ɷ������ã�
f0x=0;  %x���б߽�����
f0y=0;  
len1=length(t)-5; %��Ŀ�����5�����ϵ�·���滮��ʱ

for i=1:len1  
    %��x���н��й滮
   [f0x, xx, tt] = planner(f0x,h,t,x,i);
   %��y���н��й滮
   [f0y, yy, tt] = planner(f0y,h,t,y,i);
   plot(xx,yy,'LineWidth',2)  %�����滮��Ĺ켣��X-Y��
   grid on
   hold on
end  %ѭ������

%��Ҫ�滮�ĵ��ʣ5��ʱ
[xx, tt] = planner5(f0x,h,t,x,len1);
[yy, tt] = planner5(f0y,h,t,y,len1);
plot(xx,yy,'LineWidth',2)  %�����滮��Ĺ켣��X-Y��
grid on
title("\fontsize{15}��ʽ�����˹켣ͼ")
xlabel("X")
ylabel("Y")

function [f0xx, xx, tt] = planner(f0x,h,t,x,i)
    %��x���н��й滮
    for j=1:5
        s(j)=x(i+j-1);  %����ֵ����
    end
    m=tricurveonce(s,h,f0x);  %���ú����õ�����ط���ֵϵ��M����
    tt=linspace(t(i),t(i+1),1e5);
    xx=zeros(length(tt),1);  
    for k=1:length(tt)
       xx(k)=(m(1)*(t(i+1)-tt(k))^3+m(2)*(tt(k)-t(i))^3)/6/h+(x(i)-m(1)*h*h/6)*(t(i+1)-tt(k))/h+(x(i+1)-m(2)*h*h/6)*(tt(k)-t(i))/h;
    end
    %һ�׵�������
    f0xx=(-3*m(1)*(t(i+1)-tt(length(tt)))^2+3*m(2)*(tt(length(tt))-t(i))^2)/6/h-(x(i)-m(1)*h*h/6)/h+(x(i+1)-m(2)*h*h/6)/h;
end

function [xx,tt] = planner5(f0x,h,t,x,len1)
for j=1:5
    ss(j)=x(len1+j);  %���ɴ���ֵ����
end
m=tricurveonce(ss,h,f0x);  %���ú����õ�����ط���ֵϵ��M����
tt=[t(length(t)-4):(h/100):t(length(t))];  %����ʱ������
xx=zeros(length(tt),1);  %���ɴ�plot�켣������ֵ����
for i=1:4  %�ֶμ���
    for k=1:100
       xx(100*(i-1)+k)=(m(i)*(t(i+length(t)-4)-tt(100*(i-1)+k))^3+m(i+1)*(tt(100*(i-1)+k)-t(i+length(t)-5))^3)/6/h+(x(i+length(t)-5)-m(i)*h*h/6)*(t(i+length(t)-4)-tt(100*(i-1)+k))/h+(x(i+length(t)-4)-m(i+1)*h*h/6)*(tt(100*(i-1)+k)-t(i+length(t)-5))/h;
    end  %���ݹ�ʽ����ÿһʱ�̶�Ӧx��ֵ
end
xx(length(tt))=xx(length(tt)-1);
end