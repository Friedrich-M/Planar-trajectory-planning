clc;
clear;
close all;
h=0.5;  %预测时间步长
t=0:h:7;  %生成时间序列

% 本题不针对算法，而是为了保证得到的轨迹平滑，故预测轨迹点已事先得到
x=[1 4 6 9 11 7 3 1 4 5 3 6 11 13 14];  %路径规划点的横坐标序列
y=[-1 3 4 6 8 11 12 15 14 10 3 2 1 -1 -4]
for i=1:length(x)
    scatter(x(i),y(i),150,'black');  %路径规划给出的点
    hold on
end

s=zeros(5);  %待插值序列初始化（可反复利用）
f0x=0;  %x序列边界条件
f0y=0;  
len1=length(t)-5; %距目标点有5个以上的路径规划点时

for i=1:len1  
    %对x序列进行规划
   [f0x, xx, tt] = planner(f0x,h,t,x,i);
   %对y序列进行规划
   [f0y, yy, tt] = planner(f0y,h,t,y,i);
   plot(xx,yy,'LineWidth',2)  %画出规划后的轨迹（X-Y）
   grid on
   hold on
end  %循环结束

%需要规划的点仅剩5个时
[xx, tt] = planner5(f0x,h,t,x,len1);
[yy, tt] = planner5(f0y,h,t,y,len1);
plot(xx,yy,'LineWidth',2)  %画出规划后的轨迹（X-Y）
grid on
title("\fontsize{15}轮式机器人轨迹图")
xlabel("X")
ylabel("Y")

function [f0xx, xx, tt] = planner(f0x,h,t,x,i)
    %对x序列进行规划
    for j=1:5
        s(j)=x(i+j-1);  %待插值序列
    end
    m=tricurveonce(s,h,f0x);  %调用函数得到三弯矩法插值系数M向量
    tt=linspace(t(i),t(i+1),1e5);
    xx=zeros(length(tt),1);  
    for k=1:length(tt)
       xx(k)=(m(1)*(t(i+1)-tt(k))^3+m(2)*(tt(k)-t(i))^3)/6/h+(x(i)-m(1)*h*h/6)*(t(i+1)-tt(k))/h+(x(i+1)-m(2)*h*h/6)*(tt(k)-t(i))/h;
    end
    %一阶导数连续
    f0xx=(-3*m(1)*(t(i+1)-tt(length(tt)))^2+3*m(2)*(tt(length(tt))-t(i))^2)/6/h-(x(i)-m(1)*h*h/6)/h+(x(i+1)-m(2)*h*h/6)/h;
end

function [xx,tt] = planner5(f0x,h,t,x,len1)
for j=1:5
    ss(j)=x(len1+j);  %生成待插值序列
end
m=tricurveonce(ss,h,f0x);  %调用函数得到三弯矩法插值系数M向量
tt=[t(length(t)-4):(h/100):t(length(t))];  %生成时间序列
xx=zeros(length(tt),1);  %生成待plot轨迹横坐标值序列
for i=1:4  %分段计算
    for k=1:100
       xx(100*(i-1)+k)=(m(i)*(t(i+length(t)-4)-tt(100*(i-1)+k))^3+m(i+1)*(tt(100*(i-1)+k)-t(i+length(t)-5))^3)/6/h+(x(i+length(t)-5)-m(i)*h*h/6)*(t(i+length(t)-4)-tt(100*(i-1)+k))/h+(x(i+length(t)-4)-m(i+1)*h*h/6)*(tt(100*(i-1)+k)-t(i+length(t)-5))/h;
    end  %根据公式计算每一时刻对应x的值
end
xx(length(tt))=xx(length(tt)-1);
end