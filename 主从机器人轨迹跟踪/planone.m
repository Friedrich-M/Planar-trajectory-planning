function y=planone(sx,sy,gx,gy,theta)
%输入参数为起始点坐标、目标点坐标、起始点方位
scatter(sx,sy);  %画出起始点
hold on
scatter(gx,gy);  %画出目标点
hold on
a=zeros(3,2);
a(3,2)=1;  %初始化机器人运动模型的系数矩阵
while abs(gx-sx)>0.05||abs(gy-sy)>0.05||abs(theta)>0.05  %误差限
    [v,w]=planonce(sx,sy,gx,gy,theta);  %规划一次得到速度控制指令
    a(1,1)=cos(theta);
    a(2,1)=sin(theta);  %更新机器人运动模型的系数矩阵
    b=a*[v;w];  %得到x，y，方位角的变化率向量
    sx=sx+b(1)*0.01;  %更新x坐标
    sy=sy+b(2)*0.01  %更新y坐标
    theta=theta+b(3)*0.01  %更新方位角
    scatter(sx,sy)  %画出当前点
    hold on
end  %到达目标点附近，规划结束
end
