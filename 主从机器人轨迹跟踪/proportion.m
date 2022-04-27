function v_w_mat=proportion(sx,sy,gx,gy,theta)
    K=0.1;
    l=0.2;
    %求出期望位置与当前位置的偏差
    e_x=sx-gx;
    e_y=sy-gy;
    
    u_x=-K*e_x;
    u_y=-K*e_y;
    
    A_mat=[cos(theta) -l*sin(theta);
    sin(theta) l*cos(theta)];

    P=[cos(theta) 0;sin(theta) 0;0 1];
    U_mat=[u_x,u_y]';
    v_w_mat = A_mat\U_mat;
end

