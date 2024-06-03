clear all;
close all;
clc;

p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];
K=20;
dt=0.2;

P=[];
V=[];
A=[];

w = 0.5;

for t=0.2:0.2:40
    % Construct the reference signal
    for i = 1:20
        tref = t + i*0.2;
        r=0.25*tref;
        pt(i,1) = r*sin(w*tref);
        vt(i,1) = r*cos(w*tref)*w;
        at(i,1) = -r*sin(w*tref)*w*w;
        
        pt(i,2) = r*cos(w*tref);
        vt(i,2) = -r*sin(w*tref)*w;
        at(i,2) = -r*cos(w*tref)*w*w;
        
        pt(i,3) = 20 - 0.5*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end
    % Do the MPC
    % Please follow the example in linear mpc part to fill in the code here to do the tracking
    j(1) = xy_axis_mpc(K, dt, p_0(1), v_0(1), a_0(1), pt(:,1), vt(:,1), at(:,1));
    j(2) = xy_axis_mpc(K, dt, p_0(2), v_0(2), a_0(2), pt(:,2), vt(:,2), at(:,2));
    j(3) =  z_axis_mpc(K, dt, p_0(3), v_0(3), a_0(3), pt(:,3), vt(:,3), at(:,3));

    for i=1:3
       [p_0(i),v_0(i),a_0(i)] = forward(p_0(i),v_0(i),a_0(i),j(i),dt);
    end
    
    % Log the states
    P = [P;p_0 pt(1,:)];
    V = [V;v_0 vt(1,:)];
    A = [A;a_0 at(1,:)];
end

% 定义目标圆锥螺旋线
% 定义时间向量
t = 0.2:0.2:40;

% 预分配矩阵以提高效率
num_points = length(t);
pt = zeros(num_points, 3);
vt = zeros(num_points, 3);
at = zeros(num_points, 3);

% 循环遍历时间向量
for i = 1:num_points
    tref = t(i);
    r = 0.25 * tref; % 半径随着时间增加
    
    % 计算位置
    pt(i, 1) = r * sin(w * tref);
    pt(i, 2) = r * cos(w * tref);
    pt(i, 3) = 20 - 0.5 * tref; % z轴线性下降
end

% Plot the result
plot(P);
grid on;
legend('x','y','z');
figure;
plot3(pt(:,1),pt(:,2),pt(:,3),'g');
hold on;
plot3(P(:,1),P(:,2),P(:,3),'b');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;