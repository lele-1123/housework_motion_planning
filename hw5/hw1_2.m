clc;clear;close all;
path = ginput() * 100.0;

n_order = 7;
n_seg = size(path, 1) - 1;
n_poly_perseg = n_order + 1;

ts = zeros(n_seg, 1);
% calculate time distribution based on distance between 2 points
dist = zeros(n_seg, 1);
dist_sum = 0;
T = 25;

t_sum = 0;
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum + dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i) / dist_sum * T;
    t_sum = t_sum + ts(i);
end
ts(n_seg) = T - t_sum;
% or you can simply average the time
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 4: get the coefficients of i-th segment of both x-axis and y-axis
    Pxi = [];
    Pyi = [];
    Pxi = flipud(poly_coef_x(i*(n_order+1)+1 : i*(n_order+1)+(n_order+1)));
    Pyi = flipud(poly_coef_y(i*(n_order+1)+1 : i*(n_order+1)+(n_order+1)));
    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi,t);
        Y_n(k)  = polyval(Pyi,t);
        k = k+1;
    end
end

plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    % M1 = inv(M);
    % R = C * M1' * Q * M1 * Ct;
    R = C / M' * Q / M * Ct;
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    dF = [];
    dF = start_cond';
    idx1 = size(start_cond, 2);
    for i = 1 : n_seg-1
        dF(idx1+i) = waypoints(i+1);
    end
    idx2 = size(dF, 1);
    for i = 1 : idx1
        dF(idx2+i) = end_cond(i);
    end
    % save data;
    dP = - R_pp \ R_fp' * dF;
    poly_coef = M \ Ct * [dF;dP];
end