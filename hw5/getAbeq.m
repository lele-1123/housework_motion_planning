function [Aeq, beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    % beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    T = 0;
    for k = 0 : 3 % p, v, a, j
        for i = k : n_order % i >= k
            Aeq_start(k+1, i+1) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end
    beq_start = start_cond';

    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    % beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    T = ts(end);
    for k = 0 : 3 % p, v, a, j
        for i = k : n_order % i >= k
            Aeq_end(k+1, end-7+i) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end
    beq_end = end_cond';

    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    % 对每段轨迹的起点进行约束
    for n = 1 : n_seg-1
        T = ts(n);
        for i = 0 : n_order
            Aeq_wp(n, (n-1)*(n_order+1)+i+1) = T^i;
        end
        beq_wp(n, 1) = waypoints(n+1);
    end

    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    k = 0;
    for n = 0 : n_seg-2 % -2:中间点数-1(从0开始)
        T = ts(n+1);
        idx = n * (n_order + 1);
        for i = k : n_order
            Aeq_con_p(n+1,idx+1+i) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
        T = 0;
        idx = (n + 1) * (n_order+1);
        for i = k : n_order
            Aeq_con_p(n+1,idx+1+i) = -factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end

    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    k = 1;
    for n = 0 : n_seg-2
        T = ts(n+1);
        idx = (n) * (n_order + 1);
        for i = k : n_order
            Aeq_con_v(n+1,idx+1+i) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
        T = 0;
        idx = (n+1) * (n_order+1);
        for i = k : n_order
            Aeq_con_v(n+1,idx+1+i) = -factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    k = 2;
    for n = 0 : n_seg-2
        T = ts(n+1);
        idx = (n) * (n_order + 1);
        for i = k : n_order
            Aeq_con_a(n+1,idx+1+i) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
        T = 0;
        idx = (n+1) * (n_order+1);
        for i = k : n_order
            Aeq_con_a(n+1,idx+1+i) = -factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end

    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    k = 3;
    for n = 0 : n_seg-2
        T = ts(n+1);
        idx = (n) * (n_order + 1);
        for i = k : n_order
            Aeq_con_j(n+1,idx+1+i) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
        T = 0;
        idx = (n+1) * (n_order+1);
        for i = k : n_order
            Aeq_con_j(n+1,idx+1+i) = -factorial(i) * (T^(i-k)) / factorial(i-k);
        end
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end
