function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    n = n_order;
    n_a = n*(n-1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = start_cond';
    s = ts(1);
    k = 0;
    Aeq_start(k+1, 1)   = 1 * s^(1-k);
    k = 1;
    Aeq_start(k+1, 1:2) = [-1, 1] * n * s^(1-k);
    k = 2;
    Aeq_start(k+1, 1:3) = [1, -2, 1] * n_a * s^(1-k);
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    beq_end = end_cond';
    s = ts(end);
    k = 0;
    Aeq_end(k+1, end)       = 1 * s^(1-k);
    k = 1;
    Aeq_end(k+1, end-1:end) = [-1, 1] * n * s^(1-k);
    k = 2;
    Aeq_end(k+1, end-2:end) = [1, -2, 1] * n_a * s^(1-k);
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    for i = 1 : n_seg-1
        s1 = ts(i);
        s2 = ts(i+1);
        Aeq_con_p(i, i*(n+1))   = s1;
        Aeq_con_p(i, i*(n+1)+1) = -s2;
    end

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    for i = 1 : n_seg-1
        Aeq_con_v(i, i*(n+1)-1:i*(n+1)+2) = [-n, n, n, -n]; 
    end

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    for i = 1 : n_seg-1
        s1 = ts(i);
        s2 = ts(i+1);
        Aeq_con_a(i, i*(n+1)-2:i*(n+1)+3) = [1/s1, -2/s1, 1/s1, -1/s2, 2/s2, -1/s2] * n_a;
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end