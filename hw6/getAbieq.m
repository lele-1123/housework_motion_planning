function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    n = n_order;
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p_max = [];
    bieq_p_max = zeros(n_all_poly, 1);
    for i = 1 : n_seg
        s = ts(i);
        Aieq_p_max_i = eye(n_order+1) * s;
        Aieq_p_max = blkdiag(Aieq_p_max, Aieq_p_max_i);
        for j = 1 : n+1
            bieq_p_max((i-1)*(n+1)+j) = corridor_range(i, 2);
        end
    end
    Aieq_p_min = [];
    bieq_p_min = zeros(n_all_poly, 1);
    for i = 1 : n_seg
        s = ts(i);
        Aieq_p_min_i = -eye(n_order+1) * s;
        Aieq_p_min = blkdiag(Aieq_p_min, Aieq_p_min_i);
        for j = 1 : n+1
            bieq_p_min((i-1)*(n+1)+j) = -corridor_range(i, 1);
        end
    end
    Aieq_p = [Aieq_p_max; Aieq_p_min];
    bieq_p = [bieq_p_max; bieq_p_min];

    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v_max = [];
    for i = 1 : n_seg
        A_i = -eye(n+1) + diag(ones(n,1), 1);
        A_i(end,:) = [];
        A_i = n * A_i;
        Aieq_v_max = blkdiag(Aieq_v_max, A_i);
    end
    % bieq_v_max = zeros(n_seg*n_order, 1);
    % for i = 1 : n_seg
    %     for j = 1 : n
    %         bieq_v_max((i-1)*n+j) = v_max;
    %     end
    % end
    bieq_v_max = v_max * ones(n_seg*n_order, 1);

    % Aieq_v_min = [];
    % for i = 1 : n_seg
    %     A_i = -eye(n+1) + diag(ones(n,1), 1);
    %     A_i(end,:) = [];
    %     A_i = -n * A_i;
    %     Aieq_v_min = blkdiag(Aieq_v_min, A_i);
    % end
    % bieq_v_min = zeros(n_seg*n_order, 1);
    % for i = 1 : n_seg
    %     for j = 1 : n
    %         bieq_v_min((i-1)*n+j) = v_max;
    %     end
    % end
    Aieq_v_min = - Aieq_v_max;
    bieq_v_min = bieq_v_max;
    Aieq_v = [Aieq_v_max; Aieq_v_min];
    bieq_v = [bieq_v_max; bieq_v_min];

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a_max = [];
    for i = 1 : n_seg
        s = ts(i);
        A_i = eye(n+1) - 2 * diag(ones(n,1), 1) + diag(ones(n-1,1), 2);
        A_i(end-1:end,:) = [];
        A_i = n * (n - 1) / s * A_i;
        Aieq_a_max = blkdiag(Aieq_a_max, A_i);
    end
    % bieq_a_max = zeros(n_seg*(n_order-1), 1);
    % for i = 1 : n_seg
    %     for j = 1 : n-1
    %         bieq_a_max((i-1)*(n-1)+j) = a_max;
    %     end
    % end
    bieq_a_max = a_max * ones(n_seg*(n_order-1), 1);

    % Aieq_a_min = [];
    % for i = 1 : n_seg
    %     s = ts(i);
    %     A_i = eye(n+1) - 2 * diag(ones(n,1), 1) + diag(ones(n-1,1), 2);
    %     A_i(end-1:end,:) = [];
    %     A_i = -n * (n - 1) / s * A_i;
    %     Aieq_a_min = blkdiag(Aieq_a_min, A_i);
    % end
    % bieq_a_min = zeros(n_seg*(n_order-1), 1);
    % for i = 1 : n_seg
    %     for j = 1 : n-1
    %         bieq_a_min((i-1)*(n-1)+j) = a_max;
    %     end
    % end
    Aieq_a_min = - Aieq_a_max;
    bieq_a_min = bieq_a_max;
    Aieq_a = [Aieq_a_max; Aieq_a_min];
    bieq_a = [bieq_a_max; bieq_a_min];
    
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    % Aieq = Aieq_p;
    % bieq = bieq_p;
    % 
    % Aieq = [Aieq_p; Aieq_v_max; Aieq_a_max];
    % bieq = [bieq_p; bieq_v_max; bieq_a_max];
end