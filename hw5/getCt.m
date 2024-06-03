function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    n = (n_order + 1) / 2;
    Ct = zeros(2*n*n_seg, n*(n_seg+1));
    % 起点状态:p, v, a, j0
    Ct(1:n,1:n) = eye(n);
    % 中间点位置:p1, p2, ..., p(n-1)
    for i = 1 : n_seg-1
        Ct(n*2*i-3, n+i) = 1;
        Ct(n*2*i+1, n+i) = 1;
    end
    % 终点状态：pn, vn, an, jn
    Ct(end-n+1:end, n+n_seg:2*n+n_seg-1) = eye(n);
    % free derivatives
    % v1, a1, j1, ..., v(n-1), a(n-1), j(n-1)
    idx_df = n * 2 + n_seg - 1;
    for i = 1 : n_seg - 1
        idx = 2 * n * i;
        idx_df2 = idx_df + (i - 1) * 3;
        Ct(idx-4+2:idx-4+4, idx_df2+1:idx_df2+3) = eye(n-1);
        Ct(idx+2:idx+4, idx_df2+1:idx_df2+3) = eye(n-1);
    end
end