function M = getM(n_seg, n_order, ts)
    M = [];
    for n = 1:n_seg
        M_k = zeros(n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        T = 0;
        for k = 0 : 3
            for i = k : n_order
                M_k(k+1, i+1) = factorial(i) / factorial(i-k) * (T^(i-k));
            end
        end
        T = ts(n);
        for k = 0 : 3
            for i = k : n_order
                M_k(4+k+1, i+1) = factorial(i) / factorial(i-k) * (T^(i-k));
            end
        end
        M = blkdiag(M, M_k);
    end
end