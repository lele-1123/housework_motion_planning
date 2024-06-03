function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        for i = 4 : n_order
            for j = 4 : n_order
                Q_k(i+1, j+1) = (factorial(i)*factorial(j)) / (factorial(i-4)*factorial(j-4)*(i+j-7)) * (ts(k)^(i+j-7));
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end