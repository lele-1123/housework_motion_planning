function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 1.1 calculate Q_k of the k-th segment 
        Q_k = [];
        t_k = 1;
        s_k = ts(k);
        for i = 4 : n_order
            for j = 4 : n_order
                % Q_k(i+1, j+1) = (factorial(i)*factorial(j)) / (factorial(i-4)*factorial(j-4)*(i+j-7)) * (t_k^(i+j-7)) / s_k^(2*4-3);
                den = i+j-7;
                Q_k(i+1, j+1) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/den*(t_k^den)/(s_k^(2*4-3));
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end