n_order = 7;
T = 0;
for k = 0 : 3 % p, v, a, j
        for i = k : n_order % i >= k
            Aeq1(k+1, i+1) = factorial(i) * (T^(i-k)) / factorial(i-k);
        end
end
Aeq2(1:4,1:8) = Coeff(0);
A = Aeq2 - Aeq1;

function coeff = Coeff(t)
% Get the coefficient for Aeq
    coeff = [1,  1*t,  1*t^2,  1*t^3,  1*t^4,  1*t^5,  1*t^6,  1*t^7;
             0,  1,    2*t,    3*t^2,  4*t^3,  5*t^4,  6*t^5,  7*t^6;
             0,  0,    2,      6*t,    12*t^2, 20*t^3, 30*t^4, 42*t^5;
             0,  0,    0,      6,      24*t,   60*t^2, 120*t^3,210*t^4];
end