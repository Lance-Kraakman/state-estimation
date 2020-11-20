function [X, P] = Update(X, P, X_m, R, C, A, Q)
    Yk = C*X_m; 
    % solve 9.113 
    P = Q + A*P*A' - (A*P*C')/(1+C*P*C')*(C*P*C');
    Lk = A*(P*C'/(C*P*C' + R));
    Inn = Yk - C*X;
    X = X + Lk*Inn;
    P = P - Lk*C*P;
end
