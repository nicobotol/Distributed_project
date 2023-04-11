function [w_bar, K, Q_uu] = backward_pass(x_bar,u_bar, mu, S, Sf, R, T, target)
 % algorithm to find optimum local input w and gain K
 
    n = length(target);
    V_x = zeros(n, T);
    V_xx = cell(n, T);
    V_x(:,T) = (target'*(Sf+Sf'))';
    V_xx{T} = Sf+Sf';
    Q_uu = cell(1,T);
    K = cell(1,T);
    w_bar = zeros(n,T);

    for i=T:-1:1
     l_x = x_bar(:,i)'*(S+S');
     l_u = u_bar(:,i)'*(R+R');
     l_xx = S+S';
     l_uu = R+R';
     l_xu = zeros(n,n);

     % Linearized model
     A_i = eye(4);
     A_i(:,4) = [-u_bar(1,i)*sin(x_bar(4,i)*dt+u_bar(2,i)*cos(x_bar(4,i))*dt);
                u_bar(1,i)*cos(x_bar(4,i))*dt-u_bar(2,i)*sin(x_bar(4,i))*dt;
                0;
                1];
     B_t = ; % derivative of dynamic wrt inputs
    
     Q_x = l_x + A_i'*V_x(:,i+1);
     Q_u = l_u + f_u'*V_x(:,i+1);
     Q_xx = l_xx + A_i'*V_xx(:, i+1)*A_i;
     Q_uu{i} = l_uu + f_u'*V_xx(:, i+1)*f_u;
     Q_xu = l_xu + A_i'*V_xx(:, i+1)*f_u;
    
     Q_bar = Q_uu + mu*eye(n);
     w_bar(:,i) = -inv(Q_bar)*Q_u;
     K{i} = -inv(Q_bar)*Q_xu;
    
     V_x(:,i) = Q_x'+Q_u'*K+w_bar'*Q_uu*K+w_bar'*Q_xu';
     V_xx(:,i+1) = Q_xx+K'*Q_uu*K+Q_xu*K+K'*Q_xu';
     
    end

end