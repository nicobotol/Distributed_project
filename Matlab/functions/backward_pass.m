function [w_bar, K, Q_u, Q_uu] = backward_pass(x_bar,u_bar, mu, S, Sf, R, T, target, dt)
 % algorithm to find optimum local input w and gain K
 
    n = length(target);
    V_x = zeros(n, T+1);
    V_xx = cell(n, T+1);
    V_x(:,T+1) = (target'*(Sf+Sf'))';
    V_xx{T+1} = Sf+Sf';
    Q_u = cell(1,T);
    Q_uu = cell(1,T);
    w_bar = zeros(n,T);

    for i=T:-1:1
     l_x = (x_bar(:,i)'*(S+S'))';
     l_u = (u_bar(:,i)'*(R+R'))';
     l_xx = S+S';
     l_uu = R+R';
     l_xu = zeros(n,n);

     % Linearized model
     A_i = eye(4);
     A_i(:,4) = [-u_bar(1,i)*sin(x_bar(4,i)*dt-u_bar(2,i)*cos(x_bar(4,i))*dt);
                u_bar(1,i)*cos(x_bar(4,i))*dt-u_bar(2,i)*sin(x_bar(4,i))*dt;
                0;
                1];
     B_i = dt*[cos(x_bar(4,i)) -sin(x_bar(4,i)) 0 0;
               sin(x_bar(4,i)) cos(x_bar(4,i)) 0 0;
               0 0 1 0;
               0 0 0 1]; % derivative of dynamic wrt inputs
    
     Q_x = l_x + A_i'*V_x(:,i+1);
     Q_u{i} = l_u + B_i'*V_x(:,i+1);
     Q_xx = l_xx + A_i'*V_xx{i+1}*A_i;
     Q_uu{i} = l_uu + B_i'*V_xx{i+1}*B_i;
     Q_xu = l_xu + A_i'*V_xx{i+1}*B_i;
    
     Q_bar{i} = Q_uu{i} + mu*eye(n);
     w_bar(:,i) = -inv(Q_bar{i})*Q_u{i};
     K{i} = -inv(Q_bar{i})*Q_xu;
    
     V_x(:,i) = Q_x'+Q_u{i}'*K{i}+w_bar(:,i)'*Q_uu{i}*K{i}+w_bar(:,i)'*Q_xu';
     V_xx{i} = Q_xx+K{i}'*Q_uu{i}*K{i}+Q_xu*K{i}+K{i}'*Q_xu';
     
    end

end