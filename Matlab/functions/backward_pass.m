function [w_bar, K, Q_u, Q_uu] = backward_pass(x_bar,u_bar, mu, S, Sf, R, T, n, target, dt,V)
 % algorithm to find optimum local input w and gain K

    V_x = zeros(n-1, T+1);
    V_xx = cell(n-1, T+1);
    V_x(:,T+1) = ((x_bar(:,end)-target)'*(Sf+Sf'))';
    V_xx{T+1} = Sf+Sf';
    Q_u = cell(1,T);
    Q_uu = cell(1,T);
    w_bar = zeros(1,T);
    K = cell(1,T);
    Q_bar = cell(1,T);

    for i=T:-1:1
         l_x = ((x_bar(:,i)-target)'*(S+S'))';
         l_u = (u_bar(:,i)'*(R+R'))';
         l_xx = S+S';
         l_uu = R+R';
         l_xu = zeros(n-1,1);
     
         % Linearized model
         A_i = eye(3);
         A_i(:,3) = [-V*cos(x_bar(3,i))*dt;
                     -V*sin(x_bar(3,i))*dt;
                    1];       % derivative of dynamic wrt states negletting z (we solve a 2D problem)
         B_i = dt*[0;0;1];    % derivative of dynamic wrt inputs
        
         Q_x = l_x + A_i'*V_x(:,i+1);
         Q_u{i} = l_u + B_i'*V_x(:,i+1);
         Q_xx = l_xx + A_i'*V_xx{i+1}*A_i;
         Q_uu{i} = l_uu + B_i'*V_xx{i+1}*B_i;
%          fprintf('V_xx at %d value:', i)
%          V_xx{i+1}
         Q_xu = l_xu + A_i'*V_xx{i+1}*B_i;
        
         % prendere solo le componenti di vx, vy e omega?
         Q_bar{i} = Q_uu{i}+mu;
         w_bar(:,i) = -inv(Q_bar{i})*Q_u{i};
         K{i} = -inv(Q_bar{i})*Q_xu';

         V_x(:,i) = Q_x'+Q_u{i}'*K{i}+w_bar(:,i)'*Q_uu{i}*K{i}+w_bar(:,i)'*Q_xu';
         V_xx{i} = Q_xx+K{i}'*Q_uu{i}*K{i}+Q_xu*K{i}+K{i}'*Q_xu';
     
    end

end