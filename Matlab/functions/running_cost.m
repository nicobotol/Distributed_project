function l = running_cost(p,x_bar, u_bar,S,R)

l = x_bar(:,p)'*S*x_bar(:,p) + u_bar(:,p)'*R*u_bar(:,p);

end