function l = running_cost(p,x_bar, u_bar,S,R, target)

l = (x_bar(:,p)-target)'*S*(x_bar(:,p)-target) + u_bar(:,p)'*R*u_bar(:,p);

end