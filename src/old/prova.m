clear 
clc
x1 = [ 1 1 1]';
X = [x1] + mvnrnd([0 0 0]', eye(3), 100)';
Y = 4*[x1] + mvnrnd([0 0 0]', eye(3), 1e2)';
cov(X, Y)
cv = cov(X(:,1), Y(:, 1));
cor = cv./(std(X(:,1)).*std(Y(:,1)));
corr(X(:,1), Y(:, 1))
corr(X(:,2), Y(:, 2))
corr(X, X)
corr(X, Y)