function [B,R,Radj]=leastm_lab(X,Y)
% [B,R,Radj]=leastm_lab(X,Y)
% X    = input data matrix
% Y    = output data
% B  = regression coefficients
% R = coefficient of determination
% Radj = adjusted coefficient opf determination

% least squares regression - matrix method
[n,k]=size(X);
B=(X'*X)\X'*Y;

% correlation coefficients
Yhat=X*B;
Yhat_bar = (1/n)*sum(Yhat);
Ybar     = (1/n)*sum(Y);
Xb=Yhat - Yhat_bar;
Yb=Y    - Ybar;
R=sqrt(((sum(Xb.*Yb))^2)/(sum(Xb.^2)*sum(Yb.^2)));
Radj=sqrt(1-((n-1)/(n-k))*(1-R^2));
