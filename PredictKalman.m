function [Xhat,Phat]=PredictKalman(X,P,dt)
%Prediction Step of Kalman filter for Pedestrian Tracking
%--------
%Inputs
%--------
%States:
%                          |  x  |        
%  X=[x1 x2 x3...xn]    xi=|x^dot|      (ie. 4xn matrix)
%                          |  y  |
%                          |y^dot|
%
%Covariances:
% P=[p1 p2 p3...pn] (ie. 4x4xn matrix)
% 
% Time Step:
% dt scalar in seconds
%
%--------
%Outputs
%--------
%Predicted States:       
%  X_hat in same structure as X
%
%Predicted Covariances:
% P_hat in same structure as P
% 

n=size(X,2);

F=[1 dt 0  0;...
   0  1 0  0;...
   0  0 1 dt;...
   0  0 0  1];

Q=[dt^4/4  dt^3/2   0       0   ;...
   dt^3/2  dt^2     0       0   ;...
     0       0    dt^4/4  dt^3/2;...
     0       0    dt^3/2  dt^2  ]*0.7;

Xhat=F*X;
Phat=zeros(4,4,n);
for i=1:n
    Phat(:,:,i)=F*P(:,:,i)*transpose(F)+Q;
end

end



 
 