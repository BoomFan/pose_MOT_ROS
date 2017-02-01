function [Xn,Pn]=UpdateKalman(Xhat,Phat,Z)
%Update Step of Kalman filter for Person Tracking
%Predicted States Inputs:
%
%  Xhat/Phat: values reurned from Predict Kalman function
%
%  Z: measured location values ordered according to Xhat in the form

%                               | x |
%    Z=[z1 z2 z3...zn] where z1=| y | (ie. Matrix of size 2xn)
%

n=size(Z,2);

H=[1 0 0 0;...
   0 0 1 0];

R=[0.05 0     ;...
   0      0.05];

y=Z-H*Xhat;
Xn=zeros(4,n);
Pn=zeros(4,4,n);
for i=1:n
    S=H*Phat(:,:,i)*transpose(H)+R;
    K=Phat(:,:,i)*transpose(H)/S;
    Xn(:,i)=Xhat(:,i)+K*y(:,i);
    Pn(:,:,i)=(eye(4)-K*H)*Phat(:,:,i);
end

end