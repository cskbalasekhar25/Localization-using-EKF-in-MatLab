%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%   Measurement                             %
%                                           %
%                                           %
%                 Created by Balasekhar CSK %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function   [P,x,ChiStat]=measurement(x_a,P_a,land,b,range,R)

z=[range, b]';
d=(land(1)-x_a(1,1))^2+(land(2)-x_a(2,1))^2;

%Predicted measurements using a priori states
za=[d^0.5;
    wrapToPi(atan2((land(2)-x_a(2,1)),(land(1)-x_a(1,1))))-x_a(3,1)];
% Jacobian matrix: linearization of the measurement model
    % Dimensions: m x n: m - number of measurements / n - number of states
 H=[-(land(1)-x_a(1,1))/d^0.5 -(land(2)-x_a(2,1))/d^0.5 0;
    (land(2)-x_a(2,1))/d      -(land(1)-x_a(1,1))/d     -1];

% Innovation covariance 
S = H*P_a*H'+R;  

%Information matrix of innovations
Yzz = S^(-1);

% Calculation of Kalman gain
K = P_a*H'*Yzz;

%Innovations
dz = z-za;

% State update and Error covariance update
x = x_a + K*dz;
P = P_a - (K * H * P_a);
ChiStat = dz'*Yzz*dz;

end