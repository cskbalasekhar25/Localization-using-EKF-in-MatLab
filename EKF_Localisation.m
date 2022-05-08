%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%   EKF_Localisation                        %
%                                           %
%                                           %
%                 Created by Balasekhar CSK %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Performs sensor fusion of Odometry and Lidar values

%Loading control inputs and Lidar measurements
clear variables
load my_input.mat
load my_measurements.mat


%Initialising the values with given measurements of variance of sensors
t=t';
dt = t(2)-t(1);
v=v';
om=om';

sigma_velocity=0.01;
sigma_angular_velocity=0.25;
sigma_range_13=0.01;
sigma_range_46=0.09;
sigma_bearing=0.25;

%Array for storing pose of robot at every timestep
x = zeros(3,length(t));

%Array for storing error covariance of robot at every timestep
P = zeros(size(x,1),size(x,1),length(t));
%Initialising the first error covariance
P(:,:,1)=diag([1 1 0.1]);

%Initialising the sensor measurement matrix and control input matrix
Q=diag([sigma_velocity sigma_angular_velocity]);
R_13=diag([sigma_range_13 sigma_bearing]);
R_46=diag([sigma_range_46 sigma_bearing]);

ChiStat = zeros(1,length(t));

%Looping through all the timesteps
for k = 2:length(t)

    %Jacobian of state equation with respect to state veriables
    G=[1  0  -dt*v(k-1)*sin(x(3,k-1));
       0  1  dt*v(k-1)*cos(x(3,k-1));
       0  0  1];

    %Jacobian of state equation with respect to control input
    V=[dt*cos(x(3,k-1))  0;
       dt*sin(x(3,k-1))  0;
       0                 dt];

    %Control inputs
    Input=[v(k-1);
           om(k-1)];
    
    %Estimation of state variable from previous time step
    %Prediction step
     xa = x(:,k-1) + V*Input;

     %Predicted error covariance
    
     Pa = G*P(:,:,k-1)*G'+V*Q*V';

     %Performing measurement and correction step for all the six landmarks
     for i=1:length(l)

         if i<4

         [Pa,xa,ChiStat(1,k)]=measurement(xa,Pa,l(i,:),b(k-1,i),r(k-1,i),R_13);

         else

          [Pa,xa,ChiStat(1,k)]=measurement(xa,Pa,l(i,:),b(k-1,i),r(k-1,i),R_46);
         end
         P(:,:,k)=Pa;
         x(:,k)=xa;
     end
end

%figure for Plotting the values

plot(x(1,:),x(2,:));
hold on;
scatter(l(:,1),l(:,2));
    

 
