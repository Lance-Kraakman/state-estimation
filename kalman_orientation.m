%% get data and format it to correct units
format long g

%read the data
A_TABLE = readtable('example.xls');
data_string = A_TABLE{:,1:9};
data_string = [data_string A_TABLE{:,11:13}];
data_input = str2double(data_string);

ACCELL_SENSOR_RANGE = 2;
GYRO_RANGE = 500;

%%%% Manually calculated offsets  %%%%%%
x_rot_off = -0.749454040527344;
y_rot_off = -0.552037048339844;
z_rot_off = -0.12807373046875;

x_acc_off = -0.1843;
y_acc_off = -0.507072682617188;
z_acc_off = 0.28595;

%seperate data into variables
X_Accel = data_input(:,1); Y_Accel = data_input(:,2);
Z_Accel = data_input(:,3); Temp = data_input(:,4);
X_ROT = data_input(:,5); Y_ROT = data_input(:,6); 
Z_ROT = data_input(:,7); RSSI = data_input(:,8);
Count = data_input(:,9); Time = A_TABLE{:,10:10};
X_MAG = data_input(:,10);Y_MAG = data_input(:,11);
Z_MAG = data_input(:,12);

n = size(data_input,1); 

%convert accelleration to m/s^2
X_Accel = X_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81);
Y_Accel = Y_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81 );
Z_Accel = Z_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81);

%convert to make sense
X_ROT = X_ROT.*(1/2^15)*(GYRO_RANGE);
Y_ROT = Y_ROT.*(1/2^15)*(GYRO_RANGE);
Z_ROT = Z_ROT.*(1/2^15)*(GYRO_RANGE);
Temp = Temp.*(1/340) + 36.53;

%%%% Manually calculated offsets  %%%%%%
%offsets gyro
X_ROT = X_ROT + ones(length(X_ROT),1).*x_rot_off;
Y_ROT = Y_ROT + ones(length(Y_ROT),1).*y_rot_off;
Z_ROT = Z_ROT + ones(length(Z_ROT),1).*z_rot_off;

%offsets acell 
X_Accel = X_Accel + ones(length(X_Accel),1).*x_acc_off;
Y_Accel = Y_Accel + ones(length(Y_Accel),1).*y_acc_off;
Z_Accel = Z_Accel + ones(length(Z_Accel),1).*z_acc_off;
%%
final_data = [X_Accel, Y_Accel, Z_Accel,X_ROT, Y_ROT, Z_ROT, RSSI, Temp, X_MAG, Y_MAG, Z_MAG];

%%% Covariance matrix for kalman filter

final_data = [X_Accel, Y_Accel, Z_Accel,X_ROT, Y_ROT, Z_ROT, RSSI, Temp, X_MAG, Y_MAG, Z_MAG];

N = length(final_data);
unity = ones(N,N);
%covariance matrix of collected data
deviation_scores = final_data - (unity)*final_data.*(1/N);
cov = deviation_scores'*deviation_scores.*(1/N);
Covariance_Table = array2table(cov,'RowNames',...
    {'X_Accel', 'Y_Accel', 'Z_Accel','X_ROT', 'Y_ROT', 'Z_ROT', 'RSSI', 'Temp', 'X_MAG', 'Y_MAG', 'Z_MAG'},...
    'VariableNames',...
    {'X_Accel', 'Y_Accel', 'Z_Accel','X_ROT', 'Y_ROT', 'Z_ROT', 'RSSI', 'Temp', 'X_MAG', 'Y_MAG', 'Z_MAG'})

%%


%%%%% kalman filter %%%%%%%%
dt = 0.01; %sample rate/frequency

%states
Xk = zeros(6,1)';         

%observation matrix (C matrix in Y = c*X)
C = eye(6);

%Covariance Matrix
Pk = zeros(6);

%%% System/Plant noise %%%
Q = eye(6); Q(1,1) = 0.003; Q(2,2) = 0.003; Q(3,3) = 0.003; 

%%% Measurement nopise %%%%
R = eye(6)*0.3;

%%% Measurement Matrix %%%
Y = zeros(1, 6);

A = [
    1 0 0 dt 0 0;
    0 1 0 0 dt 0;
    0 0 1 0 0 dt;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];


B = [0 0 0 0 0 0]';

for i = 1:n
        if (i == 1)
            Yk = getMeasurements_1(final_data, i)';
            Xk = [0 0 0 0 0 0]'
        else
            
            Xk_prev = Xk; 
            Xk = A*Xk_prev; % prediction 
            
            prediction(i,:) = Xk;

         
            Yk = getMeasurements_1(final_data, i)';
            Pk = A*Pk*A' + Q; % priori covariance
            Lk = Pk*C'/(C*Pk*C' + R); % compute kalman gain
            Xk = Xk + Lk*(Yk - C*Xk); %update new states
            Pk = (eye(6) - Lk*C)*Pk; % update covariance matrix         
            update_1(i,:) = Yk;
            % we need to add magnometer data to measure yaw (theta z)
            %while doing this we can also provide another measurement for
            %our kalman filter
            %https://www.artekit.eu/resources/ak-mag3110/doc/AN4248.pdf
        end
            X_array(i,:) = Xk; 
            Yk_meas(i,:) = Yk;
end

figure(1)
plot(Time, [X_array(:,1) X_array(:,2) X_array(:,4) X_array(:,5)],'--', 'LineWidth', 1)
hold on
xlim([0 50])
ylim([-360 360])
title('state estimates')
legend({'RX','RY','RX DOT','RY DOT'},'Location','northeast','Orientation','horizontal')


figure(2)
plot(Time, [prediction(:,2) Yk_meas(:,2) X_array(:,2)],'--', 'LineWidth', 1);
hold on;
legend({'Y prediction','y measured','Y final rot'},'Location','northeast','Orientation','horizontal')
title("Y Prediction, update and final")
xlabel("Time (s)")
ylabel("Angle Rotated")
xlim([0 50])
ylim([-360 360])

figure(3)
plot(Time, [prediction(:,1) X_array(:,1)]);
hold on;
legend({'X prediction','X gyro rot','X final rot'},'Location','northeast','Orientation','horizontal')
title("X Prediction, update and final")
xlabel("Time (s)")
ylabel("Angle Rotated")
xlim([0 50])
ylim([-360 360])



%get measurements for the states we can measure
function [U] = getMeasurements_1(data, i)
    u = 0.01;
    Ax = data(i,1);
    Ay = data(i,2);
    Az = data(i,3);
    
    U(1) = atand(Ay/(sign(Az)*sqrt(Az^2 + u*(Ax^2)))); 
    U(2) = atan2d(-Ax,Az); %pitch
    U(3) = atand(Ay/sqrt(Ax^2 + Az^2));
    U(4) = data(i,4); 
    U(5) = data(i,5);
    U(6) = data(i,6);
end

function [U] = getMeasurements_2(data, i, X, dt)
  
    %Theta's
    U(1) = X(1) + dt*data(i,4);
    U(2) = X(2) + dt*data(i,5); 
    U(3) = X(3) + dt*data(i,6); 
    % theta dots
    U(4) = data(i,4); 
    U(5) = data(i,5);
    U(6) = data(i,6);
end

