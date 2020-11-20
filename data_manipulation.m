format long g

%read the data
A = readtable('example.xls');
data_string = A{:,1:9};
data_string = [data_string A{:,11:13}];
data_input = str2double(data_string);

ACCELL_SENSOR_RANGE = 2;
GYRO_RANGE = 550;

%%%% Manually calculated offsets  %%%%%%
x_rot_off = -0.749454040527344;
y_rot_off = -0.552037048339844;
z_rot_off = -0.12807373046875;

x_acc_off = 0.529477538574219;
y_acc_off = -1.1323072682617188;
z_acc_off = 0.921970348144531;

%seperate data into variables
X_Accel = data_input(:,1); Y_Accel = data_input(:,2);
Z_Accel = data_input(:,3); Temp = data_input(:,4);
X_ROT = data_input(:,5); Y_ROT = data_input(:,6); 
Z_ROT = data_input(:,7); RSSI = data_input(:,8);
Count = data_input(:,9); Time = A{:,10:10};
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
%TO_DO
%magnometre calibrastion (to-do)

% %plot the data
% colors = ["red", "green", "blue", "red", "green","blue", "green", "red","blue", "green", "red"];
% input_array = [X_Accel, Y_Accel, Z_Accel,X_ROT, Y_ROT, Z_ROT, RSSI, Temp, X_MAG, Y_MAG, Z_MAG];
% for i = 1:size(input_array, 2)
%     if (i <= 3)
%         figure(1)
%         plot(Time, input_array(:,i), 'Color',colors(i));
%         hold on;
%         legend({'X','Y','Z'},'Location','northeast','Orientation','horizontal')
%         title("Accelleration vs Time")
%         xlabel("Time (s)")
%         ylabel("Accelleration (m/s^2)")
%     elseif (i <=6)
%         figure(2)
%         plot(Time, input_array(:,i), 'Color',colors(i));
%         hold on;
%         legend({'X','Y','Z'},'Location','northeast','Orientation','horizontal')
%         title("Rotational Velocity vs/ Time")
%         xlabel("Time (s)")
%         ylabel("Rotational Velocity (º/s)")
%         xlim([0 10])
%         ylim([-50 50])
%     elseif (i<=8)
%         figure(3)
%         plot(Time, input_array(:,i), 'Color',colors(i));
%         hold on;
%         legend({'RSSI','Temp'},'Location','northeast','Orientation','horizontal')
%         title("")
%         xlabel("Time (s)")
%         ylabel("Temps")
%     else 
%         figure(4)
%         plot(Time, input_array(:,i), 'Color',colors(i));
%         hold on;
%         legend({'x','y','z'},'Location','northeast','Orientation','horizontal')
%         title("")
%         xlabel("Time (s)")
%         ylabel("MAG values")
%     end
% end

%Calculations using the linear model
%first we need to remove the gravity component and calculate the forced
%accellerations http://www.chrobotics.com/library/accel-position-velocity




% %position
% Y_Pos = zeros(length(Y_ROT),1);
% Y_Current = 0; 
% Y_Prev = 0;
% for i = 1:length(v)
%     Y_Current = Y_Prev + interval*v(i);
%     Y_Pos(i) = Y_Current; 
% end

% %velocity
% vx = zeros(length(Y_ROT),1);
% vx_prev = 0; vx_current = 0; 
% interval = 0.01;
% for i = 1:length(Y_ROT)
%     vx_current = vx_prev + interval*Y_ROT(i);
%     vx_prev = vx_current;
%     vx(i) = vx_current; 
% end


% %position
% X_Pos = zeros(length(vx),1);
% X_Current = 0; 
% X_Prev = 0;
% for i = 1:length(vx)
%     X_Current = X_Prev + interval*vx(i);
%     X_Pos(i) = X_Current; 
% end

% figure(5)
% plot(Time, Y_Pos);
% hold on;
% legend({'Y'},'Location','northeast','Orientation','horizontal')
% title("")
% xlabel("Time (s)")
% ylabel("m traveled")

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


%%%%% kalman filter %%%%%%%%
dt = 0.01; %sample rate/frequency

%states
X = zeros(18,1)';         

%observation matrix (C matrix in Y = c*X)
C = eye(18);

%Covariance Matrix
P = eye(18);

%%% System/Plant noise %%%
Q = eye(18)*0.1;

%%% Measurement nopise %%%%
R = eye(18)*0.1;

%%% Measurement Matrix %%%
Y = zeros(1, 18);


%transition matrix
A = [
    1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0 0 0 0;... %[x]
    0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0 0 0;... %[y]
    0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0 0;... %[z]
    0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0;... %[Vx]
    0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0;... %[Vy]
    0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0;... %[Vz]
    0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;... %[Ax]
    0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;... %[Ay]
    0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;... %[Az]
    0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 0 0 0;... %[theta x]
    0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 0 0;... %[theta y]
    0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 0;... %[theta z]
    0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;... %[theta dot x]
    0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;... %[theta dot y]
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;... %[theta dot z]
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;... %[mag x]
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;... %[mag y]
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;... %[mag z]
    ];



for i = 1:n
        if (i == 1)
            %%% initialise the kalman filter %%% 
            X = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -20000 -20000 14000]';
            P = P.*2; 
        else
            X_m = Measure_update_one(X, final_data, i); %measurements for the first update
            [X, P] = Predict(X,P,Q,A);
            [X, P] = Update(X, P, X_m, 0.1, C, A, Q);
        end
        X_arr(i,:) = Xk;
end
%reference this for angle calculation from magnometer

% figure(6)
% scatter3(X_MAG, Y_MAG,Z_MAG);
% 
% figure(7)
% plot(Time, X_arr(:,4));

function [X, P] = Predict(X, P, Q, F)
    X = F*X;
    P = F*P*F' + Q;
end

function [X, P] = Update(X, P, X_m, R, C, A, Q)
    Yk = C*X_m; 
    % solve 9.113 
    P = Q + A*P*A' - (A*P*C')/(1+C*P*C')*(C*P*C');
    Lk = A*(P*C'/(C*P*C' + R));
    Inn = Yk - C*X;
    X = X + Lk*Inn;
    P = P - Lk*C*P;
end

%measurements for the first update 
function [X_m] = Measure_update_one(X, Raw_Data, i)
    %%% our 'measurements' depend on previouse states
    dt = 0.01; 
    theta_x_dot = Raw_Data(i,4);
    theta_y_dot = Raw_Data(i,5);
    theta_z_dot = Raw_Data(i,6);
    
    theta_x = X(10) + dt*(theta_x_dot);
    theta_y = X(11) + dt*(theta_y_dot);
    theta_z = X(12) + dt*(theta_z_dot);
    
    %Calculate the accelerations in inertial frame
    A_I = rotation_matrix(theta_x, theta_y, theta_z).*([Raw_Data(i,1) Raw_Data(i,2) Raw_Data(i,3)]') + [0 0 -9.81]'
    
    x_vel = X(4) + dt*A_I(1);
    y_vel = X(5) + dt*A_I(2);
    z_vel = X(6) + dt*A_I(3);
    
    x_pos = X(1) + dt*x_vel;
    y_pos = X(2) + dt*y_vel;
    z_pos = X(3) + dt*z_vel;
    
    X_m = [x_pos y_pos z_pos x_vel y_vel z_vel A_I(1)...
        A_I(2) A_I(3) theta_x theta_y theta_z...
        theta_x_dot theta_y_dot theta_z_dot Raw_Data(i,9)...
        Raw_Data(i,10) Raw_Data(i,11)]'; 
      
end
