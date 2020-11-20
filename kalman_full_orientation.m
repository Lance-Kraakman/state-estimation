%% get data and format it to correct units
format long g

%read the data
A = readtable('example.xls');
data_string = A{:,1:9};
data_string = [data_string A{:,11:13}];
data_input = str2double(data_string);

%%%% Sensor full scale ranges %%%%
ACCELL_SENSOR_RANGE = 2;
GYRO_RANGE = 500;
MAG_RANGE = 4800;

%%%% Manually calculated offsets  %%%%%%
x_rot_off = 0;
y_rot_off = 0;
z_rot_off = 0;

x_acc_off = -0.1843;
y_acc_off = -0.507072682617188;
z_acc_off = 0.28595;

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

%Magnetometer scale
X_MAG = X_MAG.*(1/2^15)*(MAG_RANGE);
Y_MAG = Y_MAG.*(1/2^15)*(MAG_RANGE);
Z_MAG = Z_MAG.*(1/2^15)*(MAG_RANGE);

%%% Magnetometer offset calculation https://www.nxp.com/docs/en/application-note/AN4246.pdf
%%% for vertical and horizontal magnetic field strengths https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
y_mag_matrix = zeros(length(X_MAG), 1);
for i=1:length(X_MAG)
    y_mag_matrix(i) = X_MAG(i)^2 + Y_MAG(i)^2 + Z_MAG(i)^2;
    x_mag_fit(i,:) = [X_MAG(i) Y_MAG(i) Z_MAG(i) 1]; 
end

%%% Caclulate the solution Vector
Beta = (inv(x_mag_fit'*x_mag_fit))*(x_mag_fit')*(y_mag_matrix);
Hard_Iron_vector = (1/2).*Beta;
geo_magnetic_field = sqrt(abs(abs(Hard_Iron_vector(1))^2 + abs(Hard_Iron_vector(2)^2) + abs(Hard_Iron_vector(3)^2) + Beta(4)));


% V_off_x = ones(length(X_MAG),1)*31.6759253314878;
% V_off_y = ones(length(Y_MAG),1)*121.237963699543;
% V_off_z = ones(length(Z_MAG),1)*65.3893711416452;
% X_MAG = X_MAG - V_off_x;
% Y_MAG = Y_MAG - V_off_y;
% Z_MAG = Z_MAG - V_off_z;


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
dt = 0.02; %sample rate/frequency

%states
Xk = zeros(12,1)';         

%observation matrix (C matrix in Y = c*X)
C = eye(12);

%Covariance Matrix
Pk = zeros(12);

%%% System/Plant noise %%%
Q = eye(12); Q(1,1) = 0.1; Q(2,2) = 0.1; Q(3,3) = 0.1;

%%% Measurement nopise %%%%
R_1 = eye(12)*0.3; %we want to "trust" the first measurement more the second measurement is mainly to elimate drift! 
R_2 = eye(12)*0.001;

%%% Measurement Matrix %%%
Y = zeros(1, 9);

A = [
    1 0 0 dt 0 0 -dt 0 0 0 0 0; %[theta x]
    0 1 0 0 dt 0 0 -dt 0 0 0 0; %[theta x]
    0 0 1 0 0 dt 0 0 -dt 0 0 0; %[theta x]
    0 0 0 1 0 0 0 0 0 0 0 0; %[theta dot x]
    0 0 0 0 1 0 0 0 0 0 0 0; %[theta dot y]
    0 0 0 0 0 1 0 0 0 0 0 0; %[theta dot z]
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1];


B = [0 0 0 0 0 0 0 0 0 0 0 0]';

for i = 1:n
        if (i == 1)
            Yk = getMeasurements_1(final_data, Xk, i)';
            Xk = Yk;
        else
            
            
            % first we predict, our prediction uses the previous gyro data 
            Xk_prev = Xk;
            Xk = A*Xk; 
            
            prediction(i,:) = Xk;
            
            %We now update using the accelerometer data for theta
            Yk = getMeasurements_1(final_data, Xk, i)';
            Pk = A*Pk*A' + Q; % priori covariance
            Lk = Pk*C'/(C*Pk*C' + R_1); % compute kalman gain
            Xk = Xk + Lk*(Yk - C*Xk); %update new states
            Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix         
            
            update_1(i,:) = Yk;
            
            Yk = getMeasurements_2(final_data, i, Xk_prev, Xk,dt)';
            Pk = A*Pk*A' + Q; % priori covariance
            Lk = Pk*C'/(C*Pk*C' + R_2); % compute kalman gain
            Xk = Xk + Lk*(Yk - C*Xk); %update new states
            Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix         
            
            %update_1(i,:) = Yk;
            
            %we now update again, using the new gyroscope data
            
            % we need to add magnometer data to measure yaw (theta z)
            %while doing this we can also provide another measurement for
            %our kalman filter
            %https://www.artekit.eu/resources/ak-mag3110/doc/AN4248.pdf
        end
            X_array(i,:) = Xk; 
            %X_array_one_update(i,:) = Xk; 
            
            Yk_meas(i,:) = Yk;
end
%%
%Plots
%Live Animation plot


% tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
% op = orientationPlotter(tp,'DisplayName','Fused Data',...
%     'LocalAxesLength',2);
% for i = 1:1600
%     
% 
%     plotOrientation(op,(X_array(i,1)),(X_array(i,2)),(X_array(i,3)));
%     drawnow
%     pause(dt);
%    
% end
% delete(op)

% 
% plot(Time, [X_array(:,1) X_array(:,2) X_array(:,4) X_array(:,5)],'--', 'LineWidth', 1)
% hold on
% xlim([0 50])
% ylim([-360 360])
% title('state estimates')
% legend({'RX','RY','RX DOT','RY DOT'},'Location','northeast','Orientation','horizontal')


figure(2)
plot(Time, [prediction(:,3) Yk_meas(:,3) X_array(:,3)],'LineWidth', 1);
hold on;
legend({'Z prediction','Z measured','Z final rot'},'Location','northeast','Orientation','horizontal')
title("Z Prediction, update and final")
xlabel("Time (s)")
ylabel("Angle Rotated")
xlim([5 15])
ylim([-360 360])
% 
% figure(3)
% plot(Time, [X_Accel, Y_Accel, Z_Accel])

% figure(3)
% plot(Time, [prediction(:,1) X_array(:,1)]);
% hold on;
% legend({'X prediction','X gyro rot','X final rot'},'Location','northeast','Orientation','horizontal')
% title("X Prediction, update and final")
% xlabel("Time (s)")
% ylabel("Angle Rotated")
% xlim([5 15])
% ylim([-360 360])

% X_MAG = lowpass(X_MAG,10,1/dt); 
% Y_MAG = lowpass(Y_MAG,10,1/dt); 
% Z_MAG = lowpass(Z_MAG,10,1/dt); 

figure(4)
scatter3(X_MAG, Y_MAG,Z_MAG);
hold on; 
xlabel("x")
ylabel("y")
zlabel("z")
title("Magnetic Field Strength (uT)")
xlim([0 120])
ylim([-40 80])
zlim([-70 50])

MAG_DATA = [X_MAG Y_MAG Z_MAG];
%MAG calibration
[W, V, MFS] = magcal([MAG_DATA],'auto') 

for i = 1:length(MAG_DATA(:,1))
    CAL_DATA(i,:) = (MAG_DATA(i,:)-V)*W;
end

figure(5)
scatter3(CAL_DATA(:,1),CAL_DATA(:,2),CAL_DATA(:,3));
hold on; 
xlabel("x")
ylabel("y")
zlabel("z")
title("Magnetic Field Strength (uT)")
xlim([-60 60])
ylim([-60 60])
zlim([-60 60])




% 
% figure(5)
% scatter3((X_MAG+V_off_x), (Y_MAG+V_off_y), (Z_MAG+V_off_z));
% hold on; 
% xlabel("x")
% ylabel("y")
% zlabel("z")
% title("Magnetic Field Strength (uT)")
% xlim([-0.1e3 0.2e3])
% ylim([0.2e3 0.5e3])
% zlim([0.2e3 0.5e3])

% figure(5)
% plot(Time, [(X_MAG+V_off_x) (Y_MAG+V_off_y) (Z_MAG+V_off_z)])
% hold on;
% xlabel("Time (S)")
% ylabel("Magnetic Field Strength (uT)")
% legend({'X mag','Y mag','Z mag'},'Location','southeast','Orientation','horizontal')
% title("Magnetometre field Strengths")

% 
% figure(5)
% plot(Time, [Y_MAG X_array(:,8)]);%[X_MAG, Y_MAG,Z_MAG]
% hold on; 
% xlabel("Time")
% ylabel("uT")
% legend({'Unfiltered','Filtered','kalman filtered'})
% title("Y  MAG")
% xlim([7 15])
% 
% theta = zeros(length(Y_ROT),1);
% theta_prev = 0; theta_current = 0; 
% interval = 0.01;
% for i = 1:length(Y_ROT)
%     theta_current = theta_prev + interval*Y_ROT(i);
%     theta_prev = theta_current;
%     theta(i) = theta_current; 
% end
% figure(6)
% plot(Time,[theta, Yk_meas(:,2)]);


%get measurements for the states we can measure
function [Y] = getMeasurements_1(data,X_current,i)
    Ax = data(i,1);
    Ay = data(i,2);
    Az = data(i,3);
    u = 0.001;
    Y(1) = atand(Ay/sqrt(Ax^2+Az^2));%atand(Ay/(sign(Az)*sqrt(Az^2 + u*(Ax^2)))); 
    Y(2) = atan2d(-Ax,Az); %pitch
    Y(3) = calculate_yaw(data(i,9),data(i,10),data(i,11),Y(1),Y(2)); %atand(Ay/sqrt(Ax^2 + Az^2));
    Y(4) = data(i,4); 
    Y(5) = data(i,5);
    Y(6) = data(i,6);
    Y(7) = X_current(7); %current bias
    Y(8) = X_current(8); % current bias
    Y(9) = X_current(9); %current bias
    Y(10) = data(i,9); 
    Y(11) = data(i,10);
    Y(12) = data(i,11);
end

function [Y] = getMeasurements_2(data, i, X_previous,X_current, dt)
  
    %Theta's
    Y(1) = X_previous(1) + dt*(data(i,4) - X_current(7));
    Y(2) = X_previous(2) + dt*(data(i,5) - X_current(8)); 
    Y(3) = X_previous(3) + dt*(data(i,6) - X_current(9)); 
    % theta dots
    Y(4) = data(i,4); 
    Y(5) = data(i,5);
    Y(6) = data(i,6);
    Y(7) = X_current(7); %current bias
    Y(8) = X_current(8); % current bias
    Y(9) = X_current(9); %current bias
    Y(10) = data(i,9); 
    Y(11) = data(i,10);
    Y(12) = data(i,11);
end



