clear all; clc; 
%% create tcp client
t = tcpclient('192.168.1.64', 3333);

V_off_x = 31.6759253314878;
V_off_y = 121.237963699543;
V_off_z = 65.3893711416452;

%Set up Live plot
% tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
% op = orientationPlotter(tp,'DisplayName','Fused Data',...
%     'LocalAxesLength',2);

%%%% Sensor full scale ranges %%%%
ACCELL_SENSOR_RANGE = 2;
GYRO_RANGE = 500;
MAG_RANGE = 4800;

%%%% Manually calculated offsets  %%%%%%
x_rot_off = -0.445250454067702;
y_rot_off = -0.476958277564781;
z_rot_off = 3.84395752037967;

x_acc_off = -0.1778302002;
y_acc_off = -0.4089498555;
z_acc_off = -0.374186906736351;

%%Magnometer Calibration Paramaters

W = [0.961049346598892 -0.0403693024506181 -0.0575845924508981;
   -0.0403693024506181 0.983446017456633 -0.209563331052356;
    -0.0575845924508981 -0.209563331052356 1.10909380284382];

W = [   0.965789167214929        0.0333955106230079       0.0588079845976007;
        0.0333955106230079           1.11730827502191        0.0227935771591127;
        0.0588079845976007        0.0227935771591127         0.931637503875174]

V =  [59.0167747835314 23.0098891107168 -8.664078323705];
  
%Chebyshev filter Coeffecients


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
R_1 = eye(12)*0.3; R_1(3,3) = 3; %we want to "trust" the first measurement more the second measurement is mainly to elimate drift! 
R_2 = eye(12)*0.001;

%%% Measurement Matrix %%%
Y = zeros(1, 12);

A = [
    1 0 0 dt 0 0 -dt 0 0 0 0 0;
    0 1 0 0 dt 0 0 -dt 0 0 0 0;
    0 0 1 0 0 dt 0 0 -dt 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1];



B = [0 0 0 0 0 0 0 0 0 0 0 0]';
data_input = read(t, 120);

% t_arr = [];
% x_arr = [];
figure(1) 
hold on; 
data_array = []

DATA_POINTS = 5000
for i = 1:DATA_POINTS
    
    data_input = read(t, 120);
    data_string = split(native2unicode(data_input),',');
    
    X_Accel = str2double(data_string(1)); Y_Accel = str2double(data_string(2));
    Z_Accel = str2double(data_string(3)); Temp = str2double(data_string(4));
    X_ROT = str2double(data_string(5)); Y_ROT = str2double(data_string(6)); 
    Z_ROT = str2double(data_string(7)); RSSI = str2double(data_string(8));
    Count = str2double(data_string(9)); 
    X_MAG = str2double(data_string(10));Y_MAG = str2double(data_string(11));
    Z_MAG = str2double(data_string(12));
    
    %convert accelleration to m/s^2
    X_Accel = X_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81);
    Y_Accel = Y_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81);
    Z_Accel = Z_Accel.*(1/2^15)*(ACCELL_SENSOR_RANGE*9.81);

    %convert to make sense
    X_ROT = X_ROT.*(1/2^15)*(GYRO_RANGE);
    Y_ROT = Y_ROT.*(1/2^15)*(GYRO_RANGE);
    Z_ROT = Z_ROT.*(1/2^15)*(GYRO_RANGE);
    Temp = Temp.*(1/340) + 36.53;

    %offsets acell 
    X_Accel = X_Accel + x_acc_off;
    Y_Accel = Y_Accel + y_acc_off;
    Z_Accel = Z_Accel + z_acc_off;

    %Magnetometer scale
    X_MAG = X_MAG.*(1/2^15)*(MAG_RANGE);
    Y_MAG = Y_MAG.*(1/2^15)*(MAG_RANGE);
    Z_MAG = Z_MAG.*(1/2^15)*(MAG_RANGE);
    
    MAG_DATA = [X_MAG Y_MAG Z_MAG];
    
    CAL_MAG_DATA = (MAG_DATA-V)*W;
    
    
    final_data = [X_Accel, Y_Accel, Z_Accel,X_ROT, Y_ROT, Z_ROT, RSSI, Temp, CAL_MAG_DATA(1), CAL_MAG_DATA(2), CAL_MAG_DATA(3)];
     % collect the data 
    
    if (i == 1)
        Yk = getMeasurements_1(final_data, Xk, i)';
        Xk = Yk;
        Xk(7) = x_rot_off; Xk(8) = y_rot_off; Xk(9) = z_rot_off;
        y_meas_1 = Yk;
    else
        % first we predict, our prediction uses the previous gyro data 
        Xk_prev = Xk;
        Xk = A*Xk; 

        %We now update using the accelerometer data for theta
        Yk = getMeasurements_1(final_data, Xk, i)';
        Pk = A*Pk*A' + Q; % priori covariance
        Lk = Pk*C'/(C*Pk*C' + R_1); % compute kalman gain
        Xk = Xk + Lk*(Yk - C*Xk); %update new states
        Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix         

        y_meas_1 = Yk;
        
%        Yk = getMeasurements_2(final_data, i, Xk_prev, Xk,dt)';
         Pk = A*Pk*A' + Q; % priori covariance
         Lk = Pk*C'/(C*Pk*C' + R_2); % compute kalman gain
         Xk = Xk + Lk*(Yk - C*Xk);  %update new states
         Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix
        
    end
    
    % when we attempt this in our kalman filter this will be our prediction
    % and the rssi propagation modele will be out measurement
    A_I = rotation_matrix(Xk(1), Xk(2), 0)*([X_Accel Y_Accel 9.81]') + [0 0 -9.81]';
    
    
   
    %accelleration_array(i,:) = A_I'
    %data_array(i,:) = final_data;
    %state_array(i,:) = Xk;
   % yaw = calculate_yaw(CAL_MAG_DATA(1),CAL_MAG_DATA(2),CAL_MAG_DATA(3),Xk(1),Xk(2));
    %yaw = atan2(CAL_MAG_DATA(2),CAL_MAG_DATA(1));
    %plotOrientation(op,0,0,Xk(3));
    %drawnow

end
%roll pitch yaw (mag)
% figure(3)
% plot([0:dt:length(y_meas_1(3,:))*dt-dt], [state_array(:,1) state_array(:,2) y_meas_1(3,:)]); axis([0 20 -350 350]);
% legend({'meas x','meas y',''})
% 
% figure(4)
% plot([0:dt:(length(data_array(:,4))*dt)-dt], [data_array(:,1) data_array(:,2) data_array(:,3)]); axis([0 length(data_array(:,9))*dt -350 350]);
 
figure(4)
plot([0:dt:(length(data_array(:,4))*dt)-dt],[accelleration_array(:,1) data_array(:,7)]);

V_array =[0 0]
P_array = [0 0]
for i=2:1:(length(data_array(:,7)))
    V_array(i,:) = [(dt*accelleration_array(i,1)+V_array(i-1,1)) (dt*accelleration_array(i,2)+V_array(i-1,2))];
    %P_array(i,:) = P_array + []
end

plot(Time, V_array);

figure(5)
scatter3(data_array(:,9),data_array(:,10),data_array(:,11));
xlim([-0.1e3 0.3e3])
ylim([-0.1e3 0.3e3])
zlim([-0.1e3 0.3e3])

function [Y] = getMeasurements_1(data,X_current,i)
    u = 0.0001;
    Ax = data(1);
    Ay = data(2);
    Az = data(3);
    
    Y(1) = atand(Ay/(sign(Az)*sqrt(Az^2 + u*(Ax^2)))); 
    Y(2) = atan2d(-Ax,Az); %pitch
    Y(3) = calculate_yaw(data(9),data(10),data(11),Y(1),Y(2)); %atand(Ay/sqrt(Ax^2 + Az^2));
    Y(4) = data(4); 
    Y(5) = data(5);
    Y(6) = data(6);
    Y(7) = X_current(7); %current bias so no effect
    Y(8) = X_current(8); % current bias so no effect 
    Y(9) = X_current(9); %current bias do no effect 
    Y(10) = data(9); 
    Y(11) = data(10);
    Y(12) = data(11);
end

function [Y] = getMeasurements_2(data, i, X_previous,X_current, dt)
    if (i > 2000) 
        bias_x = ((X_current(7)*dt)/(i-1) + data(4))/i; %https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6111699/
        bias_y = ((X_current(8)*dt)/(i-1) + data(5))/i;
        bias_z = ((X_current(9)*dt)/(i-1) + data(6))/i;
    else
        bias_x = X_previous(7); %https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6111699/
        bias_y = X_previous(8);
        bias_z = X_previous(9)
    end
    %Theta's
    Y(1) = X_previous(1) + dt*(data(4) - bias_x);
    Y(2) = X_previous(2) + dt*(data(5) - bias_y); 
    Y(3) = X_previous(3) + dt*(data(6) - bias_z); 
    % theta dots
    Y(4) = data(4); 
    Y(5) = data(5);
    Y(6) = data(6);
    Y(7) = bias_x; %current bias
    Y(8) = bias_y; % current bias
    Y(9) = bias_z; %current bias
    Y(10) = data(9); 
    Y(11) = data(10);
    Y(12) = data(11);
end
