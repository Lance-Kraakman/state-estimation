clear all; clc; 
%% create tcp client
t = tcpclient('192.168.1.68', 3333);

V_off_x = 31.6759253314878;
V_off_y = 121.237963699543;
V_off_z = 65.3893711416452;

%Set up Live plot
tp = theaterPlot('XLimit',[-2 2],'YLimit',[-2 2],'ZLimit',[-2 2]);
op = orientationPlotter(tp,'DisplayName','Fused Data',...
    'LocalAxesLength',2);

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



    
W = [  1.01591609604346 9.74111664037991e-05 0.0203028025012319;      
      9.74111664037991e-05 0.982218829204793 -0.0396894088961095;
        0.0203028025012319 -0.0396894088961095 1.00416238455209; ];

V =  [-23.7353176404593 18.1505750763066 -10.7591572373651];
  
% Chebyshev filter Coeffecients
LP_001 = [8.663387E-04 1.732678E-03 8.663387E-04 1.919129E+00 -9.225943E-01];
LP_005 = [1.868823E-02 3.737647E-02 1.868823E-02 1.593937E+00 -6.686903E-01];
LP_01 = [6.372802E-02 1.274560E-01 6.372802E-02 1.194365E+00 -4.492774E-01];
HP_001 = [9.567529E-01 -1.913506E+00 9.567529E-01 1.911437E+00 -9.155749E-01];
HP_0025 = [8.950355E-01 -1.790071E+00 8.950355E-01 1.777932E+00 -8.022106E-01];
HP_005 = [8.001102E-01 -1.600220E+00  8.001102E-01 1.556269E+00 -6.441715E-01];
HP_03 = [6.362308E-01 1.272462E+00 6.362308E-01 -1.125379E+00 -4.195441E-01];

%Filter array
y_arr_alp = [0 0 0 0 0 0]; x_arr_alp = [0 0 0 0 0 0]; y_arr_mlp = [0 0 0 0 0 0]; x_arr_mlp = [0 0 0 0 0 0]; y_arr_ghp = [0 0 0 0 0 0]; x_arr_ghp = [0 0 0 0 0 0];
y_arr_ahp = [0 0 0 0 0 0]; x_arr_ahp = [0 0 0 0 0 0]; y_arr_alp_p = [0 0 0 0 0 0]; x_arr_alp_p = [0 0 0 0 0 0];


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
R_1 = eye(12)*0.01; R_1(3,3) = 0.01; %we want to "trust" the first measurement more the second measurement is mainly to elimate drift! 
R_2 = eye(12)*0.01;

%%% Measurement Matrix %%%
Y = zeros(1, 12);

A = [
    1 0 0 dt 0 0 -dt 0 0; %[theta x]
    0 1 0 0 dt 0 0 -dt 0; %[theta x]
    0 0 1 0 0 dt 0 0 -dt; %[theta x]
    0 0 0 1 0 0 0 0 0; %[theta dot x]
    0 0 0 0 1 0 0 0 0; %[theta dot x]
    0 0 0 0 0 1 0 0 0; %[theta dot x]
    0 0 0 0 0 0 1 0 0; %[mag x]
    0 0 0 0 0 0 0 1 0; %[mag y]
    0 0 0 0 0 0 0 0 1;]; %[mag z]


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


B = [0 0 0 0 0 0 0 0 0 0 0 0]';
data_input = read(t, 120);

% t_arr = [];
% x_arr = [];

data_array = []; 
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
%     X_Accel = X_Accel + x_acc_off;
%     Y_Accel = Y_Accel + y_acc_off;
%     Z_Accel = Z_Accel + z_acc_off;

    %Magnetometer scale
    X_MAG = X_MAG.*(1/2^15)*(MAG_RANGE);
    Y_MAG = Y_MAG.*(1/2^15)*(MAG_RANGE);
    Z_MAG = Z_MAG.*(1/2^15)*(MAG_RANGE);
    
    MAG_DATA = [X_MAG Y_MAG Z_MAG];
    
    CAL_MAG_DATA = (MAG_DATA-V)*W;
    
    
   final_data = [X_Accel, Y_Accel, Z_Accel,X_ROT, Y_ROT, Z_ROT, RSSI, Temp, CAL_MAG_DATA(1), CAL_MAG_DATA(2), CAL_MAG_DATA(3)];
    
    %% Apply filters to the data 
    
    %Low Pass Filter accelerations 
    [y_arr_alp(1), y_arr_alp(2), x_arr_alp(1), x_arr_alp(2)] = chev_filter(y_arr_alp(1), y_arr_alp(2),X_Accel,x_arr_alp(1), x_arr_alp(2),LP_01);
    [y_arr_alp(3), y_arr_alp(4), x_arr_alp(3), x_arr_alp(4)] = chev_filter(y_arr_alp(3), y_arr_alp(4),Y_Accel,x_arr_alp(3), x_arr_alp(4),LP_01);
    [y_arr_alp(5), y_arr_alp(6), x_arr_alp(5), x_arr_alp(6)] = chev_filter(y_arr_alp(5), y_arr_alp(6),Z_Accel,x_arr_alp(5), x_arr_alp(6),LP_01);
    
    Accell_filtered(i,:) = [y_arr_alp(1) y_arr_alp(3) y_arr_alp(5)]; %n-2 data points 
    
    %Low pass Filter Magnetometre Data
    [y_arr_mlp(1), y_arr_mlp(2), x_arr_mlp(1), x_arr_mlp(2)] = chev_filter(y_arr_mlp(1), y_arr_mlp(2),CAL_MAG_DATA(1),x_arr_mlp(1), x_arr_mlp(2),LP_01);
    [y_arr_mlp(3), y_arr_mlp(4), x_arr_mlp(3), x_arr_mlp(4)] = chev_filter(y_arr_mlp(3), y_arr_mlp(4),CAL_MAG_DATA(2),x_arr_mlp(3), x_arr_mlp(4),LP_01);
    [y_arr_mlp(5), y_arr_mlp(6), x_arr_mlp(5), x_arr_mlp(6)] = chev_filter(y_arr_mlp(5), y_arr_mlp(6),CAL_MAG_DATA(3),x_arr_mlp(5), x_arr_mlp(6),LP_01);
    
    MAG_filtered(i,:) = [y_arr_mlp(1) y_arr_mlp(3) y_arr_mlp(5)]; %n-2 data points 
    
    %%% High Pass filter Gyroscope
    [y_arr_ghp(1), y_arr_ghp(2), x_arr_ghp(1), x_arr_ghp(2)] = chev_filter(y_arr_ghp(1), y_arr_ghp(2),X_ROT,x_arr_ghp(1), x_arr_ghp(2),HP_001);
    [y_arr_ghp(3), y_arr_ghp(4), x_arr_ghp(3), x_arr_ghp(4)] = chev_filter(y_arr_ghp(3), y_arr_ghp(4),Y_ROT,x_arr_ghp(3), x_arr_ghp(4),HP_001);
    [y_arr_ghp(5), y_arr_ghp(6), x_arr_ghp(5), x_arr_ghp(6)] = chev_filter(y_arr_ghp(5), y_arr_ghp(6),Z_ROT,x_arr_ghp(5), x_arr_ghp(6),HP_001);
    
    GYRO_filtered(i,:) = [y_arr_ghp(1) y_arr_ghp(3) y_arr_ghp(5)];
    
    %%% High Pass acceleration to get induced acelleration
    [y_arr_ahp(1), y_arr_ahp(2), x_arr_ahp(1), x_arr_ahp(2)] = chev_filter(y_arr_ahp(1), y_arr_ahp(2),X_Accel,x_arr_ahp(1), x_arr_ahp(2),HP_001);
    [y_arr_ahp(3), y_arr_ahp(4), x_arr_ahp(3), x_arr_ahp(4)] = chev_filter(y_arr_ahp(3), y_arr_ahp(4),Y_Accel,x_arr_ahp(3), x_arr_ahp(4),HP_001);
    [y_arr_ahp(5), y_arr_ahp(6), x_arr_ahp(5), x_arr_ahp(6)] = chev_filter(y_arr_ahp(5), y_arr_ahp(6),Z_Accel,x_arr_ahp(5), x_arr_ahp(6),HP_001);
    
    %low pass acellerations (at a HIGHER frequency than for the kalman filter/orientation)
    [y_arr_alp_p(1), y_arr_alp_p(2), x_arr_alp_p(1), x_arr_alp_p(2)] = chev_filter(y_arr_alp_p(1), y_arr_alp_p(2),X_Accel,x_arr_alp_p(1), x_arr_alp_p(2),LP_001);
    [y_arr_alp_p(3), y_arr_alp_p(4), x_arr_alp_p(3), x_arr_alp_p(4)] = chev_filter(y_arr_alp_p(3), y_arr_alp_p(4),Y_Accel,x_arr_alp_p(3), x_arr_alp_p(4),LP_001);
    [y_arr_alp_p(5), y_arr_alp_p(6), x_arr_alp_p(5), x_arr_alp_p(6)] = chev_filter(y_arr_alp_p(5), y_arr_alp_p(6),Z_Accel,x_arr_alp_p(5), x_arr_alp_p(6),LP_001);
    
    %% Estimation of Position from "bad data" 
    
    %Band pass filter for inertial frame acelleration estimation
    Accell_Band_filtered(i,:) = [(y_arr_ahp(1)+y_arr_alp_p(1)-X_Accel) (y_arr_ahp(3)+y_arr_alp_p(3)-Y_Accel) (y_arr_ahp(5)+y_arr_alp_p(5)-Z_Accel)];
%high pass only
    %Accell_Band_filtered(i,:) = [(y_arr_ahp(1)) (y_arr_ahp(3)) (y_arr_ahp(5))];  %HPF only
    %Accell_Band_filtered(i,:) = [X_Accel Y_Accel Z_Accel];   
    if (i >= 2) 
        if (abs(Accell_Band_filtered(i,1)) < 0.01) % Mechanical Window
            count = count + 1;
        else 
            count = 0; 
        end
            if (count >= 10) % If we get 5 results in the mechanical window we set the velocity to zero
                for j=0:count-1
                    Velocity_x(i-j) = 0; %set the corrosponding velocities to zero
                end
                count = 0;
            else
                Velocity_x(i) = Velocity_x(i-1) + (((Accell_Band_filtered(i,1)-Accell_Band_filtered(i-1,1))/2)+(Accell_Band_filtered(i-1,1)))*dt;
            end
    elseif (i == 1) 
        Velocity_x(i) = 0;
    end
    
    if (i >= 2)
        Position_x(i) = Position_x(i-1) + (((Velocity_x(i)-Velocity_x(i-1))/2)+Velocity_x(i-1))*dt;
    else
        Position_x(i) = 0; 
    end
    %%% Final Data For Kalman %%%
    
%     final_data = [y_arr_alp(1) y_arr_alp(3) y_arr_alp(5),y_arr_ghp(1) y_arr_ghp(3) y_arr_ghp(5), RSSI,...
%        Temp, y_arr_mlp(1) y_arr_mlp(3) y_arr_mlp(5)];
    %% kalman Filter 
    if (i == 1)
        %%% Initialise Kalman Filter
        
        Yk = getMeasurements_1(final_data, Xk, i)';
        Xk = Yk;
        Xk(7) = x_rot_off; Xk(8) = y_rot_off; Xk(9) = z_rot_off;
        y_meas_1 = Yk;
    else
       
        % predict, our prediction uses the previous gyro data 
        Xk_prev = Xk;
        %Xk = A*Xk; 
        [Xk, Pk] = Predict(Xk, Pk, Q, A);
        
        
        %We now update using the accelerometer data for theta
        Yk = getMeasurements_1(final_data, Xk, i)';
        
        [Xk, Pk] = Update(Xk, Pk,Yk,R_1,C,A,Q);
%         Pk = A*Pk*A' + Q; % priori covariance
%         Lk = Pk*C'/(C*Pk*C' + R_1); % compute kalman gain
%         Xk = Xk + Lk*(Yk - C*Xk); %update new states
%         Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix         

        y_meas_1 = Yk;
        
       %Yk = getMeasurements_2(final_data, i, Xk_prev, Xk,dt)';
       %[Xk, Pk] = Update(Xk, Pk,Yk,R_2,C,A,Q);
%         Pk = A*Pk*A' + Q; % priori covariance
%         Lk = Pk*C'/(C*Pk*C' + R_2); % compute kalman gain
%         Xk = Xk + Lk*(Yk - C*Xk);  %update new states
%         Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix
        
    end
    A_I = rotation_matrix(Xk(1), Xk(2), Xk(3))*([X_Accel Y_Accel 9.81]') + [0 0 -9.81]';
    
    %% Position Estimation. Referenece https://www.nxp.com/docs/en/application-note/AN3397.pdf
    
    %%%  High passed filter to calculate induced acellerations 
    %%%  Orientation range adjustment because we are using rpy angles we need
    %%%  to adjust them so there is only a single solution. We do not do this
    %%%  in the kalman filter so we can keep it "Linear"
    
   %%Plots
    %accelleration_array(i,:) = A_I'
    data_array(i,:) = final_data;
    state_array(i,:) = Xk;
   % yaw = calculate_yaw(CAL_MAG_DATA(1),CAL_MAG_DATA(2),CAL_MAG_DATA(3),Xk(1),Xk(2));
    %yaw = atan2(CAL_MAG_DATA(2),CAL_MAG_DATA(1));
    plotOrientation(op,0,Xk(2),Xk(3),[0 0 0]);
    %drawnow

end

%plot the results of the filter 
%plot([0:dt:(length(data_array(:,4))*dt)-dt],[data_array(:,1) X_Accell_filtered]);
 
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

function [Yi, Yi_1, Xi, Xi_1] = chev_filter(Yi_1, Yi_2, Xi, Xi_1, Xi_2, Co_eff)

 
 Yi = Co_eff(1)*Xi + Co_eff(2)*Xi_1 + Co_eff(3)*Xi_2 + Co_eff(4)*Yi_1 + Co_eff(5)*Yi_2; 
    
end

function [Yi, Yi_1, Xi, Xi_1] = chev_filter_hp(Yi_1, Yi_2, Xi, Xi_1, Xi_2)
    %Chebyshev filter Coeffecients
    %Fc hp = 0.025
  %  A_HP = [8.950355E-01 -1.790071E+00 8.950355E-01];
  %  B_HP = [1.777932E+00 -8.022106E-01];
    %Fc hp= 0.01 for high pass component
     A_HP = [9.567529E-01 -1.913506E+00 9.567529E-01];
     B_HP = [1.911437E+00 -9.155749E-01];
    %Fc hp = 0.25
%     A_HP = [2.858110E-01 -5.716221E-01 2.858111E-01];
%     B_HP = [-5.423258E-02 -1.974768E-01];
     %Fc hp = 0.05
%      A_HP = [8.001102E-01 -1.600220E+00  8.001102E-01];
%      B_HP = [1.556269E+00 -6.441715E-01];
    
    %Fc = 0.25
 %   A_LP = [3.849163E-01 7.698326E-01 3.849163E-01];
 %   B_LP = [-3.249116E-01 -1.974768E-01];
    %Fc = 0.075
    A_LP = [3.869430E-02 7.738860E-02 3.869430E-02];
    B_LP = [1.392667E+00 -5.474446E-01];
    %Fc = 0.01
%     A_LP = [8.663387E-04 1.732678E-03 8.663387E-04];
%     B_LP = [1.919129E+00 -9.225943E-01];
   % Fc = 0.1Hz
%    A_LP = [6.372802E-02 1.274560E-01 6.372802E-02 1.194365E+00 -4.492774E-01];
    %Fc = 0.3Hz (normalised)
%      A_LP = [6.362308E-01 1.272462E+00 6.362308E-01];
%      B_LP = [-1.125379E+00 -4.195441E-01];
 
    Yi = A_HP(1)*Xi + A_HP(2)*Xi_1 + A_HP(3)*Xi_2 + B_HP(1)*Yi_1 + B_HP(2)*Yi_2;%... % High pass component
   %  + A_LP(1)*Xi + A_LP(2)*Xi_1 + A_LP(3)*Xi_2 + B_LP(1)*Yi_1 + B_LP(2)*Yi_2;% Low pass component
         
    
end

function [Y] = getMeasurements_1(data,X_current,i)
    
    Ax = data(1);
    Ay = data(2);
    Az = data(3);
    u = 0.01;
    Y(1) = atan2d(Ay,(sign(Az)*sqrt(Az^2 + u*(Ax^2)))); %atand(Ay/(sign(Az)*sqrt(Az^2 + u*(Ax^2)))); 
    Y(2) = atan2d(-Ax,(sqrt(Ay^2 + (Az^2)))); %atan2d(-Ax,Az); %pitch
    %Y(1) = atand(Ay/(sign(Az)*sqrt(Az^2 + u*(Ax^2))));
    %Y(2) = atan2d(-Ax,Az); %pitch
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
%     if (i > 2000) 
%         %bias_x = ((X_current(7)*dt)/(i-1) + data(4))/i; %https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6111699/
%         %bias_y = ((X_current(8)*dt)/(i-1) + data(5))/i;
%         %bias_z = ((X_current(9)*dt)/(i-1) + data(6))/i;
%     else
%         bias_x = X_previous(7); %https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6111699/
%         bias_y = X_previous(8);
%         bias_z = X_previous(9)
%     end
    bias_x = X_previous(7); 
    bias_y = X_previous(8);
    bias_z = X_previous(9);
    %Theta's
    Y(1) = gyro_update((data(4) - bias_z),dt,X_previous(1),-180,180); %X_previous(1) + dt*(data(4) - bias_x);%gyro_update((data(4) - bias_z),dt,X_previous(1),-180,180);%
    Y(2) = gyro_update((data(5) - bias_y),dt,X_previous(2),-180,180);   %X_previous(2) + dt*(data(5) - bias_y);%gyro_update((data(5) - bias_y),dt,X_previous(2),-90,90);%X_previous(2) + dt*(data(5) - bias_y); %here we need to restrict the range then im gucci 
    Y(3) = gyro_update((data(6) - bias_z),dt,X_previous(3),-180,180); %X_previous(3) + dt*(data(6) - bias_z);%gyro_update((data(6) - bias_z),dt,X_previous(3),-180,180);%X_previous(3) + dt*(data(6) - bias_z);
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
 
function [Xk, Pk] = Predict(Xk, Pk, Q, A)
    Xk = A*Xk; % Make prediction
    Pk = A*Pk*A' + Q; % update Covariance
end

function [Xk, Pk] = Update(Xk, Pk, Yk, R, C, A, Q)
    Pk = Q + A*Pk*A' - (A*Pk*C')/(R+C*Pk*C')*(C*Pk*A'); %Update Error Covariance
    Lk = (Pk*C'/(C*Pk*C' + R)); % Calculate Kalman Gain
    Xk = Xk + Lk*(Yk - C*Xk); % Calculate state estimate
    Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix
end

function [theta] = gyro_update(theta_dot, dt, theta_prev, range_minus, range_plus)
    theta = theta_prev + dt*theta_dot; 
    if (theta > range_plus)
        theta = theta - (2*range_plus)
    elseif (theta < range_minus) 
        theta = theta - (2*range_minus)
    end
end

% Pk = A*Pk*A' + Q; % priori covariance
%         Lk = Pk*C'/(C*Pk*C' + R_1); % compute kalman gain
%         Xk = Xk + Lk*(Yk - C*Xk); %update new states
%         Pk = (eye(12) - Lk*C)*Pk; % update covariance matrix

