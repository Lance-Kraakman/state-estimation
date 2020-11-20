function [yaw] = calculate_yaw(Bx,By,Bz,roll,pitch)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
yaw = atan2d((Bz*sind(roll)-By*cosd(roll)),...
    (Bx*cosd(pitch)+By*sind(pitch)*sind(roll)+Bz*sind(pitch)*cosd(roll)));
end

