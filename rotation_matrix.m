function [rot_matrix] = rotation_matrix(x, y, z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
rot_matrix = [cosd(z)*cosd(y) cosd(z)*sind(y)*sind(x)-sind(z)*cosd(x) cosd(z)*sind(y)*cosd(x) + sind(z)*sind(x);...'
     sin(z)*cos(y) sind(z)*sind(y)*sin(x)+cosd(z)*cosd(x) sind(z)*sind(y)*cosd(x) - cosd(z)*sind(x);...
     -1*sin(y) cos(y)*sin(x) cos(y)*cos(x)]; 
end

