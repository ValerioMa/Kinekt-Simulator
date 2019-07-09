function [ FOV ] = defineFOV(  )
%DEFINEFOV Summary of this function goes here
%   Detailed explanation goes here

r = 15*2;
alpha = 0.53;

% Create a structure with a polygond defining the fov and the bounding box
x = r*[0,cos(alpha/2),cos(alpha/2),0];
y = r*[0,sin(-alpha/2),sin(alpha/2),0];
FOV.polygon = [x;y];
FOV.limits  = [min(x),max(x); min(y), max(y)];
FOV.r = r;
FOV.alpha = alpha;

end

