function [ FOVrot ] = rotoTraslateFOV(  FOV, theta, p )
%ROTOTRASLATEFOV Summary of this function goes here
%   Detailed explanation goes here

r = FOV.r;
alpha = FOV.alpha;

x = r*[0,cos(theta-alpha/2),cos(theta + alpha/2),0]+ p(1);
y = r*[0,sin(theta-alpha/2),sin(theta + alpha/2),0]+ p(2);

FOVrot.polygon = [x;y] ;
FOVrot.limits  = [min(x),max(x); min(y), max(y)];

FOVrot.p0 = p;
FOVrot.r = r;
FOVrot.theta = theta;
FOVrot.alpha = alpha; 
% APPROCCIO CON MATRICE DI ROTAZIONE
% FOVrot.polygon = rotoTranslateP(FOV.polygon,theta,p);
% FOVrot.limits  = [min(FOVrot.polygon(1,:)),max(FOVrot.polygon(1,:));...
%                 min(FOVrot.polygon(2,:)), max(FOVrot.polygon(2,:))];
%             


end

