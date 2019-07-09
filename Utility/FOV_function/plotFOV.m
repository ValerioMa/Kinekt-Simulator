function [ fov_plot ] = plotFOV( FOVrot,fov_plot )
%PLOTFOV Summary of this function goes here
%   Detailed explanation goes here

if numel(fov_plot) == 2
    for k = 1:2
        delete(fov_plot(k));
    end
end
fov_plot(1)  = plot(FOVrot.polygon(1,1),FOVrot.polygon(2,1),'k.','markersize',15); % veicle position

% Plot bounding box
% fov_plot(2)  = fill(FOVrot.limits(1,[1,1,2,2,1]),FOVrot.limits(2,[1,2,2,1,1]),'y','facealpha',.2);  % bounding box
% Plot fov
fov_plot(2)  = fill(FOVrot.polygon(1,:),FOVrot.polygon(2,:),'r','facealpha',.6);
% set(fov_plot(2),'EdgeColor','k');
end

