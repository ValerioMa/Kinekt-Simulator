function [ particlesStates ] = povoMapPathGenerator( obstacles )
%POVOMAPPATH Summary of this function goes here
%   Detailed explanation goes here

xRangeObstacles = obstacles.xRange;
yRangeObstacles = obstacles.yRange;
obstaclesTree   = obstacles.obstaclesTree;
WallObstacles   = obstacles.obstacles;

% Agent in the map
fixedParticleNum = 2;          % number of non mooving particle (random placed)
moovingParticleNum = 600;      % number of mooving particle to show (loaded from .mat below)

% Load pregenerated path from saved matrix
load('pathInitialization/percorsiReal_1000.mat')
pathGenerated = percorsiReal;

% Define a position where to put the particle once the particle finish his path
hide_State = [NaN,NaN,NaN];

% Find the max time lenght (duration) of the path
number_time_step = 0;
moovingParticleNum = min(numel(pathGenerated),moovingParticleNum);
for i=1:moovingParticleNum
    number_time_step = max(number_time_step,numel(pathGenerated{i})/3);
end

n_vehicles = moovingParticleNum + fixedParticleNum;
particlesStates = ones(number_time_step,3,n_vehicles);

for i=1:moovingParticleNum
    Percorso = pathGenerated{i};
    righe = numel(Percorso)/3;
    particlesStates(1,:,i) = [righe,n_vehicles,NaN];  % first row = number of element (timestep of the path)
    particlesStates(2:righe+1,:,i) = Percorso;
    
    particlesStates(righe+2:number_time_step+1,1,i) = ones(number_time_step-righe,1)*hide_State(1);
    particlesStates(righe+2:number_time_step+1,2,i) = ones(number_time_step-righe,1)*hide_State(2);
    particlesStates(righe+2:number_time_step+1,3,i) = ones(number_time_step-righe,1)*hide_State(3);
end


max_iter = 1000;   % number of try, if we randomly put the particle on a obstacles
p = zeros(2,1);
% Place random static particle 
for i=(moovingParticleNum+1):n_vehicles
    k=0;
    iter_numb = 0;
    while k<1 && iter_numb < 2*max_iter
        p_rand = rand(3,1);
        p(1) =  p_rand(1)*diff(xRangeObstacles)+xRangeObstacles(1);      % x position
        p(2) =  p_rand(2)*diff(yRangeObstacles)+yRangeObstacles(1);      % y position
        p(3) =  p_rand(3)*2*pi;                                          % theta orientation
        
        % Check if point is inside ObstacleBB
        obstacleIdP = pointIsOnObstacleBB(p,obstaclesTree);
        [ inP ] = pointsInsidePolygons( WallObstacles(obstacleIdP,2), p );
        
        if isempty(inP) || not(sum(inP == 1)>0)    % point is not inside walls
            particlesStates(1,:,i) = [number_time_step,number_time_step,number_time_step];
            particlesStates(2:end,1,i) = ones(number_time_step,1)*p(1);
            particlesStates(2:end,2,i) = ones(number_time_step,1)*p(2);
            particlesStates(2:end,3,i) = ones(number_time_step,1)*p(3);
            k = 1;
        else
            iter_numb = iter_numb + 1; % The point found is inside an obstacles
        end
        
    end
    
    if iter_numb >= max_iter % Ok is enugh, 1000 trials and no position found
        error('Something wrong in the map. In 1000 trials I can not put a particle in free space')
    end
end

end

