% --------------------------------------------------------------
%
%   SIMULATION OF KINECKT
%
% --------------------------------------------------------------
%======================================================================%
%                                                                      %
%  Autors: Valerio Magnago                                             %
%          Daniele Fontanelli                                          %
%          University of Trento                                        %
%          valerio.magnago@unitn.it                                    %
%                                                                      %
%======================================================================%

% TODO: gestire count 3 in intersezione con triang

filename = which(mfilename);
[pathstr,name,ext] = fileparts(filename);
cd(pathstr);

clc;
clear all;
close all;
addpath(genpath('./Utility'));
addpath(genpath('./pathInitialization'));
set(0,'DefaultFigureWindowStyle','docked')


%**************************************************************************
%% Parameters definition
%**************************************************************************
sec_numb = 1; fprintf('%d) Load params\n',sec_numb); sec_numb = sec_numb + 1;

exportVideoFlag = 0; % 0 - disable export to video ; 1 - enable plot export to video
plotFlag = 1 || exportVideoFlag;        % 0 - disable plot; 1 - enable plot


% Define scenatio to use
scenario = 2;

% scenario:
%     1 --> simply Enviroment
%     2 --> real building floor map

AgentNumb = 10;

% Function to define the FOV of the Kinekt
FOV = defineFOV();


%% Load Obstacles map
fprintf('%d) Load obstacles map\n',sec_numb); sec_numb = sec_numb + 1;

switch scenario
    case 1
        mapName = './Data/Easymap.txt';
    case 2
        mapName = './Data/povoLineObstWall.txt';
        
    otherwise
        error('Scenario not implemented');
end


% Load Obstacles
[ WallObstacles,xRangeObstacles,yRangeObstacles ] = loadMapObstacles( mapName );
% obstacles is a cell
% first column  = point of the obstacles polygon
% second column = bounding box of the obstacles



%**************************************************************************
%% Creeate obstacles tree
%**************************************************************************
fprintf('%d) Create obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;

[ obstacles ] = createTree( WallObstacles, xRangeObstacles, yRangeObstacles);
% obstacles:
%       - obstaclesTree = binary tree of the obstacles (Bounding Box)
%       - obstacles = array of poligon (the effective obstacles)
%       - xRange = range in x of the obstacles
%       - yRange = range in y of the obstacles

%**************************************************************************
%% Creeate vehicles motion array
%**************************************************************************
fprintf('%d) Create static vehicles object\n',sec_numb); sec_numb = sec_numb + 1;

% Load matrix of motion of the particles
[ particlesStates,vehicle_indexes ] = GenerateMotionArray( obstacles,AgentNumb,scenario);

% particlesStates:
%   - particlesStates(1,1,particleID) = number of timestep (duration) of
%   the motion of particlesState
%   - particlesStates(1,2,particleID) = number of particle
%   - particlesStates(timestep+1,:,particleID) = state [x,y,theta] of the
%   particle "particleID" at time timestep


%**************************************************************************
%% Simulation
%**************************************************************************
fprintf('%d) Simulation\n',sec_numb); sec_numb = sec_numb + 1;
% In this part we save the data of the simulation in the result variable
% No graphic rappresentation is done in this section



% start to mesure the time
tic
n_particle = particlesStates(1,2,1);


% Initialize output variable
result.particleState = particlesStates(2:end,:,:);
result.simNumber = numel(vehicle_indexes);
result.simulation = cell(result.simNumber,1);
maxTimestep = 0;
for particleID=1:result.simNumber
    vehicle_index = vehicle_indexes(particleID);
    result.simulation{particleID}.mainParticleID  = vehicle_index;
    others_index = 1:n_particle;
    result.simulation{particleID}.othersParticleID                   = others_index(others_index~=vehicle_index);
    result.simulation{particleID}.timeStepNum                        = particlesStates(1,1,vehicle_index);
    result.simulation{particleID}.wallsInFov                         = cell(result.simulation{particleID}.timeStepNum,1);
    result.simulation{particleID}.particleInFovRelativePosition      = cell(result.simulation{particleID}.timeStepNum,1);
    result.simulation{particleID}.particleInFovRelativePosition      = cell(result.simulation{particleID}.timeStepNum,1);
    result.simulation{particleID}.wallsInFovRelativePosition         = cell(result.simulation{particleID}.timeStepNum,1);    
    result.simulation{particleID}.particleInFovId                    = cell(result.simulation{particleID}.timeStepNum,1);
    maxTimestep = max(maxTimestep,result.simulation{particleID}.timeStepNum);
end


for i = 1:maxTimestep
    for particleID=1:result.simNumber
        if(result.simulation{particleID}.timeStepNum<i)
            continue; % Particle ended his path
        end
        vehicle_index = result.simulation{particleID}.mainParticleID;
        others_index  = result.simulation{particleID}.othersParticleID;
        
        % Path point and orientation
        p = particlesStates(i+1,1:2,vehicle_index)';
        theta = particlesStates(i+1,3,vehicle_index);
        
        % Check if point is inside ObstacleBB
        obstacleIdP = pointIsOnObstacleBB(p,obstacles.obstaclesTree);
        
        % Check if point is inside polygons
%         [ inP ] = pointsInsidePolygons( WallObstacles(obstacleIdP,2), p );
%         
%         if inP~=0
%             fprintf('\t\t Path point seems to be inside obstables !!!\n')
%         end
        
        % Traslate the FOV of the main particle
        [ FOVrot ] = rotoTraslateFOV( FOV, theta, p );
        
        % Looking for obstacles intersection
        [ LaneView, WallSegmentsInFOV, ~ ] = findWallsIntersection( FOVrot,obstacles);
        
        
        IDparticleInFOV = [];
        for k=others_index
            p2 = particlesStates(i+1,1:2,k)';
            [ logic ] = pointInFOV( p2, FOVrot, LaneView);
            
            if logic % point is not occluded by walls
                IDparticleInFOV = [IDparticleInFOV,k];  % save the id of the particle in the FOV
            end
        end
        
        
        % Traslate point in particle coordinate reference system
        trasl = -rotoTranslateP( FOVrot.p0, -FOVrot.theta, [0;0]);
        
        % Segment of the polygon viewd, not considering the self occlusion
        segmentOnBoard = zeros(size(WallSegmentsInFOV));
        for kk=1:numel(WallSegmentsInFOV)/4
            segmentOnBoard(:,:,kk) = rotoTranslateP( WallSegmentsInFOV(:,:,kk), -FOVrot.theta, trasl );
        end
        
        % Segment of walls in fov considering self occlusion.
        LaneViewOnBoard = zeros(size(LaneView));
        for kk=1:numel(LaneView)/4
            LaneViewOnBoard(:,:,kk) = rotoTranslateP( LaneView(:,:,kk), -FOVrot.theta, trasl );
        end
        
        % Find global position of particle in FOV
        n = numel(IDparticleInFOV);
        particleInFOVonBoard = zeros(3,n);
        for k=1:n
            particleInFOVonBoard(1:2,k) = rotoTranslateP(particlesStates(i,1:2,IDparticleInFOV(k))',-FOVrot.theta, trasl);
            particleInFOVonBoard(3,k)   = particlesStates(i,3,IDparticleInFOV(k)) - FOVrot.theta;
        end
        
        % Save Data to export        
        result.simulation{particleID}.particleInFovId{i}               = IDparticleInFOV;
        result.simulation{particleID}.particleInFovRelativePosition{i} = particleInFOVonBoard;
        result.simulation{particleID}.wallsInFov{i}                    = LaneView;
        result.simulation{particleID}.wallsInFovRelativePosition{i}    = LaneViewOnBoard;
    end
    
end
computationTime = toc;
fprintf('\t- Done in %.2f [ms]\n',computationTime*1000);

%**************************************************************************
%% Save and load data
%**************************************************************************
fprintf('%d)Save the data and clear console\n',sec_numb); sec_numb = sec_numb + 1;

% Save the useful variable
save('./Data/resultDemo','result');
save('./Data/obstaclesDemo','obstacles');
save('./Data/fovDemo','FOV');

%**************************************************************************
%% Plot the results
%**************************************************************************
if plotFlag
    fprintf('%d) Demo Plot\n',sec_numb); sec_numb = sec_numb + 1;    
    if exportVideoFlag
        animation = VideoWriter('./Data/videoDemo.avi');
        animation.FrameRate = 4;
        open(animation);
    end
    
    % % Plot the map
    fig_id = figure(1); clf; hold on;
    global stop;
    stop = 0;
    pb = uicontrol(fig_id,'Style','pushbutton','String','STOP',...
                'Position',[50 20 60 40],'Callback',@stopbutton_callback);
    subplot(2,1,1);
    hold on;
    title('Map');
    fig_1_ObstPoly = plotObstacles(obstacles.obstacles(:,2), 1 ,{[0.4,0.4,0.4],0.5});
%  PLOT THE BOUNDING BOX of OBSTACLES
%     fig_1_ObstBB  = plotObstacles(obstacles.obstacles(:,3), 0 ,{'--r',0.5});
    for index = 1:numel(obstacles.obstacles(:,1))
        p = obstacles.obstacles{index,1};
        objCenter(:,index) = p;
        ptsBBox = obstacles.obstacles{index,3};
        x_lim = [min(ptsBBox(1,:)),max(ptsBBox(1,:))];
        y_lim = [min(ptsBBox(2,:)),max(ptsBBox(2,:))];
        objBB(:,:,index) = [x_lim;y_lim];
        plot(p(1),p(2),'x','markersize',5);
        text(p(1),p(2),num2str(index),'Color','b')
    end
    
    % init print variablle
    fov_plot = [];
    objPolyFillHandle = 0;
    objBBHandle = 0;
    fovHandle = 0;
    fovOnBoardHandle = 0;
    segmentInters = 0;
    segmentSelected = 0;
    
    figure(fig_id);
    subplot(2,1,1);
    hold on;
    vehicleInFOVHandle = plot([],[]);
    allVehicleHandle = plot([],[]);
    segmentInters = plot([],[]);
    segmentSelectedonHandle = plot([],[]);
    axis equal;
    xlim(obstacles.xRange);
    ylim(obstacles.yRange);
    axis manual;
    
    figure(fig_id);
    subplot(2,2,4);
    hold on;
    title('particle POV');
    fovOnBoardHandle = plotFOV( FOV,fovOnBoardHandle );
%     delete(fovOnBoardHandle(2));
    segmentSelectedonBoardHandle = plot([],[]);
    segmentIntersonBoardHandle = plot([],[]);
    vehicleInFOVHandleonBoard = plot([],[]);
    xlim([-10,30]);
    ylim([-10,10]);
    axis equal;
    axis manual
    
    figure(fig_id);
    subplot(2,2,3);
    pos = get(subplot(2,2,3),'position');
    posSegment = [pos(1)+pos(3)/8,pos(2)+pos(4)/2*1.1, pos(3)*7/8, pos(4)/3*1.05];
    posPersUI = [pos(1)+pos(3)/8,pos(2), pos(3)*7/8, pos(4)/3*1.05];
    posX = pos(1)+pos(3)/2;
    posY = pos(2)+pos(4)/2*1.2;
    sizeX = 0.5;
    sizeY = pos(4)/5*1.8;
    hold on;
    axis off;
    title('Data');
    mTextBox = uicontrol('style','text','units','normalized','HorizontalAlignment','left');
    mTextBox.Position = posSegment;
    message = [];
    set(mTextBox,'String',message);
    mTextBox2 = uicontrol('style','text','units','normalized','HorizontalAlignment','left');
    mTextBox2.Position = posPersUI;
    message = [];
    set(mTextBox2,'String',message);
    text(0.05,0.95,'Segment in FOV (P1,P2) global coordinate: ');
    text(0.05,0.4,'ID Particle in FOV:');
    
    
    n_particle = result.simNumber;
    vehicles = result.particleState;
    for particleID=1:result.simNumber
        if stop == 1
           break; 
        end
        vehicle_index = result.simulation{particleID}.mainParticleID;
        timeStepNum   = result.simulation{particleID}.timeStepNum;
        
        others_index = 1:n_particle;
        others_index = others_index(others_index~=vehicle_index);
        
        for t = 1:timeStepNum
            if stop == 1
                break; 
            end
            %         display([particleID,t]);
            % Path point and orientation
            p = vehicles(t,1:2,vehicle_index)';
            theta = vehicles(t,3,vehicle_index);
            
            % Load data from saved file
            LaneViewOnBoard      = result.simulation{particleID}.wallsInFov{t};
            IDparticleInFOV      = result.simulation{particleID}.particleInFovId{t};
            particleInFOVonBoard = result.simulation{particleID}.particleInFovRelativePosition{t};
            segmentOnBoard       = result.simulation{particleID}.wallsInFovRelativePosition{t};
            segment              = result.simulation{particleID}.wallsInFov{t};
            [ FOVrot ] = rotoTraslateFOV( FOV, theta, p );
            
            % Plot all the person in global system
            figure(fig_id);
            subplot(2,1,1);
            delete(allVehicleHandle);
            tmp = reshape(vehicles(t,1:2,:),2,n_particle);
            allVehicleHandle = plot(tmp(1,:),tmp(2,:),'k.','markersize',10);
            
            % Highlight the person in FOV (absolute coordinate)
            delete(vehicleInFOVHandle);
            subplot(2,1,1);
            fovHandle = plotFOV( FOVrot,fovHandle );
            n = numel(IDparticleInFOV);
            personInFOV = reshape(vehicles(t,1:2,IDparticleInFOV),2,n);
            vehicleInFOVHandle = plot(personInFOV(1,:),personInFOV(2,:),'g.','markersize',10);
            
            
            % Highlight person (Relative coordinate)
            subplot(2,2,4);
            delete(vehicleInFOVHandleonBoard);
            vehicleInFOVHandleonBoard = plot(particleInFOVonBoard(1,:),particleInFOVonBoard(2,:),'g.','markersize',10);
            
            % Plot the segment of wall not occluded
            delete(segmentSelectedonBoardHandle);
            delete(segmentSelectedonHandle);
            profondita = numel(segmentOnBoard)/4;
            message = [];
            if profondita>0
                % Particle coordinate system
                tmpX = reshape(segmentOnBoard(1,:,:),2,profondita);
                tmpY = reshape(segmentOnBoard(2,:,:),2,profondita);
                subplot(2,2,4);
                segmentSelectedonBoardHandle = plot(tmpX,tmpY,'b','linewidth',2);
                
                % Global coordinate system
                tmpX = reshape(segment(1,:,:),2,profondita);
                tmpY = reshape(segment(2,:,:),2,profondita);
                subplot(2,1,1);
                segmentSelectedonHandle = plot(tmpX,tmpY,'b','linewidth',2);                
                for kkk=1:profondita
                    message = [message,sprintf('[%.2f,%.2f], [%.2f,%.2f] \n',segment(1,1,kkk),segment(2,1,kkk),segment(1,2,kkk),segment(2,2,kkk))];
                end
            else
                segmentIntersonBoardHandle = plot([],[]);
                segmentInters = plot([],[]);
            end
            set(mTextBox,'String',message);
            
            % Message of particle in FOV
            message = [];
            for kkk=1:numel(IDparticleInFOV)
                message = [message,sprintf('%4.d, ',IDparticleInFOV(kkk))];
                if(rem(kkk,3)==0)
                    message = [message,sprintf('\n')];
                else
                    message = [message,sprintf('\t')];
                end
            end
            set(mTextBox2,'String',message);
            
            drawnow;
            if exportVideoFlag
                frame = getframe(fig_id);
                writeVideo(animation,frame);
            else
                %             pause();
                %pause(0.1);
            end
            
        end
    end
    
    
    
    if exportVideoFlag
        close(animation);
    end
else
    fprintf('%d) No Plots \n',sec_numb); sec_numb = sec_numb + 1;
end


%**************************************************************************
%% Load only the intresting parameters the results
%**************************************************************************
fprintf('%d) Load only the useful data \n',sec_numb); sec_numb = sec_numb + 1;

clear all;

% Load saved variable
load('./Data/resultDemo');
load('./Data/obstaclesDemo');   % to plot the map
load('./Data/fovDemo');         % to plot the FOV