function [ LaneView, WallSegmentsInFOV,WallObstacleIdS ] = findWallsIntersection( FOV,obstacle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

TOL = 1e-8;
DEBUG = 1*0;

WallObstacleIdS   = polygonBBinObstacleBB(FOV.limits,obstacle.obstaclesTree);
[ WallSegmentsInFOV ]  = PolygonsSegmentBBInsideBB( obstacle.obstacles(WallObstacleIdS,2), FOV.limits );
if ~isempty(WallSegmentsInFOV)
    [ LaneView ] = segmentsTriangFOVintersec( WallSegmentsInFOV, FOV);
    LaneView_old = LaneView;
    
    % Find all'segment intersection and eventually split segment
    n_linee = numel(LaneView)/4;
    i = 0;
    while i<n_linee
        i = i + 1;
        segment1 = LaneView(:,:,i);
        minX1 = min(segment1(1,:));
        maxX1 = max(segment1(1,:));
        minY1 = min(segment1(2,:));
        maxY1 = max(segment1(2,:));
        for k=i:n_linee
            segment2 = LaneView(:,:,k);
            minX2 = min(segment2(1,:));
            maxX2 = max(segment2(1,:));
            minY2 = min(segment2(2,:));
            maxY2 = max(segment2(2,:));
            
            if minX1<maxX2 && minX2<maxX1 && minY1<maxY2 && minY2<maxY1
                % Check intersection
                [ out, lambda1, lambda2 ] = segmentIntersection( segment1,segment2 );
                % If an intersection is found
                if ~isempty(out)
                    % if intersection split vector 1
                    if lambda1>TOL && lambda1< (1-TOL)
                        % Split segment 1
                        segment1A = [segment1(:,1),out];
                        segment1B = [out,segment1(:,2)];
                        LaneView(:,:,i) = segment1A;
                        LaneView(:,:,end+1) = segment1B;
                        n_linee = n_linee + 1;
                        segment1 = LaneView(:,:,i);
                    end
                    
                    % if intersection split vector 2
                    if lambda2>TOL && lambda2< (1-TOL)
                        % Split segment 2
                        segment2A = [segment2(:,1),out];
                        segment2B = [out,segment2(:,2)];
                        LaneView(:,:,k)     = segment2A;
                        LaneView(:,:,end+1) = segment2B;
                        n_linee = n_linee + 1;
                    end
                end
            end
        end
    end
    
    % DEBUG PLOT
    if DEBUG
        figure(10); clf;
        hold on;
        profondita = numel(LaneView)/4;
        profondita_old = numel(LaneView_old)/4;
        tmpX = reshape(LaneView(1,:,:),2,profondita);
        tmpY = reshape(LaneView(2,:,:),2,profondita);
        plot(tmpX,tmpY,'b*','linewidth',2);
        tmpX = reshape(LaneView_old(1,:,:),2,profondita_old);
        tmpY = reshape(LaneView_old(2,:,:),2,profondita_old);
        plot(tmpX,tmpY,'g','linewidth',2);
        a = plot([],[]);
        ac = plot([],[]);
    end
    
    if n_linee~=0
        % Order segment point by angle and then radius
        data = zeros(4,n_linee*2);
        for i=1:n_linee*2
            laneIndx = idivide(i-1,uint8(2))+1;
            col = rem(i-1,2)+1;
            raggio =  LaneView(:,col,laneIndx) - FOV.p0;
            normSq = raggio(1)^2+raggio(2)^2;
            theta = atan2( raggio(2),raggio(1));
            while theta > (FOV.theta + FOV.alpha/2)
                theta = theta - 2*pi;
            end
            
            while theta < (FOV.theta - FOV.alpha/2*2)
                theta = theta + 2*pi;
            end
            
            data(1,i) = theta;
            data(2,i) = normSq;
            data(3,i) = laneIndx;
            data(4,i) = col;
        end
        
        
        
        dataSort = sortrows(data')';
        
        % DEBUG PLOT  
        if DEBUG
            for currentLaneIndx=1:numel(LaneView)/4
                p = mean(LaneView(:,:,currentLaneIndx),2);
                text(p(1),p(2),num2str(currentLaneIndx))
            end
        end
        
        LaneActiveIndx = [];
        LaneViewedIndx = [];  % 0 means no lane;
        
        % Insert first vector in view
        sortIndx = 1;
        while sortIndx <= (n_linee*2)
            %         for sortIndx = 1:n_linee*2
            currentLaneIndx = dataSort(3,sortIndx);
            theta = dataSort(1,sortIndx);
            theta_ref = theta;
            
            
            while sortIndx <= (n_linee*2)
                currentLaneIndx = dataSort(3,sortIndx);
                theta_tmp = dataSort(1,sortIndx);
                
                % Control if it's not the same theta as before
                if(abs(theta_tmp-theta_ref)>1e-8) 
                    break;
                end
                
                theta = theta_tmp;
                
%                 % Theta of end of the vector
%                 if dataSort(4,sortIndx)==1
%                     col = 2;
%                 else
%                     col = 1;
%                 end
                
                % Find the index
                sortIndxEndPoint = find(dataSort(3,:)==currentLaneIndx);
                % Order the index -> first is the current index second is
                % the future index
                sortIndxEndPoint = [sortIndxEndPoint(sortIndxEndPoint==currentLaneIndx),sortIndxEndPoint(sortIndxEndPoint~=currentLaneIndx)];
                tmp = dataSort(:,sortIndxEndPoint);
                sortIndxEndPoint = 2;  %find(tmp(4,:)==col);
                sortIndxStartPoint = 1;% find(tmp(4,:)~=col);
                thetaEnd = tmp(1,sortIndxEndPoint);
                thetaStart = tmp(1,sortIndxStartPoint);
                if thetaEnd>(theta+TOL)   % add lane to active Lane
                    LaneActiveIndx = [LaneActiveIndx,double([0;thetaStart;thetaEnd;currentLaneIndx])]; % 0 is fake
                end
                
                sortIndx = sortIndx + 1;
            end
            
            %versor = [FOV.p0,FOV.p0+[cos(max(LaneActiveIndx(2,:)));sin(max(LaneActiveIndx(2,:)))]];
            versor = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]];
            % TODO: find a better way to define theta
            
            
            
            % Compute distance
            for k=1:numel(LaneActiveIndx)/4
                [ ~,lambda1, ~ ] = segmentIntersection( versor,LaneView(:,:,LaneActiveIndx(4,k)));
                LaneActiveIndx(1,k) = lambda1;
            end
            
            if ~isempty(LaneActiveIndx)
                % Order the LaneActiveIndx by the distance
                LaneActiveIndx = sortrows(LaneActiveIndx')';
                
                logic = abs( LaneActiveIndx(1,:)-LaneActiveIndx(1,1))<TOL;
                if sum(logic)<2                    
                    % possible intersection
                    LaneViewedIndx = [LaneViewedIndx,[LaneActiveIndx(4,1);theta]];
                else  % Distance is equal                    
                    maxTheta      = max(LaneActiveIndx(3,:));
                    if maxTheta>(theta+TOL)
                        logic = LaneActiveIndx(3,:)>(theta+TOL);
                        LaneActiveIndx1 = LaneActiveIndx(:,not(logic));
                        LaneActiveIndx2 = LaneActiveIndx(:,logic);
                        
                        theta2 = min(LaneActiveIndx2(3,:));
                        versor = [FOV.p0,FOV.p0+[cos(theta2);sin(theta2)]];
                        for k=1:numel(LaneActiveIndx2)/4
                            [ ~,lambda1, ~ ] = segmentIntersection( versor,LaneView(:,:,LaneActiveIndx2(4,k)));
                            LaneActiveIndx2(1,k) = lambda1;
                        end
                        LaneActiveIndx = [sortrows(LaneActiveIndx2')',LaneActiveIndx1];
                    else
%                         LaneActiveIndx = LaneActiveIndx(:,LaneActiveIndx(2,:)<(theta-TOL)); % possiblle to remove
                        theta2 = max(LaneActiveIndx(2,:));
                        versor = [FOV.p0,FOV.p0+[cos(theta2);sin(theta2)]];
                        for k=1:numel(LaneActiveIndx)/4
                            [ ~,lambda1, ~ ] = segmentIntersection( versor,LaneView(:,:,LaneActiveIndx(4,k)));
                            LaneActiveIndx(1,k) = lambda1;
                        end
                        LaneActiveIndx = sortrows(LaneActiveIndx')';
                    end
                    
                    % the lane viewd is the first one
                    LaneViewedIndx = [LaneViewedIndx,[LaneActiveIndx(4,1);theta]];
                end
                
            end
            
            % DEBUG PLOT
            if DEBUG
                delete(a);
                delete(ac);
                tmp = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r];
                a = plot(tmp(1,:),tmp(2,:),'r--');
                ac = plot(FOV.p0(1,:),FOV.p0(2,:),'r*');
                display(LaneActiveIndx(4,:));
            end
            
            left_indexes = 1:numel(LaneActiveIndx)/4;
            for k=1:numel(LaneActiveIndx)/4
                thetaEnd = LaneActiveIndx(3,k);
                if thetaEnd<=theta% eventually remove the lane
                    left_indexes = left_indexes(left_indexes~=k);
                end
            end
            LaneActiveIndx = LaneActiveIndx(:,left_indexes);
        end
        
        % HELP MEEEEE
        indx = LaneViewedIndx(1,1);
        estremiSeg = find(dataSort(3,:)==indx);
        startPoint = LaneView(:,dataSort(4,estremiSeg(1)),dataSort(3,estremiSeg(1)));
        LaneViewNew = [];
        indxPrev = indx;
        for i=1:numel(LaneViewedIndx)/2
            indx = LaneViewedIndx(1,i);
            if (indx==indxPrev && indx<numel(LaneViewedIndx)/2)
                continue;
            end
            
            estremiSegPrev = find(dataSort(3,:)==indxPrev);
            estremiSeg     = find(dataSort(3,:)==indx);
            
            if dataSort(1,estremiSegPrev(2))<=dataSort(1,estremiSeg(1)) % prev finisce prima inizio seg attuale
                endPoint = LaneView(:,dataSort(4,estremiSegPrev(2)),dataSort(3,estremiSegPrev(2)));
                LaneViewNew = cat(3,LaneViewNew,[startPoint,endPoint]);
                startPoint = LaneView(:,dataSort(4,estremiSeg(1)),dataSort(3,estremiSeg(1)));
            else
                % Find the end and the begin of the segment intersecting with
                % the ray
                
%                theta = dataSort(1,estremiSegPrev(2));
                theta = dataSort(1,estremiSeg(1));
                versore = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r];                               
                
                % Ray intersection
                [ endPoint,~, ~ ] = segmentIntersection( versore,LaneView(:,:,indxPrev));
                
                % Is the other case
                if numel(endPoint) == 0
                    theta = dataSort(1,estremiSegPrev(2));
                    versore = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r]; 
                    [ endPoint,~, ~ ] = segmentIntersection( versore,LaneView(:,:,indxPrev));
                end
                LaneViewNew = cat(3,LaneViewNew,[startPoint,endPoint]);
                
%                 theta = LaneViewedIndx(2,i);
%                 versore = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r];
                [ startPoint,~, ~ ] = segmentIntersection( versore,LaneView(:,:,indx));
            end
            
%             DEBUG PLOT        
            if DEBUG
                delete(a);
                delete(ac);
                theta = dataSort(1,estremiSeg(1));
                versore = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r]; 
                a = plot(versore(1,:),versore(2,:),'r--');
                ac = plot(FOV.p0(1,:),FOV.p0(2,:),'r*');
                profondita = numel(LaneViewNew)/4;
                tmpX = reshape(LaneViewNew(1,:,:),2,profondita);
                tmpY = reshape(LaneViewNew(2,:,:),2,profondita);
                plot(tmpX,tmpY,'c*');
            end
            indxPrev = indx;
        end
        
        % Control last point
        % project begin of segment on the other segment
        theta = LaneViewedIndx(2,i);
        
        versore = [FOV.p0,FOV.p0+[cos(theta);sin(theta)]*FOV.r];
        [ endPoint,~, ~ ] = segmentIntersection( versore,LaneView(:,:,indxPrev));
        LaneViewNew = cat(3,LaneViewNew,[startPoint,endPoint]);
        
        LaneView = LaneViewNew;
        %     LaneViewedIndx = unique(LaneViewedIndx);
        %     LaneView = LaneView(:,:,LaneViewedIndx);
    else
        LaneView = [];
    end
else
    LaneView = [];
end

end

