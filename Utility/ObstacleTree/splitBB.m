function [ BB_range, Leaf, Padre, id ] = splitBB( BB_range, objActive,orient, objBB, objCenter, id,padre)
%SPLITBB Summary of this function goes here
%   Detailed explanation goes here
% global debPlot;
% global obstacles
% 
% if numel(objActive)>1
%     tmp1 = obstacles(:,2);
% 
%     figure(debPlot.figId);
% %     delete(debPlot.fill);
% 
%      for k=debPlot.selectObj
%          delete(k)
%      end
% 
%     debPlot.fill = fill(BB_range(1,[1,2,2,1]),BB_range(2,[1,1,2,2]),'r','facealpha',0.1);
%     debPlot.selectObj = [];
%     for i=objActive
%          xV = tmp1{i,1}(1,:);
%          yV = tmp1{i,1}(2,:);    
%          debPlot.selectObj = [debPlot.selectObj, plot(xV,yV,'g')];
%     end
%     drawnow;
%     pause();
% end

orient = rem(orient+1,2);
nObj = numel(objActive);
if nObj < 3
    
    switch nObj
        case 0
            BB_range = [];  
            Leaf = [];
            Padre = [];
            
        case 1
            BB_range = cat(3,createBoundingBox( objBB, objActive ));
            Leaf = objActive; 
            Padre = padre;
            %display(['ID leaf: ', num2str(id(end))]);
            id = [id,id(end)+1]; 
        case 2
             BB_range =  cat(3, createBoundingBox( objBB, objActive(1) ), ...
                                createBoundingBox( objBB, objActive(2) ) ...
                              );
             Leaf = [objActive(1), objActive(2)];
             Padre = [padre,padre];
             %display(['ID leaf: ', num2str(id(end))]);
             id = [id,id(end)+1]; 
             %display(['ID leaf: ', num2str(id(end))]);
             id = [id,id(end)+1]; 
        otherwise
            error('splitBB: Something bad happend!');                    
    end
else
    myId = id(end);
    id = [id,id(end)+1];      
    
    if orient==0
        meanX = (BB_range(1,1)+BB_range(1,2))/2;
        logic = objCenter(1,objActive) < meanX;
    else
        meanY = (BB_range(2,1)+BB_range(2,2))/2;
        logic = objCenter(2,objActive) < meanY;
    end

    objActive1 = objActive(logic);    
    BB_range1_tmp = createBoundingBox( objBB, objActive1 );
    
    objActive2 = objActive(not(logic));
    BB_range2_tmp = createBoundingBox( objBB, objActive2 );  
    [BB_range1, Leaf1, Padre1, id]  = splitBB(BB_range1_tmp, objActive1,orient,objBB, objCenter,  id, myId);    
    [BB_range2, Leaf2, Padre2, id]  = splitBB(BB_range2_tmp, objActive2,orient,objBB, objCenter,  id, myId);
    
    Padre  = [padre,Padre1 , Padre2];    
    Leaf = [0, Leaf1, Leaf2];   % if is not a leaf add 0    
    %Leaf = [Leaf, Leaf1, Leaf2];    
    BB_range = cat(3,BB_range,BB_range1,BB_range2);
end

end

