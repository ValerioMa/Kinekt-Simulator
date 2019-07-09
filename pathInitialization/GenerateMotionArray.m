function [ particlesStates,vehicle_indexes ] = GenerateMotionArray( obstacles,AgentNumb,scenario)
%GENERATEMOTIONARRAY generate a 3D matrix of state of particle.
% - particlesStates(1,1,particleID) we get the number of timestep (durate) of the
% total motion of particleID
% - particlesStates(timestep+1,:,particleID) we get the state [x,y,theta] of the
% particle "particleID" at time "timestep".
vehicle_indexes = 1:AgentNumb;

switch scenario
    case 1
        rRange = [52,67];        
        timestep = 800;
        particlesStates = simpleMapPathGenerator(rRange,AgentNumb,timestep);
    case 2
        particlesStates = povoMapPathGenerator(obstacles);
        particlesStates(1,2:3:end) = AgentNumb;
        particlesStates = particlesStates(:,:,vehicle_indexes);
    otherwise
        error('Scenario not implemented');
end
       

end

