function [ particlesStates ] = simpleMapPathGenerator(rRange,AgentNumb,timestep)
%SIMPLEMAPPATHGENERATOR Summary of this function goes here
%   Detailed explanation goes here

deltaT = 0.05;
particlesStates = zeros(timestep+1,3,AgentNumb);

particlesStates(1,2,1) = AgentNumb;
for a=1:AgentNumb
    particlesStates(1,1,a) = timestep;
    r = rRange(1) + diff(rRange)*rand();
    theta = rand()*2*pi;    
    if rand()-0.5 < 0
        MeanSpeed = (-10 - rand()*8)/r;
        deltaTheta = -pi/2;
    else
        MeanSpeed = (10 + rand()*8)/r;
        deltaTheta = pi/2;
    end
    
    deltaOld = 0;
    for t =2:timestep+1         
       thetaParticle = theta + deltaTheta + normrnd(0,pi/100);
       particlesStates(t,3,a) = thetaParticle;
       particlesStates(t,1:2,a) = [cos(theta),sin(theta)]*r; 
       theta = theta + deltaT*( MeanSpeed);
    end
end

end

