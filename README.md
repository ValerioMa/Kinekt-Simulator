# Kinekt-Simulator
Matlab simulation of an aget moving in an environment 


 “result.particleState" is a 3D matrix.
	- The last index, is the ID of the agent, aka particle;
		- Example: “M = result.particleState(:,:,n)” is matrix containing the data corresponding to the n-th agent;
	- Then, “M(1,:)” is a row vector containing three equal entries corresponding to the maximum time step value containing a valid data.
		- Example: “M(1,:) = [i,i,i]” mean that the last valid data, i.e. Before the agent stops, is the i-th
	- Hence, “M(j+1,:) = [x,y,theta]” is a row vector containing the position and orientation of the particle at the j-th time instant, with j <= i

- “result.simulation": is a cell structure
	- “result.simulation{k}” is a walker for which a simulation with camera readings is recorded. In practice, if you have m agents, you will have a subset of h <= m for which the camera readings are recorded. Basically, k = 1,…,h
	- “result.simulation{k}.mainParticleID” is the ID of the agent considered. Basically is the index n considered above. If you want to get access to the coordinates of the walker in time, you can write: "result.particleState(:,:,result.simulation{k}.mainParticleID)”
	- "result.simulation{k}.timeStepNum” corresponds to the maximum time step for which the robot moves (then it stops because it reaches its destination. This is the same value stored in the first row of M defined previously, i.e. if “M = result.particleState(:,:,result.simulation{k}.mainParticleID)”, then “M(1,:)=[result.simulation{k}.timeStepNum, result.simulation{k}.timeStepNum, result.simulation{k}.timeStepNum]”.
	- "result.simulation{k}.wallsInFov” is a cell array of length "result.simulation{k}.timeStepNum”, containing the segments of the walls in view. Each segment is expressed in the agent “result.simulation{k}.mainParticleID” reference frame, that is with the ‘x’ axis (first coordinate) pointing forward and the ‘y’ axis (second coordinate) pointing on the left. Each segment is defined as a 2x2 matrix, whose first row are the ‘x’ and ‘y’ coordinates of the starting point, while the second row are the ‘x’ and ‘y’ coordinates of the ending point of the segment, expressed in the local agent coordinates. If multiple wall segments are in view at time step l <= result.simulation{k}.timeStepNum, the entry "result.simulation{k}.wallsInFov{l}” contains a 3D matrix, whose third entry has a dimension equals to the number of wall segments in view.
	- "result.simulation{k}.particleInFovId” is a cell array similar to "result.simulation{k}.wallsInFov” but containing an array containing the IDs of the agents currently in view. To know the coordinates, in the global frame, of the p-th agent in view at the l-th time step, one has to first elect its ID, i.e. “id = result.simulation{k}.particleInFovId{l}(p)”, then the maximum valid entry (before it stops), i.e. “MaxVal = result.particleState(1,:,id)”, and then take the minimum to retrieve the valid coordinates (since after it sops the array has a void number), i.e. “Coordinates = result.particleState(min(MaxVal, l)+1,:,id)”
