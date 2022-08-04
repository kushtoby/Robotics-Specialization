% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)



% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;   						% Please decide a reasonable number of M, 
                               	% based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
Sigma = diag([0.1 0.1 0.3]);
% R = chol(Sigma);
% R = diag([0.01 0.01 pi/6]);
% Adding initialize noise
% P = P + (randn(size(P')) * R)';

theta_mu = 0;
theta_sigma = pi/7;
encoder_mu = 0;
encoder_sigma = 0.05;
% system noise 
R = diag([0.005 0.005 0.0005]);

occScore = 10;
occMapValue = 0.5;

freeScore = 2;
freeMapValue = -0.2;
w = ones(1, M) * (1/M);

figure, 
h = imagesc(map);
colormap('gray'); axis equal;
hold on;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 

    % Get odometry and encoder
    theta = random('Normal', theta_mu, theta_sigma);
    encoder = random('Normal', encoder_mu, encoder_sigma);
    
    % rot = [ cos(theta)  -sin(theta)  encoder;
    %         sin(theta)  cos(theta)   0;
    %         0           0            1];
    % P = rot * P;
    % Adding system noise
    % P = P + (randn(size(P')) * R)';
    
    % P = P + mvnrnd([0 0 0], R, M)';
    P = repmat(myPose(:,j-1), [1, M]) + mvnrnd([0 0 0], R, M)';

    corrP = [];
    % 2) Measurement Update 
	for k = 1:M    
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
        pose = P(:, k);
  %       pose(3) = theta + pose(3);
	 %    pose(1) = pose(1) + cos(pose(3)) * encoder;
	 %    pose(2) = pose(2) + sin(pose(3)) * encoder;
		% pose = pose + randn(size(pose)) .* diag(R);
		% P(:, k) = pose;
		
    	% compute occLoc for each P_k based on observer range from step j 
    	realLoc = [ranges(:, j) .* cos(scanAngles + pose(3)), -ranges(:,j) .* sin(scanAngles + pose(3))]' + repmat(pose(1:2), [1 numel(scanAngles)]);
  		occLoc = ceil(realLoc .* myResolution) + repmat(myOrigin, [1 numel(scanAngles)]);  	
    	% occLoc = unique(occLoc', 'rows')'; % find unique occ Location to reduce computation load

    %   2-2) For each particle, calculate the correlation scores of the particles
%         max(occLoc(1, :))
%         max(occLoc(2, :))
        outOfRangeXIdx = find(occLoc(1, :) < 1 | occLoc(1, :) > size(map, 2));
        outOfRangeYIdx = find(occLoc(2, :) < 1 | occLoc(2, :) > size(map, 1));
    	outOfRange = unique([outOfRangeXIdx, outOfRangeYIdx]);
    	occLoc(:, outOfRange) = [];
        
    	occInd = sub2ind( size(map), occLoc(2, :), occLoc(1, :));
    	cellHit = map(occInd) > occMapValue;
    	cellMiss = map(occInd) < freeMapValue;
        % corrPoint = sum(cellHit) * occScore + sum(cellMiss) * freeScore;
    	corrPoint = sum(map(cellHit) * occScore) + sum(map(cellMiss) * freeScore);
    	corrP = [corrP, corrPoint];		    		
    end

    %   2-3) Update the particle weights   
    corrP(corrP < 0) = 0;
 	% w = w .* corrP;
    w = corrP;
    w = w / sum(w);
    %   2-4) Choose the best particle to update the pose
    [~, bestId] = max(w);
    myPose(:, j) = P(:, bestId);
    % 3) Resample if the effective number of particles is smaller than a threshold
    % w2 = w;
    % w2(w<0) = 0;
    n_effective = sum(w)^2 / sum(w.^2);

    if (n_effective < (0.5 *  M))
    	[new_sample] = resamplingWheel(w);
    	P = P(:, new_sample);
    	w = w(new_sample);
        % w = ones(1, M) * (1/M);
        w = w / sum(w);
    end    
    % 4) Visualize the pose on the map as needed
    poseLoc = ceil(myPose(1:2, j) .* myResolution) + myOrigin;
    plot(poseLoc(1), poseLoc(2), 'r.');
    drawnow

end

end