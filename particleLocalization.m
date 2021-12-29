% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = NaN(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
np = 1000;                           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, np]);    % particle cloud
w = ones(1, np) ./ np;                % weights
wLin = log(w); 
nMeas = length(scanAngles);

% particle fileter parameters
% coarse prediction
sigYawRate = 3 * pi/180;     % 3 deg
sigX = .05;                   % 0.2 m
sigY = .05;                   % 0.2 m

% Kalman filter to estimate velocity
% vInit = [0; 0];
dt = 1.0;   % normalized
% kf = struct;
% kf.A = eye(4);
% kf.A(1,3) = dt;
% kf.A(2,4) = dt;
% kf.Q = diag([.01^2 .01^2 .1^2 .1^2]);
% kf.C = zeros(2, 4);
% kf.C(1,1) = 1;
% kf.C(2,2) = 1;
% kf.R = diag([.1^2 .1^2]);
% kf.X = [myPose(:,1); vInit];
% kf.P = diag([10 10 1 1]);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% for j = 2
    % 1) Propagate the particles
%     P = repmat(myPose(:,j-1), [1, M]);
    P(1,:) = P(1,:) + randn([1, np]) * dt * sigX;
    P(2,:) = P(2,:) + randn([1, np]) * dt * sigY;
    P(3,:) = P(3,:) + randn([1, np]) * dt * sigYawRate;
    % TODO: prediction using motion model
%     s = sqrt(kf.X(3)^2 + kf.X(4)^2);
%     sigS = kf.P(3,3) + kf.P(4,4);
%     heading = atan2(kf.X(4), kf.X(3));
%     P = Ackermann(P, 0, 1.0, s, sigS, heading, sigYawRate, 1.0);
      
    % 2) Measurement Update
    for p = 1:np
        particle = P(:,p);
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
%         lidar_global(:,1) =  ceil(ranges(:,j).*cos(-scanAngles + particle(3)) + particle(1))*param.resol + param.origin(1);
%         lidar_global(:,2) =  ceil(ranges(:,j).*sin(-scanAngles + particle(3)) + particle(2))*param.resol + param.origin(2);
%         gridOcc = lidar_global';
        posOcc = repmat(particle(1:2), 1, nMeas) + repmat(ranges(:,j), 1, 2)' .* [cos(-scanAngles' + particle(3)); sin(-scanAngles' + particle(3))];
        gridOcc = repmat(myOrigin, 1, nMeas) + ceil(myResolution * posOcc); % replace myOrigin

    %   2-2) For each particle, calculate the correlation scores of the particles
        score = 0.0;
        for o = 1:size(gridOcc, 2)
            if gridOcc(2,o) > size(map, 1) || gridOcc(2,o) < 1 || gridOcc(1,o) > size(map, 2) || gridOcc(1,o) < 1
               continue
            elseif map(gridOcc(2,o), gridOcc(1,o)) > 1.0 % plot histogram of map
                score = score + 10;
            elseif map(gridOcc(2,o), gridOcc(1,o)) < 0.0 % plot histogram of map
                score = score - 5;
            end
        end
        score = score / size(gridOcc, 2);

    %   2-3) Update the particle weights
%         w(p) = w(p) + score;
        wLin(p) = wLin(p) + score;
        w(p) = exp(wLin(p));
    end
 
    %   2-4) Choose the best particle to update the pose
    [~, idx] = max(w);
    myPose(:,j) = P(:,idx);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    w = w ./ sum(w);
    Neff = 1/sum(w.^2);
    if Neff < 2/3 * np
%         fprintf('Resampling');
        [P, w, ~] = resample(P, w);
    end
    wLin = log(w);

    % 4) Visualize the pose on the map as needed
%     if j == 2
%         load practice.mat
%         figure;
%         imagesc(M); hold on;
%         plot(P(1,:) * param.resol + param.origin(1), P(2,:) * param.resol + param.origin(2), 'g.');
%         plot(P(1,idx) * param.resol + param.origin(1), P(2,idx) * param.resol + param.origin(2), 'r.');
%         posOcc = repmat(P(1:2,idx), 1, nMeas) + repmat(ranges(:,j), 1, 2)' .* [cos(-scanAngles' + P(3,idx)); sin(-scanAngles' + P(3,idx))];
%         gridOcc = repmat(myOrigin, 1, nMeas) + ceil(myResolution * posOcc);
%         plot(gridOcc(1,:), gridOcc(2,:), 'c.');
%         colormap('gray');
%         axis equal;
%     end

end

end



