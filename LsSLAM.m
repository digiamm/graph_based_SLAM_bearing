close all
clear
clc

addpath '../'
addpath './g2o_wrapper'
addpath '../tools/visualization'
source "./geometry_helpers_2d.m"
source "./total_least_squares.m"


% load your own dataset
[landmarks, poses, transitions, observations] = loadG2o("./datasets/slam-2d-bearingOnly.g2o");

% retrieve number of landmarks and poses from dataset
num_landmarks_true = size(landmarks,2);
num_poses = size(poses,2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%% LANDMARKS GROUND TRUTH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% retrieve position and ID of landmarks in World RF (order in vector is not the ID order)
XL_true = zeros(2, num_landmarks_true);
global id_landmarks = zeros(1,num_landmarks_true);
for (l=1:num_landmarks_true)
 	XL_true(:,l) = [landmarks(l).x_pose; landmarks(l).y_pose];			% landmark position
	id_landmarks(1,l) = landmarks(l).id;							% landmark ID
endfor;


% % retrieve pose-landmark measurements and association vector (covariance is the same for each landmark observation)
% num_landmark_measurements = size(observations,2);				% retrieve the number of landmark measurements
% ZL=zeros(1,num_landmark_measurements);
% landmark_associations=zeros(2,num_landmark_measurements);
% for (meas=1:num_landmark_measurements)				% retrieve observations
% 	obs = observations(meas);
% 	landmark_associations(:,meas)=[obs.pose_id; obs.land_id];	%1 pose_id, 2 land_id
% 	ZL(:,meas)=[obs.bearing];
% endfor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% retrieve pose-pose measurements and association vector (covariance is the same for each pose observation)
num_pose_measurements = size(transitions,2);				% retrieve the number of pose measurements
ZIJ=zeros(3,3,num_pose_measurements);
pose_associations=zeros(2,num_pose_measurements);
for (meas=1:num_pose_measurements)				% retrieve observations
	  poseobs = transitions(meas);
    tij = [poseobs.x; poseobs.y; poseobs.theta];
	  ZIJ(:,:,meas) = v2t(tij);
  	pose_associations(:,meas)=[poseobs.id_from; poseobs.id_to];
endfor


% initial odometry guess
mu = [0; 0; 0];
XR_guess = zeros(3, 3, num_poses);
global id_poses = zeros(1, num_poses);
global poses_id = ones(1, 2000)*-1;
% add starting position
XR_guess(:,:,1) = v2t(mu);

% adjust mapping
id_poses(1, 1) = transitions(1).id_from;
poses_id(1, transitions(1).id_from) = 1;


for(t=1:length(transitions))
  transition = transitions(t);
  % adjust mapping
  id_poses(1, t+1) = transition.id_to;
  poses_id(1, transition.id_to) = t+1;

  displacement = [transition.x; transition.y; transition.theta];
  mu = transition_model(mu, displacement);
  XR_guess(:, :, t+1) = v2t(mu);
endfor

% reconstruct position of landmarks
landmarks_guess  = reconstruct_landmark_position(observations, XR_guess);

% get number of landmarks reconstructed, not all of the landmarks can be reconstructed using a bering sensor
num_landmarks = length(landmarks_guess);

% retrieve position and ID of landmarks in World RF (order in vector is not the ID order)
XL_guess = zeros(2, num_landmarks);
global id_landmarks = zeros(1, num_landmarks);
for (l=1:num_landmarks)
 	XL_guess(:, l) = [landmarks_guess(2, l); landmarks_guess(3, l)];			% landmark position
	id_landmarks(1, l) = landmarks_guess(1, l);							% landmark ID
endfor;


% get maximum number of observation
num_landmark_measurements = size(observations, 2);
% since we don't know how many obsersation we will use
% depends on the number of landmarks reconstructed
% we will append to the lists dynamically
% hence intialize the lists to zero
ZL=zeros(1, 0);
landmark_associations=zeros(2, 0);
% counter for the useful measurements
counter = 1;
for (meas=1:num_landmark_measurements)
	obs = observations(meas);
  for t=1:id_landmarks
    % if land id is inside the reconstructed landmark ids
    % the measurement is useful
    if(obs.land_id == id_landmarks(t))
    	landmark_associations(:,end+1)=[obs.pose_id; obs.land_id];	%1 pose_id, 2 land_id
    	ZL(:,end+1)=[obs.bearing];
      counter++;
    endif
  endfor
endfor


% optimization least square
damping=1e-6;
kernel_threshold=1e-1;
num_iterations=25;	% 10, 25
[XR, XL, chi_stats_l, num_inliers_l, chi_stats_r, num_inliers_r, H, b]= doTotalLS(XR_guess, XL_guess,
																					ZL, landmark_associations,
																					ZIJ, pose_associations,
																					num_poses,
																					num_landmarks,
																					num_iterations,
																					damping,
																					kernel_threshold);



% Robot position ground truth
XR_true= zeros(3,3,num_poses);
global id_poses= zeros(1,num_poses);

for (pos=1:num_poses)
	gt = poses(pos);
	t = [gt.x; gt.y; gt.theta];
	XR_true(:,:,pos) = v2t(t);
	id_poses(1, pos) = gt.id;
endfor



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
hold on;
grid;
title("Robot poses");
plot(XR_true(1,:),XR_true(2,:),'r+',"linewidth",2);
hold on;
plot(XR_guess(1,:),XR_guess(2,:),'g+',"linewidth",2);
hold on;
plot(XR(1,:),XR(2,:),'b+',"linewidth",2);
legend("Ground Truth", "Guess", "Final");grid;


figure(2);
hold on;
grid;
title("Landmarks poses");
plot(XL_true(1,:), XL_true(2,:),'ro',"linewidth",2);
hold on;
plot(XL_guess(1,:), XL_guess(2,:),'g*',"linewidth",2);
hold on;
plot(XL(1,:), XL(2,:),"bs","linewidth",2);
legend("Ground Truth","Guess","Final");grid;

figure(3)
hold on;
grid;
title("chi evolution");

subplot(2,1,1);
plot(chi_stats_r, 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");

subplot(2,1,2);
plot(chi_stats_l, 'b-', "linewidth", 2);
legend("Chi Landmarks"); grid; xlabel("iterations");



figure(4);
title("H matrix");
H_ = H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
