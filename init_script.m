% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

% camera calibration
K = [314.1779 0         199.4848; 
    0         314.2218  113.7838; 
    0         0         1];


% vel tracker init
estimate_vel;
eskf1;
eskf2;
eskf1_handle = @(sensor, vic) eskf1(sensor, vic, K);
eskf2_handle = @(sensor) eskf2(sensor, K);
