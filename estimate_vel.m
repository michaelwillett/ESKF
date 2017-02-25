function [vel, omg] = estimate_vel(sensor, K)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

persistent oldPoints tracker reset iter start;



vel = zeros(3,1);
omg = zeros(3,1);
iter = iter + 1;
resample = 1;


if nargin == 0
    reset = true;
    iter = 0;
    return
end

if isempty(sensor.id)
    vel = [];
    omg = [];
    return
end

if mod(iter, resample) == 0 || reset
    C = detectFASTFeatures(sensor.img);
    C = C.selectStrongest(200).Location;
    [r ~] = ind2sub(size(C),find(C<0));
    C(r,:) = [];
end
    
if reset
    tracker = vision.PointTracker();
    initialize(tracker, C, sensor.img);
    oldPoints = C;
    start = sensor.t;
    reset = false;
    return;
end

[points, isFound] = step(tracker, sensor.img);
visiblePoints = points(isFound, :);

oldInliers = oldPoints(isFound, :);

if mod(iter, resample) == 0
    oldPoints = C;
    setPoints(tracker, C);
end


% get camera coordinates in meters
p = K\[visiblePoints'; ones(1,size(visiblePoints,1))];
p_old = K\[oldInliers'; ones(1,size(oldInliers,1))];
p(3,:) = 0;
p_old(3,:) = 0;
dt = .0205;
p_dot = (p - p_old)/dt;

b = ones(1,35)/35;
p_dot(:,1) = filter(b, 1, p_dot(:,1));
p_dot(:,2) = filter(b, 1, p_dot(:,2));

[T, q] =  estimate_pose(sensor, K);

% get rotation matrix
q = reshape(q',[4 1 size(q',2)]);

s = q(1,1,:);
x = q(2,1,:);
y = q(3,1,:);
z = q(4,1,:);

R = [   1-2*(y.^2+z.^2)   2*(x.*y-s.*z)   2*(x.*z+s.*y)
    2*(x.*y+s.*z) 1-2*(x.^2+z.^2)   2*(y.*z-s.*x)
    2*(x.*z-s.*y)   2*(y.*z+s.*x) 1-2*(x.^2+y.^2)   ];

N = size(p,2);


ex = pi;
ez = -pi/4;
Rz = [cos(ez) -sin(ez) 0; sin(ez) cos(ez) 0; 0 0 1];
Rx = [1 0 0; 0 cos(ex) -sin(ex); 0 sin(ex) cos(ex)];


Tn = repmat(T, 1, N);
p_c = R*(Rx'*Rz')*p + Tn;
Zs = p_c(3,:);

p_c = p;
p_c(3,:) = Zs;

% show video
if 0
    videoFrame = insertMarker(sensor.img, visiblePoints, '+', 'Color', 'red');
    figure(1);
    imshow(videoFrame);
    hold on;
    quiver(visiblePoints(:,1),visiblePoints(:,2),p_dot(1,:)',p_dot(2,:)');
    hold off;
    getframe();
end

% begin RANSAC
max_iter = 100;
i = 0;
inliers = [];
thresh = .8;

F1 = @(p) [  -1./p(3,:)'            zeros(size(p,2),1)  p(1,:)'./p(3,:)'   p(1,:)'.*p(2,:)'    -1-(p(1,:)'.^2)     p(2,:)'];
F2 = @(p) [  zeros(size(p,2),1)     -1./p(3,:)'         p(2,:)'./p(3,:)'   1+(p(2,:)'.^2)      -p(1,:)'.*p(2,:)'   -p(1,:)'];

while (i < max_iter) && (length(inliers) < thresh*N)
   idx = randperm(N,3);
      
   [p_dot(1,idx)'; p_dot(2,idx)'];
   
   v = ([F1(p_c(:,idx)); F2(p_c(:,idx));]) \ [p_dot(1,idx)'; p_dot(2,idx)'];
   
   A = [p_dot(1,:)'; p_dot(2,:)'] - [F1(p_c); F2(p_c);]*v;
   A = reshape(A,[size(A,1)/2,2]);
   err = sqrt(sum(A.^2,2));
   
   in = find(err < .05);
   
   if (length(in) > length(inliers))
       inliers = in;
   end
   
   i = i + 1;
end

vels = ([F1(p_c(:,inliers)); F2(p_c(:,inliers));]) \ [p_dot(1,inliers)'; p_dot(2,inliers)'];

vel = R*(Rx'*Rz')*vels([1 2 3]);
omg = R*(Rx'*Rz')*vels([4 5 6]);




