function [pos, q] = estimate_pose(sensor, K)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor,   personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k


% landmark parameters
dy = [.304 .304 .330 .304 .304 .330 .304 .304];
dx = [.304 .304 .304 .304 .304 .304 .304 .304 .304 .304 .304];
p_orig =   [.076    .076;
            .152    0;
            .152    .152;
            0       .152;
            0       0];

id = [0, 12, 24, 36, 48, 60, 72, 84,  96;
 1, 13, 25, 37, 49, 61, 73, 85,  97;
 2, 14, 26, 38, 50, 62, 74, 86,  98;
 3, 15, 27, 39, 51, 63, 75, 87,  99;
 4, 16, 28, 40, 52, 64, 76, 88, 100;
 5, 17, 29, 41, 53, 65, 77, 89, 101;
 6, 18, 30, 42, 54, 66, 78, 90, 102;
 7, 19, 31, 43, 55, 67, 79, 91, 103;
 8, 20, 32, 44, 56, 68, 80, 92, 104;
 9, 21, 33, 45, 57, 69, 81, 93, 105;
10, 22, 34, 46, 58, 70, 82, 94, 106;
11, 23, 35, 47, 59, 71, 83, 95, 107];


%init
N = length(sensor.id);
D = zeros(2*N*5,9);
pos = [];
q = [];

if N > 0    
    % correct angles
    for i = 1:N
        [c, r] = ind2sub([12 9],find(id == sensor.id(i)));
        d = [sum(dx(1:c-1)) sum(dy(1:r-1))];
        p = K\[[sensor.p0(:,i) sensor.p1(:,i) sensor.p2(:,i) sensor.p3(:,i) sensor.p4(:,i)]; ones(1,5)];

        for j = 1:5
            xq = p(1,j);
            yq = p(2,j);
            wq = 1;
            Xp = (p_orig(j,1) + d(1));
            Yp = (p_orig(j,2) + d(2));
            Wp = 1;
            D(10*(i-1) + 2*(j-1) + 1,:) = [-Xp -Yp -Wp 0 0 0 xq*xq yq*xq wq*xq];
            D(10*(i-1) + 2*(j-1) + 2,:) = [0 0 0 -Xp -Yp -Wp xq*yq yq*yq wq*yq];
        end
    end

    [~, ~, V] = svd(D);
    Hp = reshape(V(:,end),[3, 3])';
        
    % rotate camera coordinates
    ez = -pi/4 + .01;
    ex = pi;
    Rz0 = [cos(ez) -sin(ez) 0; sin(ez) cos(ez) 0; 0 0 1];
    if Hp(3,3) < 0          % flip to SO(3)
        ez = 3*pi/4 + .01;
        Hp(:,3) = -Hp(:,3);
    end
    
    Rz = [cos(ez) -sin(ez) 0; sin(ez) cos(ez) 0; 0 0 1];
    Rx = [1 0 0; 0 cos(ex) -sin(ex); 0 sin(ex) cos(ex)];

    T = Hp(:,3)/norm(Hp(:,1));
    
    Hp(:,3) = cross(Hp(:,1), Hp(:,2));
    Hp = Hp*Rz*Rx;
 
    
    [U, ~, V] = svd(Hp);
    R = (U*diag([1 1 det(U*V')])*V');
    pos = -R*Rx'*Rz0'*(T - [-0.04, 0.0, -0.03]');
    q = rotm2quat(R);
end

end
