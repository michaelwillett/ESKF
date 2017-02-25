function [X, Z] = eskf1(sensor, vic, K)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
    persistent Model t_sensor t_vicon reset;

    % initialize Model Parameters:
    if nargin == 0
        reset = true;
        return
    end
      
    X = zeros(7,1);
    Z = zeros(7,1);
    
    if reset
        te_p = .05;
        te_u = .2;
        err_p = .05;
        err_u = .15;
        reset = false;
        Model.x = [0 0 0 1 0 0 0 0 0 0]';                                      % initial nominal state
        Model.mu = zeros(9,1);                                      % initial error state
        Model.sigma = eye(9,9);                                     % initial cov of error state
        Model.A = eye(9,9);                                         % process model
        Model.Q = diag([te_p te_p te_p err_p err_p err_p .1 .1 .1].^2);   % uncertainty in process noise
        Model.W = eye(7);                                           % some variable I don't understand, but is in the slides
        Model.R = diag([te_u te_u te_u err_u err_u err_u err_u].^2);              % uncertainty in update model
        t_sensor = sensor.t;
        t_vicon = vic.t;
        return
    end
    
    
    if (vic.t > t_vicon)        % prediction update
        dt = vic.t - t_vicon;
        Model = ESKFPrediction(Model, dt, vic.vel');
        t_vicon = vic.t;
    end
     
    if (sensor.t > t_sensor & ~isempty(sensor.id))    % observation update
        dt = sensor.t - t_sensor;
        t_sensor = sensor.t;
        [v o] = estimate_vel(sensor, K, dx, dy, p_orig, id);
        Model = ESKFPrediction(Model, dt, [v o]');

        [T, q] =  estimate_pose(sensor, K, dx, dy, p_orig, id);
        Model = ESKFUpdate(Model, [T; q']);
    end
    
    X(1:7) = Model.x(1:7);
end

function [Model] = ESKFPrediction(Model, dt, u)
% KFPrediction computes the predicted mean and covariance using
% the dynamic model x_dot = A*x + B*u + U*n

    mu = Model.mu;
    sigma = Model.sigma;
    Q = Model.Q;
    p = Model.x(1:3);
    q = Model.x(4:7);
    b = Model.x(8:10);
    R = quat2rotm(q);
    
    v = u(1:3)';
    omega = u(4:6)';
    
    F = eye(9) + padarray(padarray(-R*dt, [0,6],'pre'), [3,0]);
    U = padarray(-eye(3),[6,6], 'post') + padarray(eye(3),[6,6], 'pre') + padarray(-R, [3 3]);

    % update error
    Model.mu = F*mu;
    Model.sigma = F*sigma*F' + U*Q*U';
    
    % update nominal state
    Model.x(1:3) = p + v*dt;
    Model.x(4:7) = QuatHProd(q,quat(.5*(omega - b)*dt));                  % this is for omega in local frame
end

function [Model] = ESKFUpdate(Model, z)
% KFUpdate computes the updated mean and covariance using
% the observation model z(t) = C*x(t) + W*v(t)
    
    mu_b = Model.mu;
    sigma_b = Model.sigma;
    x_b = Model.x(1:7);
    W = Model.W;
    R = Model.R;
    I = eye(9);
    
    q = Model.x(4:7);
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    Q = .5*[-qx -qy -qz;...
            qw qz -qy;...
            -qz qw qx;...
            qy -qx qw];
        
    Cl = eye(7,10);
    Cr = padarray(eye(3),[7,6], 'post') + padarray(eye(3),[7,6], 'pre') + padarray(Q, [3 3]);
    C = Cl*Cr;

% Update error state using observation model
    dz = zeros(7,1);
    dz(1:7) = z(1:7) - x_b(1:7);
    K = (sigma_b*C')/(C*sigma_b*C'+ W*R*W');
    mu_p = K*dz;
    sigma_p = (I - K*C)*sigma_b*(I - K*C)' + K*W*R*W'*K';

% Inject observed error into nominal state
    Model.x(1:3) = x_b(1:3) + mu_p(1:3);
    Model.x(4:7) = x_b(4:7) + [0; mu_p(4:6)];
    Model.x(4:7) = Model.x(4:7) / norm(Model.x(4:7));
    Model.x(8:10) = Model.x(8:10) + mu_p(7:9);
    
% Reset error
    dtheta = mu_p(4:6);
    G = eye(9,9) + padarray(Hat(.5*dtheta),[3 3]);
    Model.mu = zeros(9,1);
    Model.sigma = G*sigma_p*G';
end



% HELPER FUNCTIONS

function r = QuatHProd(p, q)
    r = [p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4); ...
         p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3); ...
         p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2); ...
         p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1)];
end


function q = quat(w)
    q = [1 w(1) w(2) w(3)]';
    q = q/norm(q);
end

function R = quat2rotm(q)
    s = q(1,1,:);
    x = q(2,1,:);
    y = q(3,1,:);
    z = q(4,1,:);

    R = [   1-2*(y.^2+z.^2)   2*(x.*y-s.*z)   2*(x.*z+s.*y)
        2*(x.*y+s.*z) 1-2*(x.^2+z.^2)   2*(y.*z-s.*x)
        2*(x.*z-s.*y)   2*(y.*z+s.*x) 1-2*(x.^2+y.^2)   ];
end

function H = Hat(w)
    H = [   0       -w(3)   w(2);...
            w(3)    0       -w(1);...
            -w(2)   w(1)    0];
end