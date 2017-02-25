function [X, Z] = eskf2(sensor, K)
% EKF2 Extended Kalman Filter with IMU as inputs
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
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

    persistent Model t_sensor reset;

    % initialize Model Parameters:
    if nargin == 0
        reset = true;
        return
    end
      
    X = zeros(10,1);
    Z = zeros(10,1);
    
    if reset
        te_p = .05;
        te_u = .2;
        
        err_p = .05;
        err_u = .15;
        reset = false;
        Model.x = [0 0 0 0 0 0 1 0 0 0 0 0 0]';                                             % initial nominal state
        Model.mu = zeros(12,1);                                                             % initial error state
        Model.sigma = eye(12);                                                              % initial cov of error state
        Model.Q = diag([te_p te_p te_p .1 .1 .1 err_p err_p err_p .1 .1 .1].^2);      % uncertainty in process noise
        Model.W = eye(10);                                                                  % some variable I don't understand, but is in the slides
        Model.R = diag([te_u te_u te_u te_u te_u te_u err_u err_u err_u err_u].^2);         % uncertainty in update model
        t_sensor = sensor.t;
        return
    end
    
    % observation update
    if (sensor.t > t_sensor)    
        dt = sensor.t - t_sensor;
        t_sensor = sensor.t;
        Model = ESKFPrediction(Model, dt, [sensor.omg; sensor.acc]);
        
        if ~isempty(sensor.id)
            [v, ~] = estimate_vel(sensor, K);
            [T, q] = estimate_pose(sensor, K);
            Model = ESKFUpdate(Model, [T; v; q']);
        else
            Model = ResetError(Model);
        end
        
    end
    
    X = Model.x;
end

function [Model] = ESKFPrediction(Model, dt, u)
% KFPrediction computes the predicted mean and covariance using
% the dynamic model x_dot = A*x + B*u + U*n

    mu = Model.mu;
    sigma = Model.sigma;
    Q = Model.Q;
    p = Model.x(1:3);
    v = Model.x(4:6);
    q = Model.x(7:10);
    b = Model.x(11:13);
    R = quat2rotm(q);
    
    omega = u(1:3);
    a = R*u(4:6) - [0 0 9.8]';
    
    F = eye(12) + padarray(padarray(-R*dt, [6,9],'pre'), [3,0], 'post');
    U = padarray(-eye(3),[9,9], 'post') + padarray(eye(3),[9,9], 'pre') + padarray(padarray(-R, [6 6],'pre'), [3,3], 'post') + padarray(padarray(-R, [6 6],'post'), [3,3], 'pre');

    % update error
    Model.mu = F*mu;
    Model.sigma = F*sigma*F' + U*Q*U';
    
    % update nominal state
    Model.x(1:3) = p + v*dt;
    Model.x(4:6) = v + a*dt;
    Model.x(7:10) = QuatHProd(quat(.5*(omega)*dt),q);                  % this is for omega in local frame
end

function [Model] = ESKFUpdate(Model, z)
% KFUpdate computes the updated mean and covariance using
% the observation model z(t) = C*x(t) + W*v(t)
    
    mu_b = Model.mu;
    sigma_b = Model.sigma;
    x_b = Model.x(1:10);
    W = Model.W;
    R = Model.R;
    I = eye(12);
    
    q = Model.x(7:10);
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    Q = .5*[-qx -qy -qz;...
            qw qz -qy;...
            -qz qw qx;...
            qy -qx qw];
        
    Cl = eye(10,13);
    Cr = padarray(eye(6),[7,6], 'post') + padarray(eye(3),[10,9], 'pre') + padarray(padarray(Q, [6 6], 'pre'), [3 3], 'post');
    C = Cl*Cr;

% Update error state using observation model
    dz = z - x_b;
    K = (sigma_b*C')/(C*sigma_b*C'+ W*R*W');
    mu_p = K*dz;
    sigma_p = (I - K*C)*sigma_b*(I - K*C)' + K*W*R*W'*K';

% Inject observed error into nominal state
    Model.x(1:6) = x_b(1:6) + mu_p(1:6);
    Model.x(7:10) = x_b(7:10) + [0; mu_p(7:9)];
    Model.x(7:10) = Model.x(7:10) / norm(Model.x(7:10));
    Model.x(11:13) = Model.x(11:13) + mu_p(10:12);
    
% Reset error
    dtheta = mu_p(7:9);
    G = eye(12) + padarray(padarray(Hat(.5*dtheta),[3 3],'post'),[6 6],'pre');
    Model.mu = zeros(12,1);
    Model.sigma = G*sigma_p*G';
end

function Model = ResetError(Model)
    % Inject observed error into nominal state
    Model.x(1:6) = Model.x(1:6) + Model.mu(1:6);
    Model.x(7:10) = Model.x(7:10) + [0; Model.mu(7:9)];
    Model.x(7:10) = Model.x(7:10) / norm(Model.x(7:10));
    Model.x(11:13) = Model.x(11:13) + Model.mu(10:12);

    % Reset error
    dtheta = Model.mu(7:9);
    G = eye(12) + padarray(padarray(Hat(.5*dtheta),[3 3],'post'),[6 6],'pre');
    Model.mu = zeros(12,1);
    Model.sigma = G*Model.sigma*G';
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