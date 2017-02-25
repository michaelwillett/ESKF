% p = zeros(3,length(data));
% e = zeros(3,length(data));
% t = zeros(1,length(data));
% for i = 1:length(data)
%     if length(data(i).id) > 0
%         [p(:,i) q] = estimate_pose_handle(data(i));
%         e(3:-1:1,i) = quat2eul(q);
%         t(i) = data(i).t;
%     end
% end
% 
% pv = zeros(3,length(vicon));
% ev = zeros(3,length(data));
% tv = zeros(1,length(vicon));
% for i = 1:length(vicon)
%     pv(:,i) = vicon(1:3,i);
%     ev(:,i) = vicon(4:6,i);
%     tv(i) = time(i);
% end
% 
% figure(1);
% subplot(3,1,1);
% plot(t,p(1,:),'b',tv,pv(1,:),'r');
% subplot(3,1,2);
% plot(t,p(2,:),'b',tv,pv(2,:),'r');
% subplot(3,1,3);
% plot(t,p(3,:),'b',tv,pv(3,:),'r');
% 
% figure(2);
% subplot(3,1,1);
% plot(t,e(1,:),'b',tv,ev(1,:),'r');
% subplot(3,1,2);
% plot(t,e(2,:),'b',tv,ev(2,:),'r');
% subplot(3,1,3);
% plot(t,e(3,:),'b',tv,ev(3,:),'r');



% init_script;
% vel = zeros(3,length(data));
% omg = zeros(3,length(data));
% t = zeros(1,length(data));
% for i = 1:length(data)
%     if data(i).is_ready && length(data(i).id) > 0
%         [vel(:,i) omg(:,i)] = estimate_vel_handle(data(i));
%         t(i) = data(i).t;
%     end
% end
% 
% velv = zeros(3,length(vicon));
% omgv = zeros(3,length(data));
% tv = zeros(1,length(vicon));
% for i = 1:length(vicon)
%     velv(:,i) = vicon(7:9,i);
%     omgv(:,i) = vicon(10:12,i);
%     tv(i) = time(i);
% end
% 
% figure(2);
% subplot(3,1,1);
% plot(t,vel(1,:),'b',tv,velv(1,:),'r');
% subplot(3,1,2);
% plot(t,vel(2,:),'b',tv,velv(2,:),'r');
% subplot(3,1,3);
% plot(t,vel(3,:),'b',tv,velv(3,:),'r');
% 
% figure(3);
% subplot(3,1,1);
% plot(tv,omgv(1,:),'r',t,omg(1,:),'b');
% subplot(3,1,2);
% plot(tv,omgv(2,:),'r',t,omg(2,:),'b');
% subplot(3,1,3);
% plot(tv,omgv(3,:),'r',t,omg(3,:),'b');



% init_script;
% p = zeros(3,length(data));
% e = zeros(3,length(data));
% t = zeros(1,length(data));
% for i = 1:length(data)
%     if data(i).is_ready && length(data(i).id) > 0
%         j = sum(time <= data(i).t);
%         sensor = data(i);
%         vic.vel = vicon(7:12,j);
%         vic.t = time(j);
%         [x ~] = eskf1_handle(sensor, vic);
%         p(:,i) = x(1:3);
%         e(3:-1:1,i) = quat2eul(x(4:7)');
%         t(i) = data(i).t;
%     end
% end




% 
init_script;
p = zeros(3,length(data));
v = zeros(3,length(data));
e = zeros(3,length(data));
t = zeros(1,length(data));
for i = 1:length(data)
%     if data(i).is_ready
        j = sum(time <= data(i).t);
        sensor = data(i);
        vic.vel = vicon(7:12,j);
        vic.t = time(j);
        [x ~] = eskf2_handle(sensor);
        p(:,i) = x(1:3);
        v(:,i) = x(4:6);
        e(3:-1:1,i) = quat2eul(x(7:10)');
%     end
    t(i) = data(i).t;
end

pv = zeros(3,length(vicon));
vv = zeros(3,length(vicon));
ev = zeros(3,length(data));
tv = zeros(1,length(vicon));
for i = 1:length(vicon)
    pv(:,i) = vicon(1:3,i);
    ev(:,i) = vicon(4:6,i);
    vv(:,i) = vicon(7:9,i);
    tv(i) = time(i);
end

figure(3);
subplot(3,1,1);
plot(t,p(1,:),'b',tv,pv(1,:),'r');
title('Position');
subplot(3,1,2);
plot(t,p(2,:),'b',tv,pv(2,:),'r');
subplot(3,1,3);
plot(t,p(3,:),'b',tv,pv(3,:),'r');

figure(4);
subplot(3,1,1);
plot(t,e(1,:),'b',tv,ev(1,:),'r');
title('RPY');
subplot(3,1,2);
plot(t,e(2,:),'b',tv,ev(2,:),'r');
subplot(3,1,3);
plot(t,e(3,:),'b',tv,ev(3,:),'r');

figure(5);
subplot(3,1,1);
plot(t,v(1,:),'b',tv,vv(1,:),'r');
title('Velocity');
subplot(3,1,2);
plot(t,v(2,:),'b',tv,vv(2,:),'r');
subplot(3,1,3);
plot(t,v(3,:),'b',tv,vv(3,:),'r');
