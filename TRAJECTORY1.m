%% trajectory generation
WP_low_acc;
%define a set of "waypoints" (contraints) for the trajectory
traj_sel = 3;

% time [s], position (x,y,z) [m], Orientation (yaw pitch roll) [DEG]
if traj_sel==1 
    constraints = [0,       20,20,0,    90,0,0;
                   30,      50,20,5,    90,35,10;
                   40,      58,15.5,10, 162,25,7;
                   60,      60,0,2      180,0,0];
elseif traj_sel==2
    constraints = [ 0   0   0         0         0         0         0;...        
        1   0           0         0         0         0         0;...
    6.9444   27.0353  -11.0895    -1.2691  -17.8689    4.6448    4.4314;...
   13.8889   38.0952  -25.0973    -2.4873  -51.7070   -1.8029   -1.8645;...
   20.8333   46.6974  -48.0545    -3.6128  -69.4588   -6.6726   -6.1263;...
   27.7778   39.6313  -71.0117    -4.7212 -107.1080  -10.8735   -9.1531;...
   34.7222   27.9570  -79.9611    -5.9627 -142.5267  -13.1720   -8.7144;...
   41.6667  -44.5469  -82.2957    -6.4027 -178.1557   -8.0442   -2.0766;...
   48.6111  -67.5883  -74.9027    -7.0973  162.2108   -6.5044    3.2926;...
   55.5556  -81.7204  -50.3891    -8.6248  119.9635   -8.1054    3.0096;...
   62.5000  -86.3287   -3.3074   -10.7885   95.5902   -7.6819   -1.3601;...
   69.4444  -85.4071   47.2763   -13.3720   88.9562   -3.7470   -6.6140;...
   76.3889  -69.7389   76.4591   -15.2714   61.7688    1.5208  -12.3492;...
   83.3333  -46.3902   86.9650   -15.9091   24.2255    9.3477  -13.2543;...
   90.2778  -19.3548   89.2996   -15.7643    4.9355   14.8030   -8.1259;...
   97.2222    2.1505   88.9105   -14.7595   -1.0366   14.6919   -6.5567;...
  104.1667    8.6022   88.9105   -13.5347         0   11.9302   -7.4452;...
  111.1111   12.2888   86.9650   -13.3567  -27.8217    3.0853   -7.7571;...
  118.0556   14.1321   85.4086   -13.6538  -40.1763   -8.4237   -8.6308;...
  125.0000   15.0538   83.0739   -13.8232  -68.4570  -13.3703   -4.1111;...
  131.9444   14.1321   78.7938   -14.4416 -102.1521  -12.4273    4.4139;...
  138.8889    7.6805   78.4047   -14.8874 -176.5486   -8.7867    9.2389;...
  145.8333   -0.9217   76.4591   -14.2209 -167.2560   -0.4272    9.7053;...
  152.7778   -1.5361   70.2335   -13.3239  -95.6365    5.8486    6.1606;...
  159.7222    0.3072   64.3969   -12.2226  -72.4727    6.2868   -0.1808;...
  166.6667    6.7588   62.4514   -11.1399  -16.7810    8.5828   -0.7664;...
  173.6111   11.0599   60.1167   -10.8808  -28.4932   12.3066    4.7437;...
  180.5556   15.9754   55.4475   -10.5200  -43.5283   12.0021    8.3825;...
  187.5000   16.8971   49.2218    -9.3512  -81.5790    6.3497    7.3499;...
  194.4444   15.0538   42.6070    -7.4707 -105.5714   -0.8404    1.7566;...
  201.3889   14.1321   35.2140    -5.0758  -97.1062   -6.5643   -2.5193;...
  208.3333   11.0599   29.3774    -2.7465 -117.7610   -9.9435   -0.2147;...
  215.2778    8.6022   25.8755    -1.8824 -125.0622  -11.0863    2.4769;...
  222.2222    6.7588   20.8171    -2.2549 -110.0222  -11.5590    4.6307;...
  229.1667    2.1505   11.8677    -1.6681 -117.2452   -8.1527    5.6081;...
  236.1111    0.3072    5.6420    -0.5689 -106.4931   -2.8508    3.2170;...
  243.0556    0.3072    4.8638   -0.9765  -90.0000    4.6519   -1.9190;...
  250.0000   -0.6144    1.7510    0.000   -106.4931   14.8082   -9.5876];
elseif traj_sel==3
    constraints=waypoints_ned;

end




%define sample time
ST = 1/100;

%construct trajectory object
trajectory = waypointTrajectory(constraints(:,2:4), ...
    TimeOfArrival=constraints(:,1), ...
    Orientation=quaternion(constraints(:,5:7),"eulerd","ZYX","frame"), ...
    SampleRate=1/ST);

% waypointInfo returns a table of specified constraints. 
tInfo = waypointInfo(trajectory);

% The trajectory object outputs the current position, velocity,
% acceleration, and angular velocity at each call. 

%% trajectory samples extraction
% Call trajectory() in a loop to get, position, velocity and 
% orientation over time. 

trajectory.reset();
%build time vector and prepare empty 
% attitude, pos, vel, acc, angvel vectors
time = 0:ST:(tInfo.TimeOfArrival(end)-ST);
time=time';
quat = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,"quaternion");
vel_ned = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
acc_n = vel_ned;
pos = vel_ned;
omega_n = vel_ned;

count = 1;
while ~isDone(trajectory)
   [pos(count,:),quat(count),vel_ned(count,:),acc_n(count,:),omega_n(count,:)] = trajectory();
   count = count + 1;
end
quat_nb = compact(quat); 

%% plot result
%position
figure;
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
set(gca,"XDir","reverse")
set(gca,"ZDir","reverse")
hold on
plot3(pos(:,1), pos(:,2), pos(:,3),"b" );
hold off

%position wrt time
figure;
plot(time, pos(:,1), "r" );
hold on; grid on;
plot(time, pos(:,2), "g" );
plot(time, pos(:,3), "b" );
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,1),'k*');
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,2),'k*');
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,3),'k*');
title("Position")
xlabel("Time")
ylabel("[m]")
hold off
legend("North","East","Down")

%velocity
figure;
plot(time, vel_ned(:,1), "r" );
hold on; grid on;
plot(time, vel_ned(:,2), "g" );
plot(time, vel_ned(:,3), "b" );
title("Velocity")
xlabel("Time")
ylabel("[m/s]")
legend("V_n","V_e","V_d")
hold off

%acceleration
figure;
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("a_n","a_e","a_d")
hold off

%angular velocity
figure;
plot(time, omega_n(:,1), "r" );
hold on; grid on;
plot(time, omega_n(:,2), "g" );
plot(time, omega_n(:,3), "b" );
title("Angular Velocity in Navigation Frame")
xlabel("Time")
ylabel("[rad/s]")
legend("\omega_n","\omega_e","\omega_d")
hold off

%Orientation (quaternion)
figure;
quat_c = compact(quat); %convert to array for plotting
plot(time, quat_c(:,1), "r" );
hold on; grid on;
plot(time, quat_c(:,2), "g" );
plot(time, quat_c(:,3), "b" );
plot(time, quat_c(:,4), "m" );
title("Attitude (Quaternion)")
xlabel("Time")
legend("q_0","q_1","q_2","q_3")
hold off


%% create ideal inertial measurements

%% convert quaternions to euler angles
% notes:
% - using the matlab function rotm = quat2rotm(quat)
%   returns a rotation matrix from body to nav 
% - quaternions in matlab must be [w x y z]
% - using rotationMatrix = rotmat(quat,'frame')
%   creates a rotation matrix that rotates nav to body
% - using quat = quaternion(RM,'rotmat','frame') creates a quaternion from rotation matrix RM for
%   frame rotations
% - using quat = quaternion(RM,'rotmat','point') creates a quaternion for
%   point rotations
% - using eul = rotm2eul(rotm,'ZYX') converts the rotation matrix rotm to euler
%   angles BUT requires rotm to be a body to nav matrix AND returns yaw, pitch, roll  in this order !!!

rpy = zeros(length(time),3);
rpy_bis = rpy;
pqr = omega_n;
acc = omega_n;

for idx=1:length(time),
    q_in = quat(idx,:);
    %try to methods:
    %   - build a rotation matrix first, then extract euler angles
    %   - use the built in constructor for euler angles
    %method 1
    rm = rotmat(q_in,'frame'); %this rot mat transforms nav to body
    pqr(idx,:) = (rm*omega_n(idx,:)')'; %rotate omega to body frame
    eul_out = rotm2eul(rm','ZYX'); 
    eul_out = eul_out([3 2 1]); %adjust for order of angles returned by rotm2eul
    rpy(idx,:)= eul_out;
    %as a side result: rotate acc to body frame
    acc(idx,:) = (rm*acc_n(idx,:)')'; 

    %method 2
    eul_out_bis = euler(q_in,"ZYX","frame");
    eul_out_bis = eul_out_bis([3 2 1]); %adjust for order of angles returned by rotm2eul
    rpy_bis(idx,:)= eul_out_bis;
end
%compare obtained euler angles
figure; 
plot(time,rpy,'r');
hold on;
plot(time,rpy_bis,'b:','linewidth',2);
hold off; grid on;

%plot Euler angles
figure;
plot(time, rpy(:,1), "r" );
hold on; grid on;
plot(time, rpy(:,2), "g" );
plot(time, rpy(:,3), "b" );
title("Euler Angles")
xlabel("Time")
ylabel("[rad]")
legend("Roll","Pitch","Yaw")
hold off

%plot angular velocity in body frame
figure;
plot(time, pqr(:,1), "r" );
hold on; grid on;
plot(time, pqr(:,2), "g" );
plot(time, pqr(:,3), "b" );
title("Angular Velocity in Body frame")
xlabel("Time")
ylabel("[rad/s]")
legend("\omega_x","\omega_y","\omega_z")
hold off

%plot acceleration in body frame
figure;
plot(time, acc(:,1), "r" );
hold on; grid on;
plot(time, acc(:,2), "g" );
plot(time, acc(:,3), "b" );
title("Acceleration in Body frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("acc_x","acc_y","acc_z")
hold off

%% verify that acceleration and velocity are consistent with position

%integrate navigation velocity 
pos_vi = cumsum(vel_ned).*ST+pos(1,:);
pos_ai = cumsum(cumsum(acc_n).*ST+vel_ned(1,:)).*ST+pos(1,:);

pos_vi_t = cumtrapz(vel_ned).*ST+pos(1,:);
pos_ai_t = cumtrapz(cumtrapz(acc_n).*ST+vel_ned(1,:)).*ST+pos(1,:);


figure;
plot(time,pos(:,1),'r', ...
     time,pos(:,2),'r', ...
     time,pos(:,3),'r');
hold on;
plot(time,pos_vi(:,1),'b', ...
     time,pos_vi(:,2),'b', ...
     time,pos_vi(:,3),'b');
plot(time,pos_ai(:,1),'g', ...
     time,pos_ai(:,2),'g', ...
     time,pos_ai(:,3),'g');
plot(time,pos_vi_t(:,1),'k:', ...
     time,pos_vi_t(:,2),'k:', ...
     time,pos_vi_t(:,3),'k:');
plot(time,pos_ai_t(:,1),'m:', ...
     time,pos_ai_t(:,2),'m:', ...
     time,pos_ai_t(:,3),'m:');
hold off
title("Position Over Time")
legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("Position (m)")
grid on


figure;
plot(time,pos_vi(:,1)-pos(:,1),'b', ...
     time,pos_vi(:,2)-pos(:,2),'b', ...
     time,pos_vi(:,3)-pos(:,3),'b');
hold on;
plot(time,pos_ai(:,1)-pos(:,1),'g', ...
     time,pos_ai(:,2)-pos(:,2),'g', ...
     time,pos_ai(:,3)-pos(:,3),'g');
plot(time,pos_vi_t(:,1)-pos(:,1),'k', ...
     time,pos_vi_t(:,2)-pos(:,2),'k', ...
     time,pos_vi_t(:,3)-pos(:,3),'k');
plot(time,pos_ai_t(:,1)-pos(:,1),'m', ...
     time,pos_ai_t(:,2)-pos(:,2),'m', ...
     time,pos_ai_t(:,3)-pos(:,3),'m');
title("Position Over Time ERROR")
legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("Position (m)")
grid on


%% verify that angular velocity is consistent with euler angles

%%integrate pqr_b
rpy_i = rpy;
for idx=2:length(time),
    phi = rpy_i(idx-1,1); theta=rpy_i(idx-1,2); psi=rpy_i(idx-1,3);
    %create Jacobian that transforms angular rates into rates of Euler
    %angles
    J = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    eul_rates = J*pqr(idx,:)';
    rpy_i(idx,:)=rpy_i(idx-1,:)+eul_rates'.*ST;
end
figure; 
plot(time,rpy);
hold on;
plot(time,rpy_i);
title("Euler angles over Time")
hold off; grid on;

figure; 
rpy_err=wrapToPi(wrapToPi(rpy)-wrapToPi(rpy_i));
plot(time,rpy_err);
title("Euler angles over Time ERROR")
hold off; grid on;

%% smooth accelerations and recompute everything

%define a second order low pass filter 
if traj_sel==1,
    lp_tc = 2;
elseif traj_sel==2,
    lp_tc = 0.5;
else
    lp_tc = 2;
end

lpfilt = tf(1,conv([1/lp_tc 1],[1/lp_tc 1]));
lpfilt_td = c2d(lpfilt,ST);

%filer acceleration
acc_n_f = filter(lpfilt_td.Numerator{1}, lpfilt_td.Denominator{1},acc_n);

%acceleration
figure;
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
plot(time, acc_n_f(:,1), "r:" , 'linewidth',2);
plot(time, acc_n_f(:,2), "g:" , 'linewidth',2);
plot(time, acc_n_f(:,3), "b:" , 'linewidth',2);
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("a_n","a_e","a_d","a_n filt","a_e filt","a_d filt")
hold off

%recompute velocity and position (starting from 0 velocity)
vel_n_f = cumtrapz(acc_n_f).*ST+vel_ned(1,:);
pos_f = cumtrapz(vel_n_f).*ST+pos(1,:);

%plot new trajectory 
figure;
plot(time,pos(:,1),'r', ...
     time,pos(:,2),'g', ...
     time,pos(:,3),'k');
hold on;
plot(time,pos_f(:,1),'b:',  ...
     time,pos_f(:,2),'b:', ...
     time,pos_f(:,3),'b:', 'linewidth',2);
hold off
title("Position Over Time")
legend("North","East","Down","North filt","East filt","Down filt")
xlabel("Time (seconds)")
ylabel("[m]")
grid on

figure;
plot(time,vel_ned(:,1),'r', ...
     time,vel_ned(:,2),'g', ...
     time,vel_ned(:,3),'k');
hold on;
plot(time,vel_n_f(:,1),'b', ...
     time,vel_n_f(:,2),'b', ...
     time,vel_n_f(:,3),'b', 'linewidth',2);
hold off
title("Velocity Over Time")
legend("V_n","V_e","V_d","V_n filt","V_e filt","V_d filt");
xlabel("Time (seconds)")
ylabel("[m/s]")
grid on

%recompute acc in body frame
acc_f = acc;
for idx=1:length(time),
    q_in = quat(idx,:);
    rm = rotmat(q_in,'frame'); %this rot mat transforms nav to body
    acc_f(idx,:) = (rm*acc_n_f(idx,:)')'; 
end
%and compare 
figure;
plot(time,acc(:,1),'r', ...
     time,acc(:,2),'g', ...
     time,acc(:,3),'k');
hold on;
plot(time,acc_f(:,1),'b', ...
     time,acc_f(:,2),'b', ...
     time,acc_f(:,3),'b', 'linewidth',2);
hold off
title("Acceleration Over Time")
legend("Acc_n","Acc_e","Acc_d","Acc_n filt", "Acc_e filt", "Acc_d filt")
xlabel("Time (seconds)")
ylabel("[m/s]")
grid on

%% replace raw acc with filtered acc (and pos and vel)
use_filtered_acc = 1;
if use_filtered_acc,
    acc = acc_f;
    acc_n = acc_n_f;
    vel_ned = vel_n_f;
    pos = pos_f;
end

%position with quiver
figure;
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos(:,1), pos(:,2), pos(:,3),"b" );
%add arrow along x axis
% x_axis_nav = Cbn*x_axis_body= Cbn*[1 0 0] = [cos theta * cos psi; cos theta * sin psi; -sin theta];
undersample=500;
x_axis_nav_x = cos(rpy(:,2)).*cos(rpy(:,3));
x_axis_nav_y = cos(rpy(:,2)).*sin(rpy(:,3));
x_axis_nav_z = -sin(rpy(:,2));
quiver3(pos(1:undersample:end,1),pos(1:undersample:end,2),pos(1:undersample:end,3),...
    x_axis_nav_x(1:undersample:end),x_axis_nav_y(1:undersample:end),x_axis_nav_z(1:undersample:end))
hold off

%position with axis
figure;
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos(:,1), pos(:,2), pos(:,3),"b" );
% x_axis_nav = Cbn*x_axis_body= Cbn*[1 0 0] = [cos theta * cos psi; cos theta * sin psi; -sin theta];
% y_axis_nav = Cbn*x_axis_body= Cbn*[0 1 0] = [-cos(phi) * sin(psi) + cos(psi)*sin(phi)*sin(theta); cos(phi) * cos(psi) + sin(psi)*sin(phi)*sin(theta); sin(phi)*cos(theta)];
% z_axis_nav = Cbn*x_axis_body= Cbn*[0 0 1] = [sin(phi)*sin(psi)+cos(psi)*cos(phi)*sin(theta); -sin(phi)*cos(psi)+sin(psi)*cos(phi)*sin(theta); cos(phi)*cos(theta)];
undersample=500;
phi= rpy(:,1); theta=rpy(:,2); psi=rpy(:,3);
x_axis_nav_x = cos(theta).*cos(psi);
x_axis_nav_y = cos(theta).*sin(psi);
x_axis_nav_z = -sin(theta);
y_axis_nav_x = -cos(phi) .* sin(psi) + cos(psi).*sin(phi).*sin(theta);
y_axis_nav_y = cos(phi) .* cos(psi) + sin(psi).*sin(phi).*sin(theta);
y_axis_nav_z = sin(phi).*cos(theta);
z_axis_nav_x = sin(phi).*sin(psi)+cos(psi).*cos(phi).*sin(theta);
z_axis_nav_y =  -sin(phi).*cos(psi)+sin(psi).*cos(phi).*sin(theta);
z_axis_nav_z =  cos(phi).*cos(theta);
AL = 5;
for i=1:undersample:length(pos),
    plot3([pos(i,1) pos(i,1)+AL*x_axis_nav_x(i,1)],...
        [pos(i,2) pos(i,2)+AL*x_axis_nav_y(i,1)],...
        [pos(i,3) pos(i,3)+AL*x_axis_nav_z(i,1)],'r',LineWidth=2);
    plot3([pos(i,1) pos(i,1)+AL*y_axis_nav_x(i,1)],...
        [pos(i,2) pos(i,2)+AL*y_axis_nav_y(i,1)],...
        [pos(i,3) pos(i,3)+AL*y_axis_nav_z(i,1)],'g',LineWidth=2);
    plot3([pos(i,1) pos(i,1)+AL*z_axis_nav_x(i,1)],...
        [pos(i,2) pos(i,2)+AL*z_axis_nav_y(i,1)],...
        [pos(i,3) pos(i,3)+AL*z_axis_nav_z(i,1)],'b',LineWidth=2);
end
hold off
set(gca,"XDir","reverse")
set(gca,"ZDir","reverse")


%% create accelerometer measurments (by adding gravity)
%add gravity to acc in body frame
%define gravity value
g_n = [0 0 9.81]';
for idx=1:length(time)
    q_in = quat(idx,:);
    % g_b = (quat2rotm(q_in)* g_n)'; %this rot mat transforms nav to body
    g_b = (rotmat(q_in,"frame")* g_n)'; %this rot mat transforms nav to body
    acc(idx,:) = acc(idx,:)-g_b; %add gravity with - sign
end

%% Gyroscope Bias
bias_gyro = [1 1 1]; %dbias_gyro=0

%% Magnetometer
% Centered in Polo A
% mag_n = [23.5 1.5 41]; %centered in Polo A
% mag_n = mag_n / norm(mag_n);
mag_n = Simulink.Parameter;
mag_n.Value = [0.4970    0.0317    0.8672];  % Each row is a beacon [north, east, down]

mag_n.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it code-generation compatible

%% Basi UWB
P_bases_ned = Simulink.Parameter;
P_bases_ned.Value = [0 0 0; -30 -30 40; 30 -25 0; 30 -50 5; -30 -30 10; 50 -5 20];  % Each row is a beacon [north, east, down]
% P_bases_ned.Value = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];  
P_bases_ned.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it code-generation compatible


%% Parameters
Q = Simulink.Parameter;
% Q.Value = eye(19)*1e-2;           % your initial value
Q.Value = eye(19).*10;           % your initial value

% Q.Value(1:3,1:3) = Q.Value(1:3,1:3).*1e-14;
% Q.Value(4:6,4:6) = Q.Value(4:6,4:6).*1e-14;
% Q.Value(16:19,16:19) = 1e-5;
%Q.Value(10:13,10:13) = 1e-4;

Q.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it tunable

dt = Simulink.Parameter;
dt.Value = 0.01;           % your initial value
dt.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it tunable

g = Simulink.Parameter;
g.Value = [0;0;9.81];           % your initial value
g.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it tunable

% Determine matrix size
n_rows = size(P_bases_ned.Value, 1);   % number of beacons
dim = n_rows + 10;

% Create identity matrix and assign to parameter
sensor_covariances = Simulink.Parameter;
sensor_covariances.Value = [ones(size(P_bases_ned.Value,1),1).*1e-2; 1e-4;1e-4;1e-4; 1e-2; 1e-2; 1e-2; 1e-2; 1e-4; 1e-4; 1e-4];  % Identity matrix of size (n_rows + 10)
% sensor_covariances.Value = [ones(size(P_bases_ned.Value,1),1).*1e-4; 1e-4;1e-4;1e-4; 1e-2; 1e-2; 1e-2; 1e-2; 1e-4; 1e-4; 1e-4];  % Identity matrix of size (n_rows + 10)
sensor_covariances.CoderInfo.StorageClass = 'ExportedGlobal';  % Makes it tunable

