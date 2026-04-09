clear all 
close all
clc
ST = 0.01;  % Sample time
dt = ST;

% Parametri principali
t_max = 60;
time = (0:ST:t_max)';

% Preallocazione
N = length(time);
pos_n = zeros(N,1);
pos_e = zeros(N,1);
pos_d = zeros(N,1);
yaw = zeros(N,1);
pitch = zeros(N,1);
roll = zeros(N,1);

% Durata delle fasi (in secondi)
T1 = 4;     % salita
T1_5 = 2;     % New horizontal move phase
T2 = 14;    % sinusoidale
T3 = 7;    % roll completo
T3_5 = 5;    % new transition to (0, 30)
T4 = 13;    % curva accelerata
T5 = 15;    % discesa

t1_end = T1;
t1_5_end = t1_end + T1_5;
t2_end = t1_5_end + T2;
t3_end = t2_end + T3;
t3_5_end = t3_end + T3_5;
t4_end = t3_5_end  + T4;
t5_end = t_max;

v_climb = -5;
% starting position
n = 0; e = 0; d = 0;
heading = 0;

% Fasi
v_climb = -5;  % vertical acceleration (m/s^2)
for i = 1:N
    t = time(i);

    if t <= T1
        % Use uniformly accelerated motion: d = 0.5 * a * t^2
        pos_n(i) = n;
        pos_e(i) = e;
        pos_d(i) = pos_d(N-1) + v_climb * t;  % starts from 0 with acceleration
        yaw(i) = 0;
        pitch(i) = 0;
        roll(i) = 0;


    elseif t <= t1_5_end
        % FASE 1.5: Move horizontally to reach start of circular motion
        dt = t - t1_end;
        s = dt / T1_5;  % Linear progression 0 -> 1

        % Linear interpolation from (0,0) to (10,0) in NE plane
        pos_n(i) = 10 * s;
        pos_e(i) = 0;
        pos_d(i) = -20;

        % Optional smooth yaw update
        yaw(i) = 0;
        pitch(i) = 0;
        roll(i) = 0;



    elseif t <= t2_end
        % FASE 2: Movimento sinusoidale
        dt = t - t1_5_end; % from 6 to 20 secs
        turn_radius = 10;
        freq = 1/(14/3);

        pos_n(i) = turn_radius * cos(pi*freq*dt);
        pos_e(i) = turn_radius * sin(pi*freq*dt);
        pos_d(i) = pos_d(i-1);
        yaw(i) = 2*pi*freq*dt;
        pitch(i) = 0;
        roll(i) = 0;

    elseif t <= t3_end
        % FASE 3: Roll completo
        dt = t - t2_end;
        pos_n(i) = pos_n(i-1);
        pos_e(i) = pos_e(i-1);
        pos_d(i) = pos_d(i-1);
        roll(i) = 2*pi * (dt / T3); % da 0 a 2pi

    % % elseif t <= t4_end
    % %     % FASE 4: Curva accelerata
    % %     dt4 = t - t3_end;
    % %     R = 10;
    % %     omega0 = 0.2;  % velocità iniziale angolare
    % %     alpha = 0.1;   % accelerazione angolare
    % % 
    % %     theta = omega0*dt4 + 0.5*alpha*dt4^2;
    % %     pos_n(i) = pos_n(i-1) + R * cos(theta);
    % %     pos_e(i) = pos_e(i-1) + R * sin(theta);
    % %     pos_d(i) = -20 + 2*sin(0.2*dt4);
    % %     yaw(i) = theta;
    % %     pitch(i) = 0.1;
    % %     roll(i) = 0.3;
    % % elseif t <= t3_5_end
    % %     % FASE 3.5: Transizione verso (0, 30)
    % %     dt35 = t - t3_end;
    % %     t_rel = dt35 / T3_5;
    % % 
    % %     % Interpolazione lineare verso (0, 30)
    % %     start_n = pos_n(i-1);
    % %     start_e = pos_e(i-1);
    % %     end_n = 0;
    % %     end_e = 30;
    % % 
    % %     pos_n(i) = (1 - t_rel) * start_n + t_rel * end_n;
    % %     pos_e(i) = (1 - t_rel) * start_e + t_rel * end_e;
    % %     pos_d(i) = -20;
    % %     yaw(i) = atan2(end_e - start_e, end_n - start_n);
    % %     % pitch(i) = 0.1;
    % %     % roll(i) = 0.1 * sin(pi * t_rel);  % smooth roll hint
    elseif t <= t3_5_end
        dt = t - t3_end;

        s = dt / T3_5;  % Linear progression 0 -> 1

        % Linear interpolation from (0,0) to (10,0) in NE plane
        pos_n(i) = pos_n(t3_end/ST+1)-20 * s;
        pos_e(i) = pos_e(i-1);
        pos_d(i) = pos_d(i-1);
    % 
    %     % Optional smooth yaw update
    %     yaw(i) = yaw(i-1);
    %     pitch(i) = pitch(i-1);
    %     roll(i) = roll(i-1);
    % 
    elseif t <= t4_end
        % FASE 4: Curva accelerata (circular arc around a fixed center)
        dt = t - t3_5_end;
        R = 30;                  % Radius of the curve (safe inside ±100)

        pos_n(i) = -R * cos(pi*freq*dt);
        pos_e(i) = R * sin(pi*freq*dt);
        pos_d(i) = pos_d(t3_5_end/ST+1)+2 * sin(0.2 * dt);
        yaw(i) = pi + pi*freq*dt;
        pitch(i) = pitch(i-1);
        roll(i) = -0.3;

    elseif t <= t5_end
        % FASE 5: Discesa verso l’origine
        dt = t - t4_end;
        t_rel = dt / T5;
        pos_n(i) = pos_n(t4_end/ST+1) * (1 - t_rel);
        pos_e(i) = pos_e(t4_end/ST+1) * (1 - t_rel);
        pos_d(i) = -20 * (1 - t_rel);  % Torna da -20 a 0

    else
        % Dopo la fine: resta fermo
        pos_n(i) = 0;
        pos_e(i) = 0;
        pos_d(i) = -20;
    end
end

% Unisci dati
pos_ned_wp = [pos_n, pos_e, pos_d];
eul = [yaw, pitch, roll];  % [rad]

% Converto in quaternion per waypointTrajectory
orient_quat = quaternion(eul, 'euler', 'ZYX', 'frame');

% Estrai 45 waypoint uniformi
num_wp = 75;
ind = round(linspace(1, length(time), num_wp));
waypoints_ned = [time(ind), pos_ned_wp(ind,:), eul(ind,:)];

% % trajectory = waypointTrajectory(waypoints_ned(:,2:4), ...
% %     TimeOfArrival = waypoints_ned(:,1), ...
% %     Orientation = quaternion(waypoints_ned(:,5:7),"euler","ZYX","frame"), ...
% %     SampleRate = 1/ST);
% % % waypointInfo returns a table of specified constraints.
% % tInfo = waypointInfo(trajectory);
% % t_end = tInfo.TimeOfArrival(end);
% % 
% % %% trajectory samples extraction
% % trajectory.reset();
% % time = 0:ST:(tInfo.TimeOfArrival(end)-ST);                                      % time vector
% % time=time'; 
% % quat_b = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,"quaternion");    % orientation quaternion, from nav to body, expressed in body frame
% % vel_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);                % velocity in navigation frame [m/s, m/s, m/s]
% % acc_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);                % acceleration in navigation frame [m/s^2, m/s^2, m/s^2]
% % pos_ned_wp = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);              % position in NED system [m, m, m]
% % omega_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);              % angular rate -> x_axiz y_axis z_axis [rad/s, rad/s, rad/s]
% % 
% % count = 1;
% % while ~isDone(trajectory)
% %    [pos_ned_wp(count,:),quat_b(count),vel_n(count,:),acc_n(count,:),omega_n(count,:)] = trajectory();  % acc and vel are in navigation frame
% %    count = count + 1;
% % end
% % 
% % rpy = eulerd(quat_b, 'ZYX', 'frame');  % roll, pitch, yaw in gradi
% % rpy = deg2rad(rpy);  % in radianti per il calcolo degli assi
% % 
% % %position with quiver
% % figure;
% % plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
% % title("Trajectory")
% % xlabel("North")
% % ylabel("East")
% % zlabel("Down")
% % grid on
% % axis equal
% % hold on
% % plot3(pos_ned_wp(:,1), pos_ned_wp(:,2), pos_ned_wp(:,3),"b");
% % %add arrow along x axis
% % % x_axis_nav = Cbn*x_axis_body= Cbn*[1 0 0] = [cos theta * cos psi; cos theta * sin psi; -sin theta];
% % undersample=500;
% % x_axis_nav_x = cos(rpy(:,2)).*cos(rpy(:,3));
% % x_axis_nav_y = cos(rpy(:,2)).*sin(rpy(:,3));
% % x_axis_nav_z = -sin(rpy(:,2));
% % quiver3(pos_ned_wp(1:undersample:end,1), pos_ned_wp(1:undersample:end,2), pos_ned_wp(1:undersample:end,3), ...
% %          x_axis_nav_x(1:undersample:end), x_axis_nav_y(1:undersample:end), x_axis_nav_z(1:undersample:end));
% % 
% % hold off
% % 
% % %position with axis
% % figure;
% % plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
% % title("Trajectory")
% % xlabel("North")
% % ylabel("East")
% % zlabel("Down")
% % grid on
% % axis equal
% % hold on
% % plot3(pos_ned_wp(:,1), pos_ned_wp(:,2), pos_ned_wp(:,3),"b" );
% % % x_axis_nav = Cbn*x_axis_body= Cbn*[1 0 0] = [cos theta * cos psi; cos theta * sin psi; -sin theta];
% % % y_axis_nav = Cbn*x_axis_body= Cbn*[0 1 0] = [-cos(phi) * sin(psi) + cos(psi)*sin(phi)*sin(theta); cos(phi) * cos(psi) + sin(psi)*sin(phi)*sin(theta); sin(phi)*cos(theta)];
% % % z_axis_nav = Cbn*x_axis_body= Cbn*[0 0 1] = [sin(phi)*sin(psi)+cos(psi)*cos(phi)*sin(theta); -sin(phi)*cos(psi)+sin(psi)*cos(phi)*sin(theta); cos(phi)*cos(theta)];
% % undersample=500;
% % phi= rpy(:,1); theta=rpy(:,2); psi=rpy(:,3);
% % x_axis_nav_x = cos(theta).*cos(psi);
% % x_axis_nav_y = cos(theta).*sin(psi);
% % x_axis_nav_z = -sin(theta);
% % y_axis_nav_x = -cos(phi) .* sin(psi) + cos(psi).*sin(phi).*sin(theta);
% % y_axis_nav_y = cos(phi) .* cos(psi) + sin(psi).*sin(phi).*sin(theta);
% % y_axis_nav_z = sin(phi).*cos(theta);
% % z_axis_nav_x = sin(phi).*sin(psi)+cos(psi).*cos(phi).*sin(theta);
% % z_axis_nav_y =  -sin(phi).*cos(psi)+sin(psi).*cos(phi).*sin(theta);
% % z_axis_nav_z =  cos(phi).*cos(theta);
% % AL = 5;
% % for i=1:undersample:length(pos_ned_wp),
% %     plot3([pos_ned_wp(i,1) pos_ned_wp(i,1)+AL*x_axis_nav_x(i,1)],...
% %         [pos_ned_wp(i,2) pos_ned_wp(i,2)+AL*x_axis_nav_y(i,1)],...
% %         [pos_ned_wp(i,3) pos_ned_wp(i,3)+AL*x_axis_nav_z(i,1)],'r',LineWidth=2);
% %     plot3([pos_ned_wp(i,1) pos_ned_wp(i,1)+AL*y_axis_nav_x(i,1)],...
% %         [pos_ned_wp(i,2) pos_ned_wp(i,2)+AL*y_axis_nav_y(i,1)],...
% %         [pos_ned_wp(i,3) pos_ned_wp(i,3)+AL*y_axis_nav_z(i,1)],'g',LineWidth=2);
% %     plot3([pos_ned_wp(i,1) pos_ned_wp(i,1)+AL*z_axis_nav_x(i,1)],...
% %         [pos_ned_wp(i,2) pos_ned_wp(i,2)+AL*z_axis_nav_y(i,1)],...
% %         [pos_ned_wp(i,3) pos_ned_wp(i,3)+AL*z_axis_nav_z(i,1)],'b',LineWidth=2);
% % end
% % hold off
% % set(gca,"XDir","reverse")
% % set(gca,"ZDir","reverse")
% % 
% % figure;
% % plot(time, acc_n(:,1), 'r', 'LineWidth', 1.5); hold on;
% % plot(time, acc_n(:,2), 'g', 'LineWidth', 1.5);
% % plot(time, acc_n(:,3), 'b', 'LineWidth', 1.5);
% % xlabel('Time [s]');
% % ylabel('Acceleration [m/s^2]');
% % title('Navigation Frame Acceleration Components');
% % legend('a_N', 'a_E', 'a_D');
% % grid on;
% % figure;
% % plot(time, vel_n(:,1), 'r', 'LineWidth', 1.5); hold on;
% % plot(time, vel_n(:,2), 'g', 'LineWidth', 1.5);
% % plot(time, vel_n(:,3), 'b', 'LineWidth', 1.5);
% % xlabel('Time [s]');
% % ylabel('Acceleration [m/s^2]');
% % title('Velocity');
% % legend('a_N', 'a_E', 'a_D');
% % grid on;
% % figure;
% % plot(time, pos_ned_wp(:,1), 'r', 'LineWidth', 1.5); hold on;
% % plot(time, pos_ned_wp(:,2), 'g', 'LineWidth', 1.5);
% % plot(time, pos_ned_wp(:,3), 'b', 'LineWidth', 1.5);
% % xlabel('Time [s]');
% % ylabel('Acceleration [m/s^2]');
% % title('position');
% % legend('a_N', 'a_E', 'a_D');
% % grid on;


