%% F calculation
% 19-dimensional state vector
syms north east down            % pos (NED)
syms vel_n vel_e vel_d          % vel (nav frame)
syms acc_x acc_y acc_z          % acc (body frame)
syms q_w q_x q_y q_z            % quaternion (from nav to body)
syms w_x w_y w_z                % omega (body frame)
syms byas_gx byas_gy byas_gz    % bias gyro
syms dt

% Rotation matrix from quaternion (nav to body)
q = [q_w, q_x, q_y, q_z];
R_nb = [q_w^2 + q_x^2 - q_y^2 - q_z^2,     2*(q_x*q_y - q_w*q_z),           2*(q_x*q_z + q_w*q_y);
        2*(q_x*q_y + q_w*q_z),             q_w^2 - q_x^2 + q_y^2 - q_z^2,   2*(q_y*q_z - q_w*q_x);
        2*(q_x*q_z - q_w*q_y),             2*(q_y*q_z + q_w*q_x),           q_w^2 - q_x^2 - q_y^2 + q_z^2];


% Convert body-frame acceleration to navigation frame
acc_ned = R_nb' * [acc_x; acc_y; acc_z];
acc_n = acc_ned(1);
acc_e = acc_ned(2);
acc_d = acc_ned(3);

% positions
north_sym = north + vel_n*dt + 0.5*acc_n*dt^2;
east_sym = east + vel_e*dt + 0.5*acc_e*dt^ 2;
down_sym = down + vel_d*dt + 0.5*acc_d*dt^2;

% vel
vel_sym_n = vel_n + acc_n*dt;
vel_sym_e = vel_e + acc_e*dt;
vel_sym_d = vel_d + acc_d*dt;

% acc (unmodified)
accx_sym = acc_x;
accy_sym = acc_y;
accz_sym = acc_z;

% attitude - quaternion update
omega = [w_x, w_y, w_z];  % angular velocity
% theta = norm(omega.*dt); 
% axis = omega / norm(omega);  % rotation axis
% delta_q = [cos(theta/2), axis.*sin(theta/2),];  % [w x y z] form

d_angle = omega.* dt;
delta_q = [cos(d_angle(1)/2)*cos(d_angle(2)/2)*cos(d_angle(3)/2)+sin(d_angle(1)/2)*sin(d_angle(2)/2)*sin(d_angle(3)/2);
       sin(d_angle(1)/2)*cos(d_angle(2)/2)*cos(d_angle(3)/2)-cos(d_angle(1)/2)*sin(d_angle(2)/2)*sin(d_angle(3)/2);
       cos(d_angle(1)/2)*sin(d_angle(2)/2)*cos(d_angle(3)/2)+sin(d_angle(1)/2)*cos(d_angle(2)/2)*sin(d_angle(3)/2);
       cos(d_angle(1)/2)*cos(d_angle(2)/2)*sin(d_angle(3)/2)-sin(d_angle(1)/2)*sin(d_angle(2)/2)*cos(d_angle(3)/2)];

s_w = q_w*delta_q(1) - q_x*delta_q(2) - q_y*delta_q(3) - q_z*delta_q(4);
s_x = q_w*delta_q(2) + q_x*delta_q(1) + q_y*delta_q(4) - q_z*delta_q(3);
s_y = q_w*delta_q(3) - q_x*delta_q(4) + q_y*delta_q(1) + q_z*delta_q(2);
s_z = q_w*delta_q(4) + q_x*delta_q(3) - q_y*delta_q(2) + q_z*delta_q(1);
% % Components of delta_q
% dq_w = delta_q(1);
% dq_x = delta_q(2);
% dq_y = delta_q(3);
% dq_z = delta_q(4);
% 
% % Now compute: q_new = delta_q ⊗ q
% s_w = dq_w*q_w - dq_x*q_x - dq_y*q_y - dq_z*q_z;
% s_x = dq_w*q_x + dq_x*q_w + dq_y*q_z - dq_z*q_y;
% s_y = dq_w*q_y - dq_x*q_z + dq_y*q_w + dq_z*q_x;
% s_z = dq_w*q_z + dq_x*q_y - dq_y*q_x + dq_z*q_w;

new_quat = [s_w; s_x; s_y; s_z];

q_w_sym = new_quat(1);
q_x_sym = new_quat(2);
q_y_sym = new_quat(3);
q_z_sym = new_quat(4);

% omega (unmodified)
wx_sym = w_x;
wy_sym = w_y;
wz_sym = w_z;

% gyroscope bias (unmodified)
bgx_sym = byas_gx;
bgy_sym = byas_gy;
bgz_sym = byas_gz;

% output:
% Jacobians computation
Jacob_north = jacobian(north_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_east  = jacobian(east_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_down  = jacobian(down_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_veln  = jacobian(vel_sym_n,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_vele  = jacobian(vel_sym_e,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_veld  = jacobian(vel_sym_d,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accx  = jacobian(accx_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accy  = jacobian(accy_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accz  = jacobian(accz_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_qw = jacobian(q_w_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_qx = jacobian(q_x_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_qy = jacobian(q_y_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_qz = jacobian(q_z_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_wx = jacobian(wx_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_wy = jacobian(wy_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_wz = jacobian(wz_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_bx = jacobian(bgx_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_by = jacobian(bgy_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_bz = jacobian(bgz_sym,[north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);

Jacob_F = [
    Jacob_north;
    Jacob_east;
    Jacob_down;
    Jacob_veln;
    Jacob_vele;
    Jacob_veld;
    Jacob_accx;
    Jacob_accy;
    Jacob_accz;
    Jacob_qw;
    Jacob_qx;
    Jacob_qy;
    Jacob_qz;
    Jacob_wx;
    Jacob_wy;
    Jacob_wz;
    Jacob_bx;
    Jacob_by;
    Jacob_bz
    ];

F_func = matlabFunction(Jacob_F,'File','F_matrix','Vars', {north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz, dt});
