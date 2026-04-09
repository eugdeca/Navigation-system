%% H calculation
% y_pred = h(x_pred) is the function that gives the measurement
% corresponding to a given state vector
% H = dh/dx is the jacobian of h relative to the state variable x

syms north east down            % pos (NED)
syms vel_no vel_e vel_d          % vel (nav frame)
syms acc_x acc_y acc_z          % acc (body frame)
syms q_w q_x q_y q_z            % quaternion (from nav to body)
syms w_x w_y w_z                % omega (body frame)
syms byas_gx byas_gy byas_gz    % byass gyro
syms nort_P east_P down_P

% Measure = 12-dimensional vector
% measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_x, acc_y, acc_z), Mag (n, e, d), gyro (wx, wy, wz)]

% State = 19-dimensional vector
% [north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]
x = [north; east; down; vel_no; vel_e; vel_d; acc_x; acc_y; acc_z; q_w; q_x; q_y; q_z; w_x; w_y; w_z; byas_gx; byas_gy; byas_gz];

% UWB measure
N = size(P_bases_ned.Value, 1);

P_bases_ned_sym = sym('a', [N, 3]);
d = sym(zeros(N, 1));  % <-- make d symbolic!

for i = 1:N
    diff = x(1:3) - P_bases_ned_sym(i, :)';
    d(i) = sqrt(diff(1)^2 + diff(2)^2 + diff(3)^2);
end

% Substitute symbolic beacon positions with numeric values
UWB_d = d;  % initialize UWB_d as symbolic vector
beacon_positions_numeric = P_bases_ned.Value; % extract raw numeric matrix


for i = 1:N
    for j = 1:3
        UWB_d = subs(UWB_d, P_bases_ned_sym(i, j), beacon_positions_numeric(i, j));
    end
end


% accelerometer measure

R_nb = [q_w^2 + q_x^2 - q_y^2 - q_z^2,     2*(q_x*q_y - q_w*q_z),           2*(q_x*q_z + q_w*q_y);
        2*(q_x*q_y + q_w*q_z),             q_w^2 - q_x^2 + q_y^2 - q_z^2,   2*(q_y*q_z - q_w*q_x);
        2*(q_x*q_z - q_w*q_y),             2*(q_y*q_z + q_w*q_x),           q_w^2 - q_x^2 - q_y^2 + q_z^2];
g_b = R_nb * g_n;
accel_x = acc_x - g_b(1);
accel_y = acc_y - g_b(2);
accel_z = acc_z - g_b(3);

% magnetometer measure
% R_nb = [q_w^2 + q_x^2 - q_y^2 - q_z^2,     2*(q_x*q_y - q_w*q_z),           2*(q_x*q_z + q_w*q_y);
%         2*(q_x*q_y + q_w*q_z),             q_w^2 - q_x^2 + q_y^2 - q_z^2,   2*(q_y*q_z - q_w*q_x);
%         2*(q_x*q_z - q_w*q_y),             2*(q_y*q_z + q_w*q_x),           q_w^2 - q_x^2 - q_y^2 + q_z^2];
% 
% mag_sym = R_nb * [0.4970    0.0317    0.8672]';
% mag_sym_1 = mag_sym(1);
% mag_sym_2 = mag_sym(2);
% mag_sym_3 = mag_sym(3);
% TRIAD measure
sym_qw = q_w;
sym_qx = q_x;
sym_qy = q_y;
sym_qz = q_z;

% gyroscope measure
gyro_x = w_x + byas_gx;
gyro_y = w_y + byas_gy;
gyro_z = w_z + byas_gz;
 

% Jacobian computation
Jacob_UWB_d = jacobian(UWB_d, x);
Jacob_accel_n = jacobian(accel_x, x);
Jacob_accel_e = jacobian(accel_y, x);
Jacob_accel_d = jacobian(accel_z, x);
Jacob_triadw = jacobian(sym_qw, x);
Jacob_triadx = jacobian(sym_qx, x);
Jacob_triady = jacobian(sym_qy, x);
Jacob_triadz = jacobian(sym_qz, x);
Jacob_gyrox = jacobian(gyro_x, x);
Jacob_gyroy = jacobian(gyro_y, x);
Jacob_gyroz = jacobian(gyro_z, x);

Jacob_H = [
        Jacob_UWB_d;
        Jacob_accel_n;
        Jacob_accel_e;
        Jacob_accel_d;
        Jacob_triadw;
        Jacob_triadx;
        Jacob_triady;
        Jacob_triadz;
        Jacob_gyrox;
        Jacob_gyroy;
        Jacob_gyroz;
    ];

H_func = matlabFunction(Jacob_H, 'File','H', 'Vars', {x});