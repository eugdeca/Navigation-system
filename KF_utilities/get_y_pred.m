function y_pred = get_y_pred(x, g, P_bases_ned)
%GET_Y_PRED returns the y predicted based on the x predicted
%   Measure = 12-dimensional vector
%   measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_x, acc_y, acc_z), Mag (n, e, d), gyro (wx, wy, wz)]
%   State = 19-dimensional vector
%   [north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]

% Prediction
    north = x(1); east = x(2); down = x(3);         % pos (NED)
    vel_n = x(4); vel_e = x(5); vel_d = x(6);       % vel (nav frame)
    acc_x = x(7); acc_y = x(8); acc_z = x(9);       % acc (body frame)
    q_w = x(10); q_x = x(11); q_y = x(12); q_z = x(13); % quaternion (from nav to body)
    w_x = x(14); w_y = x(15); w_z = x(16);          % omega
    bwx = x(17); bwy = x(18); bwz = x(19);          % byass gyro
    q = quaternion([q_w q_x q_y q_z]);
    g_n = [0;0;9.81];
    % pos - UWB
    N = size(P_bases_ned, 1);
    pred_d = zeros(N, 1);

    for i = 1:N
        diff = [north; east; down] - P_bases_ned(i, :)';
        pred_d(i) = sqrt(diff(1)^2 + diff(2)^2 + diff(3)^2);
    end
    
    %g_b = rotateframe(q,[0 0 9.81]);
    % acc
    g_b = rotmat(q,"frame")*g_n;
    pred_accx = acc_x - g_b(1);
    pred_accy = acc_y - g_b(2);
    % pred_accz = acc_z - g;
    pred_accz = acc_z - g_b(3);

    % Triad
    pred_qw = q_w;
    pred_qx = q_x;
    pred_qy = q_y;
    pred_qz = q_z;

    % gyro
    pred_gyro_x = w_x + bwx;
    pred_gyro_y = w_y + bwy; 
    pred_gyro_z = w_z + bwz;

    % PITOT:
    % pred_vel_north = vel_n;

    % y_pred = [pred_north, pred_east, pred_down, pred_accx, pred_accy, pred_accz, pred_mag1, pred_mag2, pred_mag3, pred_gyro_x, pred_gyro_y, pred_gyro_z, pred_vel_north]';
    y_pred = [pred_d', pred_accx, pred_accy, pred_accz, pred_qw, pred_qx,pred_qy,pred_qz, pred_gyro_x, pred_gyro_y, pred_gyro_z]';

end

