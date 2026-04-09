clear all
close all
clc


% Time [s], Position NED [m], Orientation [Yaw Pitch Roll] in deg
waypoints_ned = [ ...
    0,    0,   0,   0,     0,   0,    0;       % Start point
    10,   5,   3,  -1,    45,  10,   30;       % Slight move, yaw right, pitch up, roll
    20,  12,   6,  -2,    90, -20,  -30;       % Turn further, pitch down
    30,  20,  10,  -3,   135,  15,   20;       % Turn left + roll right
    40,  25,   5,   -1,   180, -15,   45;       % Ascend with large roll and yaw
    50,  30,   0,   0,   225,   0,    0;       % End point with full yaw shift
];

ST = 1/100;  % Sample time
