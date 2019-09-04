%% Angle dynamic parameters
%k_p1 = 1.005753;
%t_p1 = 0.094085;

k_p2 = 1.005572;
zeta_p2 = 1.000000;
wn_p2 = 10.672720;

%k_r1 = 1.008643;
%t_r1 = 0.104289;

k_r2 = 1.001888;
zeta_r2 = 1.000000;
wn_r2 = 12.485386;

%k_y1 = 1.006843;
%t_y1 = 0.180402;

k_y2 = 1.0; %0.972472;
zeta_y2 = 1.000000;
wn_y2 = 9.875759;

%% Controller params
x_kp = 1.4;
x_ki = 0.0;
x_kd = 0.05;
vx_kp = 0.3;
vx_ki = 0.0;
vx_kd = 0.0;

y_kp = 1.4;
y_ki = 0.0;
y_kd = 0.05;
vy_kp = 0.3;
vy_ki = 0.0;
vy_kd = 0.0;

%% Other parameters
g = 9.81;
Ts = 0.01;
Tsim = 10;
x_initial = 0;
y_initial = 0;
yaw_initial = 0;

%% Transfer function and state space
s = tf('s');
pitch_tf = k_p2*wn_p2^2/(s^2 + 2*zeta_p2*wn_p2*s + wn_p2^2);
[num, den] = tfdata(pitch_tf, 'v');
[Ap, Bp, Cp, Dp] = tf2ss(num, den);

roll_tf = -k_r2*wn_r2^2/(s^2 + 2*zeta_r2*wn_r2*s + wn_r2^2);
[num, den] = tfdata(roll_tf, 'v');
[Ar, Br, Cr, Dr] = tf2ss(num, den);

yaw_tf = k_y2*wn_y2^2/(s^2 + 2*zeta_y2*wn_y2*s + wn_y2^2);
[num, den] = tfdata(yaw_tf, 'v');
[Ay, By, Cy, Dy] = tf2ss(num, den);