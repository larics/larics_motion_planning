%% To make custom messages
% roboticAddons -> this starts gui in which you can install tool for genmsg
% rosgenmsg('/home/antun/catkin_ws/src')

%% 
x_ref.time = [0:0.01:10]';
x_ref.signals.values = linspace(0, 1, 201)';
x_ref.signals.values = [x_ref.signals.values; linspace(1, 1, 800)'];

vx_ff = x_ref;
vx_ff.signals.values = vx_ff.signals.values*0;

y_ref = x_ref;
y_ref.signals.values = y_ref.signals.values*0;

vy_ff = x_ref;
vy_ff.signals.values = vy_ff.signals.values*0;

yaw_ref = x_ref;
yaw_ref.signals.values = yaw_ref.signals.values*0;

%%
rosinit('http://antun-ethernet:11311', ...
           'NodeHost', 'antun-ethernet');
%%
simulate_roll_pitch_service = rossvcserver('/simulate_arducopter', ...
    'larics_motion_planning/MultiDofTrajectory', @simulateRollPitchCallback);