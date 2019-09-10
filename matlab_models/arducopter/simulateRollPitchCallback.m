function resp = simulateRollPitchCallback(~, req,resp)
%simulateRollPitchCallback Callback function for a service to simulate roll
%and pitch angles from input trajectory.
%   RESP = simulateRollPitchCallback(~,REQ,RESP) Takes input trajectory and
%   runs simulation with identified model to get roll and pitch angles for
%   every discretization time in trajectory
%
%   And empty response message is given as an argument to this function and
%   after assigning data to it, it has to be returned as a single output
%   argument. This output will be sent to the service client.
%
%   See also ROSServicesExample.
tic
trajectory_length = size(req.Waypoints.Points, 1);
% Set up parameters for the simulation
arducopter_params;
evalin('base', 'arducopter_params');
% Simulation time is 2s longer to allow for transient effect to settle
Tsim = trajectory_length*Ts + 10.0;

% Set up time for simulation
x_ref.time = [0:Ts:Tsim]';
y_ref.time = [0:Ts:Tsim]';
yaw_ref.time = [0:Ts:Tsim]';
vx_ff.time = [0:Ts:Tsim]';
vy_ff.time = [0:Ts:Tsim]';
% Set up signal length for simulation
total_length = size(x_ref.time, 1);
x_ref.signals.values = linspace(0, 0, total_length)';
y_ref.signals.values = linspace(0, 0, total_length)';
yaw_ref.signals.values = linspace(0, 0, total_length)';
vx_ff.signals.values = linspace(0, 0, total_length)';
vy_ff.signals.values = linspace(0, 0, total_length)';

% for loop goes that goes through all trajectory points and sets them to
% variables for simulation

for i=1:trajectory_length
    x_ref.signals.values(i, 1) = req.Waypoints.Points(i).Positions(1, 1);
    y_ref.signals.values(i, 1) = req.Waypoints.Points(i).Positions(2, 1);
    yaw_ref.signals.values(i, 1) = req.Waypoints.Points(i).Positions(6, 1);
    vx_ff.signals.values(i, 1) = req.Waypoints.Points(i).Velocities(1, 1);
    vy_ff.signals.values(i, 1) = req.Waypoints.Points(i).Velocities(2, 1);
end

for i=(trajectory_length+1):total_length
    x_ref.signals.values(i, 1) = req.Waypoints.Points(end).Positions(1, 1);
    y_ref.signals.values(i, 1) = req.Waypoints.Points(end).Positions(2, 1);
    yaw_ref.signals.values(i, 1) = req.Waypoints.Points(end).Positions(6, 1);
end

assignin('base', 'Tsim', Tsim);
assignin('base', 'x_ref', x_ref);
assignin('base', 'y_ref', y_ref);
assignin('base', 'yaw_ref', yaw_ref);
assignin('base', 'vx_ff', vx_ff);
assignin('base', 'vy_ff', vy_ff);
assignin('base', 'x_initial', x_ref.signals.values(1, 1));
assignin('base', 'y_initial', y_ref.signals.values(1, 1));
assignin('base', 'yaw_initial', yaw_ref.signals.values(1, 1));
sim_out = sim('arducopter.slx', 'ReturnWorkspaceOutputs', 'on');
disp('Trajectory simulated')
%class(resp.Trajectory.Points)
%resp.Trajectory.Points = req.Waypoints.Points;
%last_point = robotics.ros.msggen.trajectory_msgs.JointTrajectoryPoint;


for i=trajectory_length:total_length
    roll = sim_out.roll.signals.values(i,1);
    pitch = sim_out.pitch.signals.values(i,1);
    if (abs(roll)<1e-4) && (abs(pitch)<1e-4)
        break
    end
end

endpoint = i;
sim_out.pitch.signals.values(1:endpoint,1);

resp.Roll = sim_out.roll.signals.values(1:endpoint,1);
resp.Pitch = sim_out.pitch.signals.values(1:endpoint,1);
%sim_out.roll.signals.values(1:endpoint,1)
%resp.Pitch
%resp.Trajectory.Points = arrayfun(@(~) rosmessage('trajectory_msgs/JointTrajectoryPoint'),zeros(1,endpoint));

% for i=1:endpoint
%     roll = sim_out.roll.signals.values(i,1);
%     pitch = sim_out.pitch.signals.values(i,1);
%     if i <= trajectory_length
%         resp.Trajectory.Points(i) = req.Waypoints.Points(i);
%         resp.Trajectory.Points(i).Positions(4,1) = roll;
%         resp.Trajectory.Points(i).Positions(5,1) = pitch;
%     else
%         resp.Trajectory.Points(i) = copy(req.Waypoints.Points(end));
%         resp.Trajectory.Points(i).Positions(4,1) = roll;
%         resp.Trajectory.Points(i).Positions(5,1) = pitch;
%     end
% end

disp('New trajectory length:')
length(resp.Pitch)

end