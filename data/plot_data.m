%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ELMO joint data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; clear;

% Load data
joint_data = importdata('data.csv');
status_data = importdata('diagnostics.csv');

% extract the joint data
time = joint_data(:,1);
joint_pos = joint_data(:,2:7);
joint_vel = joint_data(:,8:13);
joint_tau = joint_data(:,14:19);

% extract the status data
inputs = status_data(:,2:7);
controls = status_data(:,8:13);
status = status_data(:,14:19);

% desired time range
start_time = 0;
end_time = time(end);
idx = time >= start_time & time <= end_time;
time = time(idx);
joint_pos = joint_pos(idx,:);
joint_vel = joint_vel(idx,:);
joint_tau = joint_tau(idx,:);
inputs = inputs(idx,:);
controls = controls(idx,:);
status = status(idx,:);

% labels
labels = ["Left Hip Frontal", "Left Hip Sagittal", "Left Knee", ...
          "Right Hip Frontal", "Right Hip Sagittal", "Right Knee"];

figure('Name','Joint States');
tabgp = uitabgroup;
for i = 1:6
    
    tab = uitab(tabgp, 'Title', labels(i));
    axes('Parent', tab);

    subplot(2,1,1);
    plot(time, joint_pos(:,i)); hold on;  % plot the joint position
    plot(time, joint_vel(:,i));           % plot the joint velocity

    title('Joint Position and Velocity');
    xlabel('Time (s)');
    legend(["Position (rad)", "Joint Velocity (rad/s)"]);
    grid on;

    subplot(2,1,2);
    plot(time, joint_tau(:,i)); hold on;  % plot the joint torque

    title('Torque applied');
    xlabel('Time (s)');
    grid on;

end

figure('Name','Diagnostic Data');
tabgp = uitabgroup;
for i = 1:6
    
    tab = uitab(tabgp, 'Title', labels(i));
    axes('Parent', tab);

    subplot(3,1,1);
    plot(time, inputs(:,i)); hold on;   % plot the digital inputs
    title('Inputs');
    xlabel('Time (s)');
    grid on;

    subplot(3,1,2);
    plot(time, controls(:,i)); hold on; % plot the control word
    title('Control Word');
    xlabel('Time (s)');
    grid on;

    subplot(3,1,3);
    plot(time, status(:,i)); hold on;   % plot the status word
    title('Status Word');
    xlabel('Time (s)');
    grid on;
end
