%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ELMO joint data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; clear;

% Load data
data = importdata('data.csv');

% extract the data
time = data(:,1);
joint_pos = data(:,2:7);
joint_vel = data(:,8:13);

% desired time range
start_time = 0.5;
end_time = time(end);
idx = time >= start_time & time <= end_time;
time = time(idx);
joint_pos = joint_pos(idx,:);
joint_vel = joint_vel(idx,:);

% tab plot the left leg. 
figure('Name','Joint Positions');
labels = ["Left Hip Frontal", "Left Hip Sagittal", "Left Knee", ...
          "Right Hip Frontal", "Right Hip Sagittal", "Right Knee"];
legnd = ["Position (rad)", "Velocity (rad/s)", ""];

tabgp = uitabgroup;
for i = 1:6
    
    tab = uitab(tabgp, 'Title', labels(i));
    axes('Parent', tab);

    % plot the joint position
    plot(time, joint_pos(:,i)); hold on;

    % plot the joint velocity
    plot(time, joint_vel(:,i)); hold on;

    title('Joint Position and Velocity');
    xlabel('Time (s)');
    yline(0);
    legend(legnd);
    grid on;
end
