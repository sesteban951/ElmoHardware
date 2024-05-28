%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ELMO joint data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; clear;

% Load data
data = importdata('data.csv');
status_data = importdata('diagnostics.csv');

% extract the data
time = data(:,1);
joint_pos = data(:,2:7);
joint_vel = data(:,8:13);
joint_tau = data(:,14:19);

status = status_data(:,2:7);

% desired time range
start_time = 0;
end_time = time(end);
idx = time >= start_time & time <= end_time;
time = time(idx);
joint_pos = joint_pos(idx,:);
joint_vel = joint_vel(idx,:);
joint_tau = joint_tau(idx,:);
status = status(idx,:);

% tab plot the left leg. 
figure('Name','Joint Positions');
labels = ["Left Hip Frontal", "Left Hip Sagittal", "Left Knee", ...
          "Right Hip Frontal", "Right Hip Sagittal", "Right Knee"];
legnd = ["Position (rad)", "Velocity (rad/s)", ""];

tabgp = uitabgroup;
for i = 1:6
    
    tab = uitab(tabgp, 'Title', labels(i));
    axes('Parent', tab);

    subplot(2,1,1);
    plot(time, joint_pos(:,i)); hold on;  % plot the joint position
    plot(time, joint_vel(:,i));           % plot the joint velocity

    title('Joint Position and Velocity');
    xlabel('Time (s)');
    % yline(0);
    legend(legnd);
    grid on;

    subplot(2,1,2);
    plot(time, joint_tau(:,i)); hold on;  % plot the joint torque

    title('Torque applied');
    xlabel('Time (s)');
    % yline(0);
    grid on;

end

figure('Name','Status Word');
for i = 1:6
    subplot(2,3,i);
    plot(time, status(:,i));
    title(labels(i));
    xlabel('Time (s)');
    ylabel('Status');
    grid on;
end
