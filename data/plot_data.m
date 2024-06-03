%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ELMO joint data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clc; clear;

% Load data
time = importdata('time.csv');
joint_data = importdata('data.csv');
joint_ref_data = importdata('commands.csv');
status_data = importdata('diagnostics.csv');

% extract the joint data
joint_pos = joint_data(:,1:6);           % joint position read
joint_vel = joint_data(:,7:12);          % joint velocity read
joint_tau = joint_data(:,13:18);         % joint torque computed
joint_pos_ref = joint_ref_data(:,1:6);   % joint position desired
joint_vel_ref = joint_ref_data(:,7:12);  % joint velocity desired
joint_tau_ref = joint_ref_data(:,13:18); % joint torque feedforward

% extract the status data
inputs = status_data(:,1:6);    % digital inputs
controls = status_data(:,7:12); % control word
status = status_data(:,13:18);  % status word

% desired time range
start_time = 0;
end_time = time(end);
idx = time >= start_time & time <= end_time;
time = time(idx);
joint_pos = joint_pos(idx,:);
joint_vel = joint_vel(idx,:);
joint_tau = joint_tau(idx,:);
joint_pos_ref = joint_pos_ref(idx,:);
joint_vel_ref = joint_vel_ref(idx,:);
joint_tau_ref = joint_tau_ref(idx,:);
inputs = inputs(idx,:);
controls = controls(idx,:);
status = status(idx,:);

% labels
labels = ["Left Hip Frontal", "Left Hip Sagittal", "Left Knee", ...
          "Right Hip Frontal", "Right Hip Sagittal", "Right Knee"];

% plot the joint states read
figure('Name','Joint States');
tabgp = uitabgroup;
for i = 1:6
    
    tab = uitab(tabgp, 'Title', labels(i));
    axes('Parent', tab);

    % joint position
    subplot(3,1,1);
    plot(time, joint_pos(:,i)); hold on; 
    plot(time, joint_pos_ref(:,i));          
    title('Position');
    xlabel('Time (s)');
    legend(["Actual Pos (rad)", "Desired Pos (rad)"]);
    grid on;

    % joint velocities
    subplot(3,1,2);
    plot(time, joint_vel(:,i)); hold on; 
    plot(time, joint_vel_ref(:,i));           
    title('Velocity');
    xlabel('Time (s)');
    legend(["Actual Vel (rad/s)", "Desired Vel (rad/s)"]);
    grid on;

    % joint torque
    subplot(3,1,3);
    plot(time, joint_tau(:,i)); hold on;  % plot the joint torque
    plot(time, joint_tau_ref(:,i)); % plot the feedforward torque

    title('Torque');
    xlabel('Time (s)');
    legend(["Computed Torque (Nm)", "Feedforward Torque (Nm)"]);
    grid on;

end

% plot diagnostic data
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

% plot data rates
figure('Name','Data Rates');
dt = time(2:end) - time(1:end-1);
dt_ms = dt * 1000;
plot(time(2:end), dt_ms);
xlabel('Time [s]');
ylabel('dt [ms]');

mean = mean(dt_ms);
std = std(dt_ms);
mesg = sprintf('Data Rates \n Mean: %.3f ms, Std: %.3f ms', mean, std);
title(mesg);
