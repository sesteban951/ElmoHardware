%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot sample state machine data (control deployed on target PC)
% % ELMO order of joints (physical default order)
%   1. HFL  (Hip Frontal Left)
%   2. HSL  (Hip Sagittal Left)
%   3. HSR  (Hip Sagittal Right)
%   4. KL   (Knee Left)
%   5. HFR  (Hip Frontal Right)
%   6. KR   (Knee Right)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data = load('sw_cw.mat')

cw = data.CW;
sw = data.SW;
[r, c] = size(cw)

hz = 500;
t = linspace(0, r/hz, r);

% plot the control word
figure(1); grid on;
sgtitle('Control Word')

subplot(2,3,1);
plot(t, cw(:,1),'r')
title("Hip Frontal Left")
subplot(2,3,2)
plot(t, cw(:,2),'r')
title("Hip Sagittal Left")
subplot(2,3,3)
plot(t, cw(:,3),'r')
title("Hip Sagittal Right")
subplot(2,3,4)
plot(t, cw(:,4),'r')
title("Knee Left")
subplot(2,3,5)
plot(t, cw(:,5),'r')
title("Hip Frontal Right")
subplot(2,3,6)
plot(t, cw(:,6),'r')
title("Knee Right")


% plot the status word
figure(2);
sgtitle('Status Word')
subplot(2,3,1)
plot(t, sw(:,1),'b')
title("Hip Frontal Left")
subplot(2,3,2)
plot(t, sw(:,2),'b')
title("Hip Sagittal Left")
subplot(2,3,3)
plot(t, sw(:,3),'b')
title("Hip Sagittal Right")
subplot(2,3,4)
plot(t, sw(:,4),'b')
title("Knee Left")
subplot(2,3,5)
plot(t, sw(:,5),'b')
title("Hip Frontal Right")
subplot(2,3,6)
plot(t, sw(:,6),'b')
title("Knee Right")