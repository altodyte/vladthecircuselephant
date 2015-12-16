% figure('OuterPosition',[800+1 40+1 800 900-40]);
filename = '20151216T101321_stable_HFwobble_P75I0.0005D1.0.csv';
M = csvread(filename);
timeVec = M(:,1);
pitchPsi = M(:,2);
pitchPhi = M(:,3);
pitchVa = M(:,4);
rollPsi = M(:,5);
rollPhi = M(:,6);
rollVa = M(:,7);
plot(timeVec, pitchPsi, 'b.-', 'linewidth', 2);
hold all
plot(timeVec, pitchPhi, 'b--', 'linewidth', 2);
plot(timeVec, pitchVa/12, 'b-', 'linewidth', 1);
plot(timeVec, rollPsi, 'g.-', 'linewidth', 2);
plot(timeVec, rollPhi, 'g--','linewidth', 2);
plot(timeVec, rollVa/12, 'g-', 'linewidth', 1);
legend('pitchPsi', 'pitchPhi', 'pitchVa', ...
    'rollPsi', 'rollPhi', 'rollVa', ...
    'location', 'best');
xlabel('time [s]');
ylabel('value [rad or rad/s or 12 V]');