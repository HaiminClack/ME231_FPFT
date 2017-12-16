refx = [];
refy = [];
for i = 1:61
    refx = [refx ref_MPC{i}(1)];
    refy = [refy ref_MPC{i}(2)];
end

subplot(5,1,1)
plot(refx,'b')
hold on
plot(states_SIM(:,1),'r')
grid on
legend('reference','actual state')
title('X')
xlim([0,60])

subplot(5,1,2)
plot(refy,'b')
hold on
plot(states_SIM(:,2),'r')
grid on
legend('reference','actual state')
title('Y')
xlim([0,60])

subplot(5,1,3)
plot(states_SIM(:,3),'b')
grid on
title('Velocity')
xlim([0,60])

subplot(5,1,4)
plot(controls_MPC(:,1),'b')
grid on
title('a')
xlim([0,60])

subplot(5,1,5)
plot(controls_MPC(:,2),'b')
grid on
title('delta')
xlim([0,60])

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017