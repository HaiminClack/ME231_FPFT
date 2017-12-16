clear; close all; clc

% Car width and length
cw = 1.6/2;
cl = 3.0/2;
carShape = [cw,cl];

% Opponents
car1_x = -2.0;
car1_y = 8;
car2_x = -3.5;
car2_y = 16;
car3_x = 3.5;
car3_y = 24;

% sensing range [ x_lower, x_upper, y_lower, y_upper ]       
XYbounds =  [ -5, 5, 0, 30 ];   

% initial state for planning
x0_plan = [0 0 0 0];

% target state
yend = 30;
xF = [0 yend 0 0];

% shape of obstacles
nOb =  3; 	% number of obstacles 
vOb = [4 4 4];	% number of vertices of each obstacle, vector of dimenion nOb
rho = 1.0;
ob1 = getCarVertice(car1_x,car1_y,rho*cw,rho*cl,1);
ob2 = getCarVertice(car2_x,car2_y,rho*cw,rho*cl,1);
ob3 = getCarVertice(car3_x,car3_y,rho*cw,rho*cl,1);
lOb = getlOb(ob1,ob2,ob3);
% plotObs(nOb,vOb,lOb,x0,xF,XYbounds)

% initial state for tracking
x0_track = [0 0 0 pi/2];

% Simulation time per planning loop
Tf = 6;

for i = 1:30
    states = plannerMPC(XYbounds,x0_plan,xF,nOb,vOb,lOb);
    
    [states_MPC,states_SIM,controls_MPC,ref_MPC,average_runtime] = track(states,x0_track,Tf);
    
    figure(1)
    for j = 1:length(states_SIM)-1
        plot(states_MPC{j}(:,1),states_MPC{j}(:,2),'g') % Plot MPC traj
        hold on
        plot(ref_MPC{j}(1,1),ref_MPC{j}(2,1),'ko')      % Plot Reference point
        hold on
        carBox([states_SIM(j,1);states_SIM(j,2)],states_SIM(j,4),cw,cl);
        hold on
        pause(0.05)
        xlim([-5,5])
        ylim([states(1,3)-10,states(end,3)])
    end
   
    vcurrent = mean(states_SIM(:,3));
    delta_y = vcurrent*Tf;
    yend = yend + delta_y;
    
    x0_plan = [states_SIM(end,1) states_SIM(end,2) 0 0];
    xF = [0 yend 0 0];
    XYbounds =  [ -5, 5, 0, yend ]; 
    % Opponents
    car1_x = -3.5 + 2.5*(1-2*rand);
    car1_y = car1_y+delta_y+1*(1-2*rand);
    car2_x = 0 + 4*(1-2*rand);
    car2_y = car2_y+delta_y+1*(1-2*rand);
    car3_x = 3.5 + 2.5*(1-2*rand);
    car3_y = car3_y+delta_y+1*(1-2*rand);
    ob1 = getCarVertice(car1_x,car1_y,rho*cw,rho*cl,1);
    ob2 = getCarVertice(car2_x,car2_y,rho*cw,rho*cl,1);
    ob3 = getCarVertice(car3_x,car3_y,rho*cw,rho*cl,1);
    lOb = getlOb(ob1,ob2,ob3);
   
    x0_track = states_SIM(end,:);
end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017