function [states_MPC,states_SIM,controls_MPC,ref_MPC,average_runtime] = track(states,x0,Tf)

%% Create the structures and set dimensions
%  Those dimensions must match ones defined in the c++ code generator.
Tend = 0.5;     % MPC time horizon  
N   = 5;       % Sampling points
Ts = Tend/N;    % Sampling interval: Tend/N
NX  = 4;
NU  = 2;
NY  = NX+NU;
NYN = NX;

% External Disturbances
d1 = 1.5;
d2 = 1.5;

% State constraints
xlb = -5.0;
xub =  5.0;
ylb =  states(1,3);
yub =  1e8;
vlb =  0.0;
vub = 5.0;
thetalb = -3.6;
thetaub = 3.6;

% Control bound
ualb = -0.5;
uaub = 0.5;
uslb = -0.8;
usub =  0.8;

% Timing
time_run = [];

%% Interpolate reference trajectory
% xref = interp1(states(:,1),states(:,2),(0:0.5:10)','poly');
% yref = interp1(states(:,1),states(:,3),(0:0.5:10)','poly');
% plot(xref,yref);

%% Fit and uniformly sample the reference trjectory
% ***NOTE*** For the function to be well defined, use y as x in the polyfit
P = fit(states(:,3),states(:,2),'smoothingspline');

%% States initialization
% Initial state
mpcInput.x0 = x0;

% State init
mpcInput.x = zeros(N+1,NX);
mpcInput.u = zeros(N,NU);

% Control constraints
mpcInput.lbValues = repmat([ualb;uslb],N,1);
mpcInput.ubValues = repmat([uaub;usub],N,1);

mpcInput.lbAValues = repmat([xlb;ylb;vlb;thetalb],N,1);
mpcInput.ubAValues = repmat([xub;yub;vub;thetaub],N,1);

% Weighting matrices
mpcInput.W  = diag([10 1 3 0 ... % states
                    1 1]); % controls            
mpcInput.WN = diag([10 1 3 0]);

%% Simulation Init
time = 0;
iter = 0;

states_MPC = {};
ref_MPC = {};
controls_MPC = [];
states_SIM = mpcInput.x0;

%% Control Loop
while time(end) < Tf
    % initial value embedding:
    mpcInput.x0 = states_SIM(end,:);
    
    ystart = states_SIM(end,2);
    
    % give reference trajectory
    xref = [];
    yref = [];
    vdesired = 5.0;
    yend = ystart + vdesired*Ts*N;
    for y = ystart:(yend-ystart)/N:yend
%         xref = [xref polyval(P,y)];
%         yref = [yref y];
        xref = [xref P(yend)];
        yref = [yref yend];
    end
%     xref = polyval(P,yend)*ones(1,N);
%     yref = yend*ones(1,N);
    ref_MPC{end+1} = [xref;yref];
    % plot(states(:,2),states(:,3))
    % hold on
    % plot(xref,yref)
    % xlim([-5,5])
    mpcInput.y  = zeros(N, NY);
    mpcInput.y(:,1)  = xref(1:N);
    mpcInput.y(:,2)  = yref(1:N);
    mpcInput.y(:,3)  = 5.0*ones(N,1);
    
    mpcInput.yN = zeros(1, NYN);
    mpcInput.yN(1,1) = xref(end);
    mpcInput.yN(1,2) = yref(end);
      
    mpcOutput = acado_solver( mpcInput ); 

    disp(['RTI step at ' num2str(time(end)) 's: ' num2str(mpcOutput.info.cpuTime*1e3,'%.2f') ...
          ' ms), KKT = ' num2str(mpcOutput.info.kktValue,'%.2e')...
          ', iter = ' num2str(iter+1)]);
    disp(' ')
    
        if (mpcOutput.info.status ~= 0)
        disp(['QP error, status flag = ' num2str(mpcOutput.info.status)])
        end
    
    time_run = [time_run mpcOutput.info.cpuTime];
    
    % Save MPC results
    states_MPC{end+1} = mpcOutput.x;
    controls_MPC = [controls_MPC; mpcOutput.u(1,:)];
  
    mpcInput.x = mpcOutput.x;
    mpcInput.u = mpcOutput.u;
    
    %% Simulation
    % Disturbance uncertainty realization
    dx = d1*(1-2*rand);
    dy = d2*(1-2*rand);
    
    % Apply first control to the simulator:
    sim_input.x = states_SIM(end,:).';
    sim_input.u = mpcOutput.u(1,:).';
    
    % Exernal disturbance
    sim_input.od(1,1) = dx;
    sim_input.od(2,1) = dy;
    
    % Run simulation
    sim_output = acado_integrator(sim_input);
    states_SIM = [states_SIM; sim_output.value.'];

    % Shift horizon
    mpcInput.x = [mpcInput.x(2:end,:);mpcInput.x(end,:)];
    mpcInput.u = [mpcInput.u(2:end,:);mpcInput.u(end,:)];
    
    % Update timer
    time = [time; iter*Ts];
    iter = iter + 1;
end

average_runtime = mean(time_run);

end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017