
%%%%%%%%%%%%%%%
% Computes trajectory by ignoring the obstacles, used for initialization
%%%%%%%%%%%%%%%

function [xSol, timeSol, stateSol, controlSol, solverTime, out] = WarmStart(N,Ts,x0,xF,XYbounds)

    BEGIN_ACADO;  

        acadoSet('problemname', 'WarmStart');           % Set your problemname. If you 
                                                        % skip this, all files will
                                                        % be named "myAcadoProblem"  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % defining optimization variables
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                           
        DifferentialState x;                            % DifferentialState
        DifferentialState y;    
        DifferentialState vx;    
        DifferentialState vy;
        DifferentialState timeScale;
        DifferentialState L;

        Control ax;                                     % Control
        Control ay;    

        f = acado.DiscretizedDifferentialEquation(Ts);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % cost function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % (min time)+
        % (min states/control inputs)      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Q = blkdiag(0,0,0,0);
        R = blkdiag(1,1);
        f.add( next(L) == 0.5*timeScale + 1*timeScale^2 +...
            [x; y; vx; vy]'*Q*[x; y; vx; vy] + [ax; ay]'*R*[ax; ay]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % dynamics of the car
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % - Double Integrator with forward Euler integration 
        % States:[x,y,vx,vy]  Control inputs:[ax ay]
        % - sampling time scaling, is identical over the horizon
        f.add(next(x)  == x + timeScale*Ts*vx);
        f.add(next(y)  == y + timeScale*Ts*vy);
        f.add(next(vx) == vx + timeScale*Ts*ax);
        f.add(next(vy) == vy + timeScale*Ts*ay);
        f.add(next(timeScale) == timeScale);
        
%         % - Dubins car model with forward Euler integration 
%         % - States:[x,y,theta,v]  Control inputs:[us, ua]
%         % - sampling time scaling, is identical over the horizon
%         f.add(next(x) == x + timeScale*Ts*v*cos(theta));
%         f.add(next(y) == y + timeScale*Ts*v*sin(theta));
%         f.add(next(theta) == theta + timeScale*Ts*us);
%         f.add(next(v) == v + timeScale*Ts*ua);
%         f.add(next(timeScale) == timeScale);    

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Optimal Control Problem
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ocp = acado.OCP(0.0, N, N);                     % Set up the Optimal Control Problem (OCP)
                                                        % Start at 0s, control in 20 intervals upto 10s
        ocp.minimizeMayerTerm( L );                     % Minimize this Mayer Term
        ocp.subjectTo( f );                             % Your OCP is always subject to your 
                                                        % differential equation

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % bounds on states and inputs
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ocp.subjectTo( XYbounds(1) <= x <= XYbounds(2)  );
    	ocp.subjectTo( XYbounds(3) <= y <= XYbounds(4)  );
        ocp.subjectTo( -3.0 <= ax <= 3.0  );
        ocp.subjectTo( -3.0 <= ay <= 3.0  );
        ocp.subjectTo( 0.5 <= timeScale <= 2.5  );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % start and finish point
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ocp.subjectTo( 'AT_START', L  == 0.0    );
        ocp.subjectTo( 'AT_START', x == x0(1)  );
        ocp.subjectTo( 'AT_START', y == x0(2)  );
        ocp.subjectTo( 'AT_START', vx == x0(3)  );
        ocp.subjectTo( 'AT_START', vy == x0(4)  );
        ocp.subjectTo( 'AT_END',   x == xF(1)  );
        ocp.subjectTo( 'AT_END',   y == xF(2)  );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set up the optimization algorithm
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        algo = acado.OptimizationAlgorithm( ocp );
        algo.set( 'MAX_NUM_ITERATIONS', 750 );
        algo.set( 'KKT_TOLERANCE', 1e-1 );
        algo.set( 'INTEGRATOR_TYPE', 'INT_RK45');
    %     algo.set( 'INTEGRATOR_TOLERANCE',   1e-3);  
    %     algo.set( 'ABSOLUTE_TOLERANCE',   1e-3   );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % set initial guess
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xINIT = zeros(N+1,7);   % Order: timestamp,x,y,vx,vy,timeScale,L
        xINIT(:,1) = 0:1:N;
        for i = 2:5
            xINIT(:,i) = linspace(x0(i-1),xF(i-1),N+1);
        end
        xINIT(:,6) = 0.5*ones(N+1,1);

        uINIT = zeros(N,3);     % Order: timestamp,ax,ay
        uINIT(:,1) = 0:1:N-1;
    	uINIT(:,2) = 0.6*ones(N,1);
        uINIT(:,3) = 0.6*ones(N,1);
        algo.initializeDifferentialStates(xINIT);
        algo.initializeControls(uINIT);

    END_ACADO;                                      % Always end with "END_ACADO".
                                                    % This will generate a file problemname_ACADO.m. 
                                                    % Run this file to get your results. 
                                                    % You can run the file problemname_ACADO.m as
                                                    % many times as you want without having to compile again.
    tic()
    out = WarmStart_RUN();                          % Run the test. The name of the RUN file
                                                    % is problemname_RUN, so in
                                                    % this case active_damping_RUN
    time = toc();

    % print solution time
    fprintf('Elapsed time: %f\n',time);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % return values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xSol = out.STATES(:,2:5);
    timeSol = out.STATES(:,6);
    stateSol = out.STATES;
    controlSol = out.CONTROLS;
    solverTime = time;
    
    save outWS out
    
end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017