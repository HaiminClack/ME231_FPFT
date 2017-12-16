% accurately solve a collision avoidance problem, optimizing the warm start path

function [stateSol, controlSol, solverTime, outSol] = ...
    PlanningDistOpt(x0,xF,m,N,Ts,carShape,stateWS,controlWS,XYbounds,nOb,vOb,A,b)

    % algorithm parameters
     
    % desired safety distance
    dmin = 0.5;        % Smaller than the value in Warm Start

    % regularization parameter to improve numerical stability
    reg = 1e-4;

    % refined horizon
    K = m*N;

    % interpolate warm start data
    STATES_INIT = zeros(m*N+1,7);
    STATES_INIT(:,1) = 0:1:m*N;
    for i = 2:7
        STATES_INIT(:,i) = interp1(stateWS(:,1),stateWS(:,i),0:1/m:N,'poly');
    end

    CONTROLS_INIT = zeros(m*N+1,27);
    CONTROLS_INIT(:,1) = 0:1:m*N;
    for i = 2:27
        CONTROLS_INIT(:,i) = interp1(controlWS(:,1),controlWS(:,i),0:1/m:N,'poly');
    end
    
    BEGIN_ACADO;  

        % set ACADO
        acadoSet('problemname', 'PlanningDistOpt');     % Set your problemname. If you 
                                                        % skip this, all files will
                                                        % be named "myAcadoProblem"  
        % defining optimization variables                         
        DifferentialState x;                            % DifferentialState
        DifferentialState y;    
        DifferentialState vx;    
        DifferentialState vy;
        DifferentialState timeScale;
        DifferentialState L;

        Control ax;                                  % Control
        Control ay;    
        Control la1;                                 % dual multiplier associated with obstacleShape
        Control la2;
        Control la3;
        Control la4;
        Control la5;
        Control la6;
        Control la7;
        Control la8;
        Control la9;
        Control la10;
        Control la11;
        Control la12;
        Control n1;
        Control n2;
        Control n3;
        Control n4;
        Control n5;
        Control n6;
        Control n7;
        Control n8;
        Control n9;
        Control n10;
        Control n11;
        Control n12;
        
        f = acado.DiscretizedDifferentialEquation(Ts);
        
        % cost function

        % (min time)+
        % (min states/control inputs)+      
        % (multiplier penalty)
        Q = blkdiag(0,1,1,1);
        R = blkdiag(1,1);
        f.add( next(L) == 0.5*timeScale + 1*timeScale^2 +...
            [x; y; vx; vy]'*Q*[x; y; vx; vy] + [ax; ay]'*R*[ax; ay]+...
            reg*la1^2+reg*la2^2+reg*la3^2+reg*la4^2+reg*la5^2+reg*la6^2+reg*la7^2 +...
            reg*la8^2+reg*la9^2+reg*la10^2+reg*la11^2+reg*la12^2 +...
            reg*n1^2+reg*n2^2+reg*n3^2+reg*n4^2+reg*n5^2+reg*n6^2+reg*n7^2 +...
            reg*n8^2+reg*n9^2+reg*n10^2+reg*n11^2+reg*n12^2);
        
        % dynamics of the car

        % - Double Integrator with forward Euler integration
        % States:[x,y,vx,vy]  Control inputs:[ax ay]
        % - sampling time scaling, is identical over the horizon
        f.add(next(x)  == x + timeScale*Ts*vx);
        f.add(next(y)  == y + timeScale*Ts*vy);
        f.add(next(vx) == vx + timeScale*Ts*ax);
        f.add(next(vy) == vy + timeScale*Ts*ay);
        f.add(next(timeScale) == timeScale);
        
        % Optimal Control Problem
        ocp = acado.OCP(0.0, K, K);                     % Set up the Optimal Control Problem (OCP)
                                                        % Start at 0,
                                                        % control in K intervals up to N
        ocp.minimizeMayerTerm( L );                     % Minimize this Mayer Term
        ocp.subjectTo( f );                             % Your OCP is always subject to your 
                                                        % differential equation
        
        % bounds on states and inputs
        ocp.subjectTo( XYbounds(1)+carShape(1) <= x <= XYbounds(2)-carShape(1)  );
    	ocp.subjectTo( XYbounds(3) <= y <= XYbounds(4)  );
        ocp.subjectTo( -2.0 <= ax <= 2.0  );
        ocp.subjectTo( -2.0 <= ay <= 2.0  );
        ocp.subjectTo( 0.5 <= timeScale <= 2.5  );
        ocp.subjectTo( la1 >= 0.0); ocp.subjectTo( la2 >= 0.0); ocp.subjectTo( la3 >= 0.0);
        ocp.subjectTo( la4 >= 0.0); ocp.subjectTo( la5 >= 0.0); ocp.subjectTo( la6 >= 0.0);
        ocp.subjectTo( la7 >= 0.0); ocp.subjectTo( la8 >= 0.0); ocp.subjectTo( la9 >= 0.0);
        ocp.subjectTo( la10 >= 0.0); ocp.subjectTo( la11 >= 0.0); ocp.subjectTo( la12 >= 0.0);
        ocp.subjectTo( n1 >= 0.0); ocp.subjectTo( n2 >= 0.0); ocp.subjectTo( n3 >= 0.0);
        ocp.subjectTo( n4 >= 0.0); ocp.subjectTo( n5 >= 0.0); ocp.subjectTo( n6 >= 0.0);
        ocp.subjectTo( n7 >= 0.0); ocp.subjectTo( n8 >= 0.0); ocp.subjectTo( n9 >= 0.0);
        ocp.subjectTo( n10 >= 0.0); ocp.subjectTo( n11 >= 0.0); ocp.subjectTo( n12 >= 0.0);

        % start and finish point
        ocp.subjectTo( 'AT_START', L  == 0.0    );
        ocp.subjectTo( 'AT_START', x  == x0(1)  );
        ocp.subjectTo( 'AT_START', y  == x0(2)  );
        ocp.subjectTo( 'AT_START', vx == x0(3)  );
        ocp.subjectTo( 'AT_START', vy == x0(4)  );
        ocp.subjectTo( 'AT_END',   x  == xF(1) );   % Consider replacing by a set
        ocp.subjectTo( 'AT_END',   y  == xF(2) );
        
        % obstacle avoidance constraints    
        A1 = A(1:4,:);	% extract obstacle matrix associated with j-th obstacle
        b1 = b(1:4);	% extract obstacle matrix associated with j-th obstacle
        A2 = A(5:8,:);
        b2 = b(5:8);
        A3 = A(9:12,:);
        b3 = b(9:12);
        
        g = [carShape(2),carShape(1),carShape(2),carShape(1)];
        
        %% norm(A'*lambda) <= 1    *** REMOVED SQRT ***
        ocp.subjectTo(  (A1'*[la1;la2;la3;la4])'*(A1'*[la1;la2;la3;la4]) <= 1 );
        ocp.subjectTo(  (A2'*[la5;la6;la7;la8])'*(A2'*[la5;la6;la7;la8]) <= 1 );
        ocp.subjectTo(  (A3'*[la9;la10;la11;la12])'*(A3'*[la9;la10;la11;la12]) <= 1 );

        %% (A*p - b)*lambda > 0
        ocp.subjectTo( -g*[n1;n2;n3;n4]+(A1*[x;y]-b1)'*[la1;la2;la3;la4] >= dmin );
        ocp.subjectTo( -g*[n5;n6;n7;n8]+(A2*[x;y]-b2)'*[la5;la6;la7;la8] >= dmin );
        ocp.subjectTo( -g*[n9;n10;n11;n12]+(A3*[x;y]-b3)'*[la9;la10;la11;la12] >= dmin );
        
        % Set up the optimization algorithm
        algo = acado.OptimizationAlgorithm( ocp );
        algo.set( 'MAX_NUM_ITERATIONS', 1500 );
        algo.set( 'KKT_TOLERANCE', 1e-3 );
        algo.set( 'INTEGRATOR_TYPE', 'INT_RK45');
    %     algo.set( 'INTEGRATOR_TOLERANCE',   1e-3);  
    %     algo.set( 'ABSOLUTE_TOLERANCE',   1e-3   );
    
        % set initial guess
        algo.initializeDifferentialStates(STATES_INIT);
        algo.initializeControls(CONTROLS_INIT);
                        
    END_ACADO;                                          % Always end with "END_ACADO".
                                                        % This will generate a file problemname_ACADO.m. 
                                                        % Run this file to get your results. 
                                                        % You can run the file problemname_ACADO.m as
                                                        % many times as you want without having to compile again.
    tic()
    out = PlanningDistOpt_RUN();                           % Run the test. The name of the RUN file
                                                        % is problemname_RUN, so in
                                                        % this case active_damping_RUN
    solverTime = toc();

	% return values
    fprintf('Elapsed time: %f\n',solverTime);
    stateSol = out.STATES;
    controlSol = out.CONTROLS;
    outSol = out;
end


%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017
