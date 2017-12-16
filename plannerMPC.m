function stateOpt = plannerMPC(XYbounds,x0,xF,nOb,vOb,lOb)

    %%%% problem parameters %%%%
    % horizon
    N  = 5;

    % nominal sampling time
    Ts = 0.5;

    % Car width and length
    cw = 1.6/2;
    cl = 3.0/2;
    carShape = [cw,cl];
    
    fprintf('**** START ****\n')

    %%%%%% Warm Start: Planning without Obstacles %%%%%%
    fprintf('**** Warm Start: Planning without Obstacles ****\n')
    [xWS, scaleTimeWS, stateWS, controlWS, timeWS, status] = WarmStart(N,Ts,x0,xF,XYbounds);
%     plotTraj(xWS,N,nOb,vOb,lOb,XYbounds,'Trajectory without obstacles',1);

    %%%%%% obtain H-rep of all obstacles %%%%%%
    [AOb, bOb] = obstHrep2(nOb, vOb, lOb);

    %%%%% Iterative Warm Start  %%%%%%
    TotalTime = timeWS;
    count = 1;
    for scaleFactor = 1.0:0.2:1.0
        lObNew = scaleObs(nOb,vOb,lOb,scaleFactor);
        [AObNew, bObNew] = obstHrep2(nOb, vOb, lObNew);

        fprintf('**** Iterative Warm Start at Scaling Factor %f ****\n',scaleFactor)
        [stateOut, controlOut, solverTime, out] = ...
            PlanningDist(x0,xF,N,Ts,carShape,stateWS,controlWS,XYbounds,nOb,vOb,AObNew,bObNew);
        if out.CONVERGENCE_ACHIEVED==1
            fprintf('**** Problem solved SUCCESSFULLY ****\n')
            stateWS = stateOut;
            controlWS = controlOut;
%             plotTraj(stateOut(:,2:3),N,nOb,vOb,lObNew,XYbounds,'Iterative Warm Start',count)
        else
%             plotTraj(stateOut(:,2:3),N,nOb,vOb,lObNew,XYbounds,'Iterative Warm Start - Infeasible Solution',count)
            fprintf('**** WARNING: Problem could not be solved ****\n')
        end
        TotalTime = TotalTime + solverTime;
    %     count = count + 1;
    end

    % If NO final optimization
    stateOpt = stateOut;
    plotTraj(stateOpt(:,2:3),N,nOb,vOb,lOb,XYbounds,'',count)

    %%%%% Final Optimization  %%%%%%
%     count = 3;
%     fprintf('**** Optimizing the path ****\n')
%     m = 2;  % horizon multiplier
%     [stateOpt, controlOpt, solverTime, outOpt] = ...
%         PlanningDistOpt(x0,xF,m,N,Ts,carShape,stateWS,controlWS,XYbounds,nOb,vOb,AOb,bOb);
%     if outOpt.CONVERGENCE_ACHIEVED==1
%         fprintf('**** Problem solved SUCCESSFULLY ****\n')
%         plotTraj(stateOpt(:,2:3),m*N,nOb,vOb,lOb,XYbounds,'Optimized Path',count)
%     else
%         plotTraj(stateOpt(:,2:3),m*N,nOb,vOb,lOb,XYbounds,'Optimized Path - Potentially Infeasible Solution',count)
%         fprintf('**** WARNING: Problem could not be solved ****\n')
%     end
%     TotalTime = TotalTime + solverTime;

    fprintf('**** Total solver time: %f ****\n',TotalTime);

    fprintf('**** DONE ****\n')
end


%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017