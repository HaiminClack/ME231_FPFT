function plotTraj(xp,N,nOb,vOb,lOb,XYbounds,disp_title,plotNumb)
	
    figure(plotNumb)

    x0 = xp(1,1:2);
    xF = xp(end,1:2);
    
    plot(x0(1),x0(2),'.k','MarkerSize',30)
    hold on
    plot(xF(1),xF(2),'.g','MarkerSize',30)
    hold on

    % Plot Obstacle
    for j = 1 : nOb
        for k = 1 : vOb(j)
            line([lOb{j,k}(1),lOb{j,k+1}(1)] , [lOb{j,k}(2),lOb{j,k+1}(2)],'color','k','LineStyle','-');
            hold on
        end
    end
    
    % Plot Sensing Range
    xl = XYbounds(1); xu = XYbounds(2);
    yl = XYbounds(3); yu = XYbounds(4);
    x=[xl xu xu xl xl xl xl xl xl xl xl xu xu xu xu xu xu xu xl];
    y=[yl yl yl yl yl yl yu yu yl yu yu yu yu yl yl yu yu yu yu];
    
    for i = 1:1:N+1
        % Car shape
        cw = 1.6/2;
        cl = 3.0/2;
        carxl = xp(i,1)-cw; carxu = xp(i,1)+cw;
        caryl = xp(i,2)-cl; caryu = xp(i,2)+cl;
        carx=[carxl carxu carxu carxl carxl carxl carxl carxl carxl carxl carxl carxu carxu carxu carxu carxu carxu carxu carxl];
        cary=[caryl caryl caryl caryl caryl caryl caryu caryu caryl caryu caryu caryu caryu caryl caryl caryu caryu caryu caryu];
        plot(carx,cary,'g');
        hold on        
    end
    
    plot(xp(1:N+1,1),xp(1:N+1,2),'MarkerSize',50);
    hold on
    
    plot(x,y,'--');
    xlabel('x');
    ylabel('y');
    axis equal
%     xlim([xl,xu])
%     ylim([yl,yu])
    title(disp_title)
end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017