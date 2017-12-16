function plotObs(nOb,vOb,lOb,x0,xF,XYbounds)

    plot(x0(1),x0(2),'.k','MarkerSize',30)
    hold on
    plot(xF(1),xF(2),'.g','MarkerSize',30)
    hold on

    % Plot Obstacle
    for j = 1 : nOb
        for k = 1 : vOb(j)
            line([lOb{j,k}(1),lOb{j,k+1}(1)] , [lOb{j,k}(2),lOb{j,k+1}(2)],'color','k','LineStyle','-');
        end
    end
    
    % Plot Sensing Range
    xl = XYbounds(1); xu = XYbounds(2);
    yl = XYbounds(3); yu = XYbounds(4);
    x=[xl xu xu xl xl xl xl xl xl xl xl xu xu xu xu xu xu xu xl];
    y=[yl yl yl yl yl yl yu yu yl yu yu yu yu yl yl yu yu yu yu];
    
    plot(x,y,'--');
    xlabel('x');
    ylabel('y');
    
    axis equal
    xlim([xl,xu])
    ylim([yl,yu])
end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017