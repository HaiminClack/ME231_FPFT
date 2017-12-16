function carPosition = getCarVertice(x, y, cw, cl, isVertical)

    if isVertical
        carPosition = [x-cw, y+cl; x+cw, y+cl; x+cw, y-cl; x-cw, y-cl];
    else
        carPosition = [x-cl, y+cw; x+cl, y+cw; x+cl, y-cw; x-cl, y-c2];
    end

end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017