function lObNew = scaleObs(nOb,vOb,lOb,scaleFactor)
    
    lObNew = lOb;
    
    % For each obstacle
    for j = 1 : nOb
        
        % Original obstacle
        V = [];
        for k = 1 : vOb(j)
            V = [V ; [lOb{j,k}(1), lOb{j,k}(2)] ];
        end
        poly = Polyhedron('V',V);
        cc = chebyCenter(poly);
        cc = cc.x;
        
        % Construct scaled obstacle
        VNew = [];
        for v = poly.V'
            scaledVec = (cc - v)*scaleFactor;
            VNew = [VNew; (cc-scaledVec)'];
        end
        polyNew = Polyhedron('V',VNew);
        
        % Save scaled obstacle
        lObNew{j,end} = polyNew.V(1,:)';
        for k = 1 : vOb(j)
            lObNew{j,k} = polyNew.V(k,:)';
        end
        
    end
    
end

%% Attribution
% Xiangyu Gao, Haimin Hu, Zichen Xiao, Chi Zhang, and Kaixin Zheng, ME 231A Project, UC Berkeley, Date: Dec.15th, 2017