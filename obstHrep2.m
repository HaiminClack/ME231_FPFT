function [A_all, b_all] = obstHrep2(nOb, vOb, lOb)

	% do simple checks
	if nOb ~= length(lOb(:,1))
		fprintf('ERROR in number of obstacles')
	end

	% these matrices contain the H-rep
	A_all = zeros(sum(vOb),2);
	b_all = zeros(sum(vOb),1);

	% counter for lazy people
	lazyCounter = 1;
    
    for j = 1 : nOb	% building H-rep for each polytope of obstacle
		
        % Original obstacle
        V = [];
        for k = 1 : vOb(j)
            V = [V ; [lOb{j,k}(1), lOb{j,k}(2)] ];
        end
        poly = Polyhedron('V',V);
        
		% store everything
		A_all(lazyCounter : lazyCounter+vOb(j)-1,:) = poly.A;
		b_all(lazyCounter : lazyCounter+vOb(j)-1) = poly.b;
	
		% update counter
		lazyCounter = lazyCounter + vOb(j);
    end

end
