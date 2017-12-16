function lOb = getlOb(ob1,ob2,ob3)

    lOb = {};
    for i = 1:length(ob1)
        lOb{1,i} = ob1(i,:)';
    end
    lOb{1,length(ob1)+1} = ob1(1,:)';
    for i = 1:length(ob2)
        lOb{2,i} = ob2(i,:)';
    end
    lOb{2,length(ob1)+1} = ob2(1,:)';
    for i = 1:length(ob3)
        lOb{3,i} = ob3(i,:)';
    end
    lOb{3,length(ob1)+1} = ob3(1,:)';
    
%     %     	[ 	[[obst1_x1;obst1_y1],[obst1_x2;obst1_y2],[obst1_x3;obst1_y4],...,[obst1_x1;obst1_y1]]    , 		[[obst2_x1;obst2_y1],[obst2_x2;obst2_y2],[obst2_x3;obst2_y4],...,[obst2_x1;obst2_y1]]     ,     ...   ]	
%     lOb = {    [-20;5], [-5;10], [-5;3], [-15;3], [-20;5]; 
% 	 	   [10;10], [20;5], [20;-5], [1.3;-5], [10;10] ;
% 		   [0;18], [20;18], [20;13], [0,13], [0;18] 		};		%vetices given in CLOCK-WISE direction

end