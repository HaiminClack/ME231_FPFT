function [car1, car2, car3, car4] = carBox(x0,phi,w,l)
    car1 = x0(1:2) + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0(1:2) + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0(1:2) - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0(1:2) - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)],'r')
end
