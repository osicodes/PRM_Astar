function y = via_obstacle(map,Segments,Node,point)
    y = 0;
    dx = (point(1) - Node(1))/Segments;
    dy = (point(2) - Node(2))/Segments;
    x = 0;
    for n = 1:Segments - 1
        xn = Node(1) + n * dx;
        yn = Node(2) + n * dy;
        
        if (map(round(xn),round(yn)) == 1)
            x = x+1; 
        end
    end
    if(x>0)
        y=1;
    end

end