function d = point2seg(pose,wall,sensorOrigin)

        wallsp_x = wall(1);
        wallsp_y = wall(2);
        wallep_x = wall(3);
        wallep_y = wall(4);
        sensor_xy = robot2global(pose,sensorOrigin);
     
        % Using Heron's formula to calculate the minimum distance from sensor to
        % the line segment
        % sensor to x1,y1
        d1 = sqrt((sensor_xy(1)-wallsp_x)*(sensor_xy(1)-wallsp_x)+(sensor_xy(2)-wallsp_y)*(sensor_xy(2)-wallsp_y));                                                                
        % sensor to x2,y2
        d2 = sqrt((sensor_xy(1)-wallep_x)*(sensor_xy(1)-wallep_x)+(sensor_xy(2)-wallep_y)*(sensor_xy(2)-wallep_y)); 
        % x1,y1 to x2,y2
        d3 = sqrt((wallep_x-wallsp_x)*(wallep_x-wallsp_x)+(wallep_y-wallsp_y)*(wallep_y-wallsp_y)); 
        
        if (d1*d1) >= (d2*d2+d3*d3)
            d = d2;      
        elseif (d2*d2) >=(d1*d1+d3*d3)
            d = d1;  
        else
            l = (d1+d2+d3)/2;
            s = sqrt(l*(l-d1)*(l-d2)*(l-d3));
            d = 2*s/d3;
        end
        
end