function issect = circle_lineseg(centerx,centery,radius,x1,y1,x2,y2)

pt1 = [x1 y1];
pt2 = [x2 y2];
diffvec = pt2-pt1;
slope = diffvec(2)/diffvec(1);
intercpt = y1 - x1*slope;
[xout,yout] = linecirc(slope,intercpt,centerx,centery,radius);

g = length(xout);
%compvec = zeros(g,1);
if ~isnan(xout(1))
    for i=1:g
        nowx = xout(i);
        nowy = yout(i);
        if ((nowx>=x1 && nowy>=y1) && (nowx<=x2 && nowy<=y2))...
                || ((nowx<=x1 && nowy<=y1) && (nowx>=x2 && nowy>=y2))...
                || ((nowx>=x1 && nowy<=y1) && (nowx<=x2 && nowy>=y2))...
                || ((nowx<=x1 && nowy>=y1) && (nowx>=x2 && nowy<=y2))
            %compvec(i) = 1;
            issect = true;
            break
        else
            %compvec(i) = 0;
            issect = false;
        end
    end
else
    if x1~=x2
        issect = false;
    else
        if (abs(centerx-x1)<=radius && centery>=min(y1,y2)...
                && centery<=max(y1,y2))...
                || (norm([centerx centery] - [x1 y1])<=radius)...
                || (norm([centerx centery] - [x2 y2])<=radius)
            issect = true;
        else
            issect = false;
        end
    end
end

end