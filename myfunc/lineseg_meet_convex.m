function isintersect = lineseg_meet_convex(O,L)

Op = ptsloop(O);
AB = diff(Op,1,2);
AP1 = L(:,1,:)-O;
AP2 = L(:,2,:)-O;
P1P2 = L(:,2,:)-L(:,1,:);
P1B = Op(:,2:end)-L(:,1,:);

ABxAP1 = cross2d(AB,AP1);
ABxAP2 = cross2d(AB,AP2);
P1P2xP1A= cross2d(P1P2,-AP1);
P1P2xP1B= cross2d(P1P2,P1B);

isintersect = any(all([ABxAP1.*ABxAP2;P1P2xP1A.*P1P2xP1B]<=0));

if size(L,3) == 1
    isinconvex = all(inconvex(O,L));
    isintersect = isintersect || isinconvex;

else
    isinconvex = all(reshape(inconvex(O,reshape(L,2,[],1)),1,2,[]));
    isintersect = any([isinconvex,isintersect]);
end


end