function links = Link5R_2d(L,ag)
%Link5R_2d
%    LINKS = Link5R_2d(L,ag)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    27-Mar-2024 13:55:38
%    Modified for vectorize calculation

if size(ag,1)>1
    ag = reshape(ag',1,size(ag,2),[]);
end

t2 = cos(ag(1,1,:));
t3 = sin(ag(1,1,:));
t4 = ag(1,1,:)+ag(1,2,:);
t5 = L(1).*t2;
t6 = L(1).*t3;
t7 = ag(1,3,:)+t4;
t8 = cos(t4);
t9 = sin(t4);
t10 = ag(1,4,:)+t7;
t11 = cos(t7);
t12 = sin(t7);
t13 = L(2).*t8;
t14 = L(2).*t9;
t15 = ag(1,5,:)+t10;
t16 = cos(t10);
t17 = sin(t10);
t18 = L(3).*t11;
t19 = L(3).*t12;
t20 = L(4).*t17;
t21 = L(4).*t16;
links = reshape([zeros(1,2,size(ag,3)),t5,t6,t5+t13,t6+t14,t5+t13+t18,t6+t14+t19,...
    t5+t13+t18+t21,t6+t14+t19+t20,t5+t13+t18+t21+L(4).*cos(t15),...
    t6+t14+t19+t20+L(4).*sin(t15)],[2,6,size(ag,3)]);
