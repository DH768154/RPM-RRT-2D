function shape2d = randShape2d(n_range,eps_range,r,r_ratio_range,...
    ag_range,trans_range,varargin)
% random shape in 2d
% eps<2, convex



if nargin >= 7
    if strcmpi(varargin{1},'fix')
        t1 = 0;
        ag = ag_range;
        trans = trans_range;
        eps = eps_range;
        r_ratio = r_ratio_range;
        n_vetx = n_range+1; % at least 4 pts from 0 to 2pi
    end
else
    t1 = (rand(1)-0.5)*pi;
    ag = (rand(1)-0.5)*ag_range;
    trans = (rand(2,1)-0.5)*trans_range;
    eps = rand(1)*eps_range;

    r_ratio = rand(1)*r_ratio_range;
    r_ratio = max([r_ratio,1/r_ratio_range]);

    n_vetx = ceil(rand(1)*n_range)+3; % at least 4 pts from 0 to 2pi
end



theta = linspace(t1,t1+2*pi,n_vetx); 

a = [r;r*r_ratio];
x1 = a(1) * abs(cos(theta)).^eps.*sign(cos(theta));
x2 = a(2) * abs(sin(theta)).^eps.*sign(sin(theta));

O = [x1(1:end-1);x2(1:end-1)];

shape2d = [cos(ag),-sin(ag);sin(ag),cos(ag)] * O + trans; 
%shape2d = shape2d(:,randperm(n_vetx-1));
end