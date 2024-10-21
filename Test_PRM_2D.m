clc;clear;close all
addpath('myfunc','-begin')
nsamples = 10000;

ag2 = [-40,-30,-30,-30,-30]/180*pi;
ag1 = zeros(1,5);
%ag2 = [40,30,20,10,5]/180*pi;

w = [1,1,0.8,0.6,0.6]; % Weight
w = w/sum(w);
minstep = [1,1,1,1,1]*3/180*pi;
slicenum0 = 2;

calcmap = true;

%% Define th Robot

L = [1,1,1,1,1];
n = length(L);
links0 = [0,L;zeros(1,n+1)];

ag_range = [-1,1;-1,1;-1,1;-1,1;-1,1]'.*120/180*pi;

%% Define Obstacles

O = cell(4,1);
O{1} = randShape2d(10,1.7,0.9,1.3,1,[3;2]-0.8,'fix');
O{2} = randShape2d(8,1,0.6,1.6,0.2,[2.5;-1.5]+0.3,'fix');
O{3} = randShape2d(8,1,0.6,2,-0.2,[2;-3.5],'fix');
O{4} = [-2.2,-2,-2,-2.2;-5,-5,2,2];
Op = ptsloop(O);

%% Check Start / Stop State Valid

startstop = [ag1;ag2];
links01 = Link5R_2d(L,startstop);
isintersect = iscollision_Link5R_2d(links01,O);
if any(isintersect)
    error('Statrt / Stop Pose Collision')
end

%% Samples
if ~calcmap
    load('roadmap2d.mat');
else
    dag = ag_range(2,:) - ag_range(1,:);
    ag = rand(nsamples,n).*dag + ag_range(1,:);

    %% FWD Kinematic for Robot

    links = Link5R_2d(L,ag);

    %% Check Valid States

    tic
    isintersect = iscollision_Link5R_2d(links,O);
    ag_valid = ag(~isintersect,:);
    links_valid = links(:,:,~isintersect);
    t(1) = toc;

    %% knn, construct graph

    tic

    r = sqrt(sum((45/180*pi.*w*n).^2)); % Search range
    A = knngraph(ag_valid,10,r,w);
    t(2) = toc;

    %% Check Valid Path

    tic
    ind = find(tril(A)~=0);
    [row,col] = ind2sub(size(A),ind);

    for i = 1:length(ind)
        validpath = validpath_Link5R_2d(L,O,...
            ag_valid(row(i),:),ag_valid(col(i),:),slicenum0,minstep);
        if ~validpath
            A(row(i),col(i)) = 0;
            A(col(i),row(i)) = 0;
        end
        if mod(i,1000) == 0
            fprintf('Check Path: %.0f / %.0f\n',i,length(ind))
        end
    end
    
    t(3) = toc;

    save('roadmap2d.mat','A','ag_valid')
end

%% Start and Stop States to Sampled States

tic
[idx, D] = knnsearch(ag_valid.*w, startstop.*w,'K',10+1);
indstart = false(1,size(idx,2));
indstop = false(1,size(idx,2));

for i = 1:size(idx,2)
    indstart(i) = validpath_Link5R_2d(L,O,...
        startstop(1,:),ag_valid(idx(1,i),:),slicenum0,minstep);
    indstop(i)  = validpath_Link5R_2d(L,O,...
        startstop(2,:),ag_valid(idx(2,i),:),slicenum0,minstep);
end


%% Update graph, add start stop

A_start_stop = zeros(2,size(ag_valid,1));
A_start_stop(1,idx(1,indstart)) = D(1,indstart);
A_start_stop(2,idx(2,indstop)) = D(2,indstop);

A_all = zeros(size(A)+2);
A_all(3:end,3:end) = A;
A_all(1:2,3:end) = A_start_stop;
A_all(3:end,1:2) = A_start_stop';

ag_all = [startstop;ag_valid];


%% Shortest Path

As = SimpleAdjacency(A_all);
[route, distant] = Dijkstra(As, 1, 2);
if isempty(route)
    figure
    p = plot(graph(A_all));
    highlight(p,1,'Marker','o','MarkerSize',5,'NodeColor','red')
    highlight(p,2,'Marker','o','MarkerSize',5,'NodeColor','b')
    return
end

t(4) = toc;

%% Plot path points

ags = ag_all(route(1),:);
maxstep = 1/180*pi;

for i = 1:length(route)-1
    ag01 = ag_all(route(i),:);
    ag02 = ag_all(route(i+1),:);
    cag = agsteps(ag01,ag02,maxstep);
    ags = cat(1,ags,cag(2:end,:));
end

links_r = Link5R_2d(L,ags);

%% Simulation

f = figure;
title('PRM')
for i = 1:length(O)
    patch(Op{i}(1,:),Op{i}(2,:),'.-y','LineWidth',1); hold on
end
plot(links01(1,:,1),links01(2,:,1),'.-b','MarkerSize',5,'LineWidth',1); hold on
plot(links01(1,:,2),links01(2,:,2),'.-r','MarkerSize',5,'LineWidth',1); hold on
p1 = plot(links_r(1,:,1),links_r(2,:,1),'.-k','MarkerSize',10,'LineWidth',1); hold on
grid on; axis equal
xlim([-2.2,5.5]); ylim([-5,2.2])
set(f,'Units','normalized','Position',[0.2,0.2,0.4,0.6])
pause(1)

if isempty(route)
    return
end
for i = 1:size(links_r,3)
    p1.XData = links_r(1,:,i);
    p1.YData = links_r(2,:,i);
    drawnow
    %pause(0.1)
end
for i = 1:5
    p1.XData = links_r(1,:,size(links_r,3));
    p1.YData = links_r(2,:,size(links_r,3));
    drawnow    
end

%% Plot Graph and Path

figure;
p = plot(graph(A_all));
highlight(p,1,'Marker','o','MarkerSize',5,'NodeColor','red')
highlight(p,2,'Marker','o','MarkerSize',5,'NodeColor','b')
highlight(p,route,'EdgeColor','red','LineWidth',1);
