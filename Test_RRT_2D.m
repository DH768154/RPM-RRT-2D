clc;clear;close all
addpath('myfunc','-begin')
%nsamples = 10000;

ag2 = [-40,-30,-30,-30,-30]/180*pi;
ag1 = zeros(1,5);
%ag2 = [40,30,20,10,5]/180*pi;

w = [1,1,0.8,0.6,0.6]; % Weight
w = w/sum(w);
minstep = [1,1,1,1,1]*3/180*pi;
slicenum0 = 2;

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

%%

max_iter = 10000;

%% initialize tree

stepsize = 60/180*pi;
%stepsize = norm([1,1,1,1,1]*20/180*pi);

dag = ag_range(2,:) - ag_range(1,:);

vtx = NaN(max_iter+1,n);
vtx(1,:) = ag1;
edges = NaN(2,max_iter);
D = NaN(1,max_iter);
count = 1;
r = 20/180*pi;

ag_bound = [];

tic
for i = 1:max_iter
    rand_state = rand(1,n).*dag + ag_range(1,:);

    idx = knnsearch(vtx(1:count,:).*w,rand_state.*w);
    nn = vtx(idx,:);
    dir = rand_state-nn;

    %stepsize = (rand(1)-0.5)*180/pi;
    step1 = dir/max(abs(dir))*stepsize;
    if norm(step1)>norm(dir)
        step1 = dir;
    end
    %step1 = dir/norm(dir)*stepsize;
    vtx_add = nn + step1;
    vtx_add = bound2range(vtx_add,ag_range);

    links = Link5R_2d(L,vtx_add);

    if iscollision_Link5R_2d(links,O)
       
%         idx = knnsearch(vtx(1:count,:).*w,vtx_add.*w);
%         ag01 = vtx_add;
%         ag02 = vtx(idx,:);
%         agm = (ag01+ag02)/2;
% 
%         while any(abs(ag02-ag01)>minstep)
% 
%             links = Link5R_2d(L,agm);
%             if ~iscollision_Link5R_2d(links,O)
%                 count = count+1;
%                 vtx(count,:) = agm;
%                 edges(:,count-1) = [idx;count];
%                 D(count-1) = sum((vtx_add-ag02).^2.*w);
%                 break;
%             else
%                 ag01 = agm;
%             end
%             agm = (ag01+ag02)/2;
%         end
%%
    else
        if validpath_Link5R_2d(L,O,vtx_add,nn,slicenum0,minstep)

            %%
            count = count+1;
            vtx(count,:) = vtx_add;
            edges(:,count-1) = [idx;count];
            D(count-1) = sum((vtx_add-nn).^2.*w)^0.5;

            dist2goal = sum((vtx_add-ag2).^2.*w)^0.5;
            if dist2goal<=(30/180*pi)
                if validpath_Link5R_2d(L,O,vtx_add,ag2,slicenum0,minstep)
                    count = count+1;
                    vtx(count,:) = ag2;
                    edges(:,count-1) = [count-1;count];
                    D(count-1) = dist2goal;
                    break
                end
            end
        end
    end

end
iter = i
vtx = vtx(1:count,:);
edges = edges(:,1:count-1);
D = D(:,1:count-1);

if iter==max_iter 
    ind = knnsearch(vtx.*w,ag2.*w,'K',5);
    for j = 1:length(ind)
        if validpath_Link5R_2d(L,O,vtx(ind(j),:),ag2,slicenum0,minstep)
            vtx = [vtx;ag2];
            edges = [edges,[ind(j);count+1]];
            D = [D,sum(vtx(ind(j)-s2).^2.*w)^0.5]; % Dist for Last one  
            break
        else
            error('Not Reach')
        end
    end
end


toc
%%
G0 = graph(edges(1,:),edges(2,:),D);
A0 = full(adjacency(G0,'weighted'));
A0 = SimpleAdjacency(A0);
[route0, dist0, c_n0] = Dijkstra(A0, 1, size(vtx,1));

vtx_main = vtx(route0,:);

%%
tic
k = 20;
ind = knnsearch(vtx.*w,vtx_main.*w,'K',k);
ind = ind(:,2:end);
edges_ext = NaN(2,(count-1)*k+1);
edges_ext(:,1:count-1) = edges;
D_ext = NaN(1,(count-1)*k+1);
D_ext(:,1:count-1) = D;
% edge = [edge,NaN(2,k*(count-1))];
% D = [D,NaN((count-1)*k)];
count_ext = count;


for i = 1:size(vtx_main,1)
    indchecked1 = edges_ext(2,edges_ext(1,:)==route0(i));
    indchecked2 = edges_ext(1,edges_ext(2,:)==route0(i));
    indcheck = ind(i,~ismember(ind(i,:),[indchecked1,indchecked2]));
    vtxcheck = vtx(indcheck,:);
    for j = 1:length(indcheck)
        if validpath_Link5R_2d(L,O,vtxcheck(j,:),vtx_main(i,:),slicenum0,minstep)
            count_ext = count_ext+1;
            edges_ext(:,count_ext-1) = [route0(i);indcheck(j)];
            D_ext(count_ext-1) = sum((vtx_main(i,:)-vtxcheck(j,:)).^2.*w)^0.5;
        end
    end
end
D_ext = D_ext(1:count_ext-1);
edges_ext = edges_ext(:,1:count_ext-1);
toc


%%



%%
G = graph(edges_ext(1,:),edges_ext(2,:),D_ext);
A = full(adjacency(G,'weighted'));
A = SimpleAdjacency(A);
[route, dist, c_n] = Dijkstra(A, 1, size(vtx,1));
figure;
p = plot(G); hold on
highlight(p,route0,'EdgeColor','black','LineWidth',2);
highlight(p,route,'EdgeColor','red','LineWidth',2);
highlight(p,1,'Marker','o','NodeColor','red','MarkerSize',5)
highlight(p,size(vtx,1),'Marker','o','NodeColor','blue','MarkerSize',5)

%% Plot path points (Extended Graph)

ags = vtx(route(1),:);
maxstep = 1/180*pi;

for i = 1:length(route)-1
    ag01 = vtx(route(i),:);
    ag02 = vtx(route(i+1),:);
    cag = agsteps(ag01,ag02,maxstep);
    ags = cat(1,ags,cag(2:end,:));
end

links_r = Link5R_2d(L,ags);

%% Plot path points (Original Graph)

ags = vtx(route0(1),:);
maxstep = 1/180*pi;

for i = 1:length(route0)-1
    ag01 = vtx(route0(i),:);
    ag02 = vtx(route0(i+1),:);
    cag = agsteps(ag01,ag02,maxstep);
    ags = cat(1,ags,cag(2:end,:));
end

links_r0 = Link5R_2d(L,ags);


%% Simulation

f = figure;
title('RRT (Extended Edge)')
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

%% Simulation

f = figure;
title('RRT (Original)')
for i = 1:length(O)
    patch(Op{i}(1,:),Op{i}(2,:),'.-y','LineWidth',1); hold on
end
plot(links01(1,:,1),links01(2,:,1),'.-b','MarkerSize',5,'LineWidth',1); hold on
plot(links01(1,:,2),links01(2,:,2),'.-r','MarkerSize',5,'LineWidth',1); hold on
p1 = plot(links_r0(1,:,1),links_r0(2,:,1),'.-k','MarkerSize',10,'LineWidth',1); hold on
grid on; axis equal
xlim([-2.2,5.5]); ylim([-5,2.2])
set(f,'Units','normalized','Position',[0.2,0.2,0.4,0.6])
pause(1)

if isempty(route)
    return
end
for i = 1:size(links_r0,3)
    p1.XData = links_r0(1,:,i);
    p1.YData = links_r0(2,:,i);
    drawnow
    %pause(0.1)
end
for i = 1:5
    p1.XData = links_r0(1,:,size(links_r0,3));
    p1.YData = links_r0(2,:,size(links_r0,3));
    drawnow    
end
%%

% f = figure;
% for i = 1:length(O)
%     patch(Op{i}(1,:),Op{i}(2,:),'.-y','LineWidth',1); hold on
% end
% plot(links01(1,:,1),links01(2,:,1),'.-b','MarkerSize',5,'LineWidth',1); hold on
% plot(links01(1,:,2),links01(2,:,2),'.-r','MarkerSize',5,'LineWidth',1); hold on
% %p1 = plot(links_r(1,:,1),links_r(2,:,1),'.-k','MarkerSize',5,'LineWidth',1); hold on
% grid on; axis equal
% xlim([-4,5]); ylim([-5,5])
% set(f,'Units','normalized','Position',[0.2,0.2,0.6,0.6])
% 
% ind = knnsearch(vtx,ag2,'K',20);
% 
% ags = vtx(ind,:);
% links_r = Link5R_2d(L,ags);
% 
% for i = 1:20
%     plot(links_r(1,:,i),links_r(2,:,i),'.-k','MarkerSize',5,'LineWidth',1); hold on
% end
% 













