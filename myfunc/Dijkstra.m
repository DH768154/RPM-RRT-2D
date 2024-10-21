function varargout = Dijkstra(A, n_init, n_goal)
% 
% Find Shortest Path using Dijkstra
%
% route = Dijkstra(A, n_init, n_goal);
% [route, distant] = Dijkstra(A, n_init, n_goal);
% [route, distant, c_n] = Dijkstra(A, n_init, n_goal);
%
% Input:
% A: Undircted, No self loop Adjacency Matrix, diag=0, no connection = inf
% n_init, n_goal: initial, goal nodes
%
% Output:
% route: shortest path
% distant: shortest distance
% c_n: Closed Nodes (Visited Nodes)
%
%%
if size(A,1)~=size(A,2)
    error('Adjacency Matrix Should be Square')
else
    N = size(A,1);
end

%% Initialize
dist = inf(N,1); % Distance from Initial Node
dist(n_init) = 0;
prev = NaN(N,1); % Parent Node
U = true(1,N); % Unvisited Node

%% Find nearest distance from n_init to all nodes
while U(n_goal)
    Cs = find(dist==min(dist(U)) & U'); % min distance in unvisited nodes
    C = Cs(randperm(length(Cs),1)); % Randomly Choose 1 C from Cs
    U(C) = false; % set node to visited
    ind_adj = find(A(C,:)~=0 & A(C,:)~=inf & U); % find neighbour
    alt = dist(C)+A(C,ind_adj)'; % find new distance
    is_update = alt<dist(ind_adj); % check if new distance is smaller
    ind_update = ind_adj(is_update); % index of distance need to be updated
    dist(ind_update) = alt(is_update); % update distance
    prev(ind_update) = C; % parents node
end

%% From parent node and distance, find the route
route = [n_goal;NaN(N,1)];

count = 1;
while ~ismember(n_init,route)
    if isnan(route(count))
        route = [];
        distant = inf;
        break % No Path Found, Return  inf for distance
    else
        route(count+1) = prev(route(count));
        count = count+1;
    end   
end

if isempty(route)
    fprintf('No Path Found between Node %.0f ~ %.0f\n',...
        n_init,n_goal)
else
    route = route(count:-1:1);
    distant = dist(n_goal);
end

%% Output

varargout{1} = route; % Path

if nargout >= 2
    varargout{2} = distant; % Shortest Distance
end

if nargout == 3
    c_n = find(~U);
    varargout{3} = c_n; % Visited Node
end

end