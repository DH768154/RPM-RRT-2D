function A = SimpleAdjacency(A)

A(A==0) = inf;
A(logical(eye(size(A)))) = 0;

end