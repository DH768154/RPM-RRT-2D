function maxiter = slicedatamaxiter(p1,p2,slicenum0,minstep)
% axis 1 from 0 ~ 20
% axis 2 from 5 ~ 10
% find iteration needed for slicedata function to ensure angle step for
% axis 1 less than minstep(1) and axis 2 less than minstep(2)

maxiter = ceil(log2(max(2*abs(p2-p1)/slicenum0./minstep)));
%maxiter = ceil(1-log2(min(minstep./(abs(p2-p1)/slicenum0))));
maxiter = max([0,maxiter]);
end