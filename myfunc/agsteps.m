function [ags,numags] = agsteps(ag1,ag2,maxstep)

if all(ag1==ag2)
    ags = ag1;
else
%dag = max(abs(ag1-ag2));
%numags = ceil(dag/maxstep);
numags = max(ceil(abs(ag1-ag2)./maxstep));

ags = linspacearray(ag1,ag2,numags+1);
end
end