function validpath = validpath_Link5R_2d(L,O,ag1,ag2,slicenum0,minstep)
validpath = true;
maxiter = slicedatamaxiter(ag1,ag2,slicenum0,minstep);
for j = 1:maxiter
    ag_check = slicedata(ag1,ag2,j,slicenum0);
    if j==1
        ag_check = ag_check(2:end-1,:); % start/stop point already checked
    end

    links_check = Link5R_2d(L,ag_check);
    isintersect_check = iscollision_Link5R_2d(links_check,O);
    if any(isintersect_check)
        validpath = false;
        return
    end
end

end