function flags = myisvalid(S)
% input S: a 3-by-17 matrix consisting of 3D coordiantes of the joints
% flags: a binary vector telling whether the corresponding bone is valid or
% not. 
% Assume the prior model already loaded in parent functions
%
% copyright: Ijaz Akhter, MPI Tuebingen
% Jan 15, 2015

    global prnts chlds chldsT jmp edges angleSprd sepPlane E2 bounds
    
    nprts = length(chlds);      % excluding torso

    dS = S(:, edges(:,1)) - S(:, edges(:,2));     % find relative depths: coordinates by taking the parent as origin

    flags = true(1,size(dS,2));
    angles = zeros(2,size(dS,2));
    dSl = global2local(dS);     % convert relative to local coordinates

    for i=1:nprts
        chldB = dSl(:, chlds(i));           % the bone to validate
        [th, phi, ri] = cart2sph(chldB(1), chldB(2), chldB(3));
        chldB = chldB/ri;
        th = radtodeg(th);
        phi = radtodeg(phi);
        t_j = floor((th+180)/jmp + 1);
        p_j = floor((phi+90)/jmp + 1);
        angles(:, chlds(i)) = [t_j; p_j];

        if ismember(chlds(i), chldsT)
            if ~angleSprd{i}(t_j, p_j)
                flags(chlds(i)) = false;
            end
        else
            t_p = angles(1,prnts(i));
            p_p = angles(2,prnts(i));

            v = squeeze(sepPlane{i}(t_p,p_p,:));
            v = v/norm(v(1:3));

            if any(isnan(v)) || v'*[chldB;1]>0
                flags(chlds(i)) = false;
            else
                e1 = v(1:3);
                e2 = squeeze(E2{i}(t_p,p_p,:));
                T = gramschmidt([e1,e2,cross(e1, e2)]);
                bnd = squeeze(bounds{i}(t_p,p_p,:));

                u = T(:,2:3)'*chldB;
                if u(1)<bnd(1) || u(1)>bnd(2) || u(2)<bnd(3) || u(2)>bnd(4)
                    flags(chlds(i)) = false;
                end
            end
        end
    end
end