function Sleg = getLegSfromQFull(q, lu, ll, lf)
    % The Forward Kinematics calculating Catesian joint positions (S) from 
    % joint angles (q) in generalized coordinates
    % q: [tz, tx, ty (hip: zxy eular joint), qe (elbow), hx, hy] (1-by-6)
    % lu: length of thigh, dont care
    % ll: length of (lower) leg, dont care
    % lf: length of foot, dont care
    % Sleg: 3-by-4 mat, with each column being 3d pos of hip, knee, heel
    % and toe
    
    % NOTES: zxy eular joint means: 
    % 1st DoF (+q) left thigh adduction, (z)
    % 2nd DoF (+q) left thigh extension, (x)
    % 3rd DoF (+q) left thigh external rotation, (y)
    % Ankle:
    % 5th DoF (+q) left ankle dorsiflexion
    % 6th DoF (+q) left ankle eversion
    % (agent facing -z axis, +x axis pointing from left to right arm)
    
    % hip at 0,0,0
    Sleg = zeros(3,4);
    
    tz = q(1); tx = q(2); ty = q(3); qe = q(4); hx = q(5); hy = q(6);
    
    Az = [cos(tz) -sin(tz) 0 0; sin(tz) cos(tz) 0 0;
          0 0 1 0; 0 0 0 1];
    Ax = [1 0 0 0; 0 cos(tx) sin(tx) 0;
          0 -sin(tx) cos(tx) 0; 0 0 0 1];
    Ay = [cos(ty) 0 sin(ty) 0; 0 1 0 0;
          -sin(ty) 0 cos(ty) 0; 0 0 0 1];
    Tu = [1 0 0 0; 0 1 0 -lu; 0 0 1 0; 0 0 0 1];
    S_knee = Az * Ax * Ay * Tu * [0; 0; 0; 1];
    Sleg(:,2) = S_knee(1:3,:);
    
    Ae = [1 0 0 0; 0 cos(qe) sin(qe) 0;
        0 -sin(qe) cos(qe) 0; 0 0 0 1];
    Tl = [1 0 0 0; 0 1 0 -ll; 0 0 1 0; 0 0 0 1];
    S_heel = Az * Ax * Ay * Tu * Ae * Tl * [0; 0; 0; 1];
    Sleg(:,3) = S_heel(1:3,:);
    
    Ahx = [1 0 0 0; 0 cos(hx) -sin(hx) 0;
          0 sin(hx) cos(hx) 0; 0 0 0 1];
    Ahy = [cos(hy) 0 sin(hy) 0; 0 1 0 0;
          -sin(hy) 0 cos(hy) 0; 0 0 0 1];
    Tf = [1 0 0 0; 0 1 0 0; 0 0 1 -lf; 0 0 0 1];
    S_toe = Az * Ax * Ay * Tu * Ae * Tl * Ahx * Ahy * Tf * [0; 0; 0; 1];
    Sleg(:,4) = S_toe(1:3,:);
end
    
    
    