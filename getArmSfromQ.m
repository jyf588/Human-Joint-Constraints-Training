function Sarm = getArmSfromQ(q, lu, ll)
    % The Forward Kinematics calculating Catesian joint positions (S) from 
    % joint angles (q) in generalized coordinates
    % q: [tz, tx, ty (shoulder: zxy eular joint), qe (elbow)] (1-by-4)
    % lu: length of upper arm, dont care
    % ll: length of lower arm, dont care
    % Sarm: 3-by-3 mat, with each column being 3d pos of shld,elbow & wrist
    
    % NOTES: zxy eular joint means: 
    % 1st DoF (+q) left arm adduction, (z)
    % 2nd DoF (+q) left arm extension, (x)
    % 3rd DoF (+q) left arm external rotation, (y)
    % (agent facing -z axis, +x axis pointing from left to right arm)
    
    %shld at 0,0,0
    Sarm = zeros(3);
    
    tz = q(1); tx = q(2); ty = q(3); qe = q(4);
    
    Az = [cos(tz) -sin(tz) 0 0; sin(tz) cos(tz) 0 0;
          0 0 1 0; 0 0 0 1];
    Ax = [1 0 0 0; 0 cos(tx) sin(tx) 0;
          0 -sin(tx) cos(tx) 0; 0 0 0 1];
    Ay = [cos(ty) 0 sin(ty) 0; 0 1 0 0;
          -sin(ty) 0 cos(ty) 0; 0 0 0 1];
    Tu = [1 0 0 0; 0 1 0 -lu; 0 0 1 0; 0 0 0 1];
    Ae = [1 0 0 0; 0 cos(qe) -sin(qe) 0;
        0 sin(qe) cos(qe) 0; 0 0 0 1];
    Tl = [1 0 0 0; 0 1 0 -ll; 0 0 1 0; 0 0 0 1];
    S_elbow = Az * Ax * Ay * Tu * [0; 0; 0; 1];
    Sarm(:,2) = S_elbow(1:3,:);
    S_wrist = Az * Ax * Ay * Tu * Ae * Tl * [0; 0; 0; 1];
    Sarm(:,3) = S_wrist(1:3,:);
end