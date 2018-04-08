% Implementation of Algorithm 1 in the paper:
% Generate training data points of LEFT-leg(& foot) poses
% ultilizing the isvalid() function in Akhter and Black's paper

% isvalid() takes full-body pose (see def for pose in its readme.pdf)
% we first generate a dummy full-body pose and substitute the left-leg
% with random sampled poses, and we only care the subarray of isvalid's output
% corresponding to the left-leg DoFs here.
% Bone lengths never matter as isvalid() only cares bone orientations.

% NOTE: isvalid defines left and right from viewer's perspective, different
% from our definition (from the agent itself). We thus substitute
% pose(:,10:13) which is the RIGHT leg as in isvalid()
% NOTE2: agent facing -z axis, +x axis pointing from left to right arm

pose = [0 -0.4    0;
0 0 0;
-0.2    0    0;
-0.2 -0.4    0;
-0.2 -0.8    -0.1;
0.2    0    0;
0.2 -0.4    0;
0.2 -0.8   -0.1;
0 0.2 0;
-0.15  -0.6     0;
-0.15  -1.2     0; 
-0.15  -1.6     0;
-0.15  -1.6     -0.2;
0.15 -0.6    0; 
0.15 -1.2    0;
0.15 -1.6    0;
0.15  -1.6     -0.2];
pose = pose';
% isvalid(pose)

% **To accelerate, I move the database loading code in isvalid() here
% so that matfiles do not need to be loaded in each forloop iteration
global prnts chlds chldsT di a jmp edges angleSprd sepPlane E2 bounds
% shared with myisvalid, global2local and local2global functions

var = load('staticPose');
di = var.di;
a = var.a;

var = load('jointAngleModel_v2');
jmp = var.jmp;

chlds = var.chlds;
prnts = var.prnts;
edges = var.edges;
angleSprd = var.angleSprd;
sepPlane = var.sepPlane;
E2 = var.E2;
bounds = var.bounds;

chldsT = [3 6 8 10 14];     % torso's child

N = 100; 
qTrain = zeros(N,7);
for i = 1:N
    % sample joint angles in some loose boxed ranges
    % i.e. some poses are never considered, e.g. knee bending backwards
    tz = rand * 2 * pi;
    tx = rand * 2 * pi;
    ty = rand * pi - pi/2; 
    qe = rand * pi; % knee DoF
    hx = rand * pi - pi/2; % ankle DoFs
    hy = rand * pi - pi/2;
    qTrain(i,1:6) = [tz,tx,ty,qe,hx,hy];
    
    % ForwardKinematics: joint angles in q coordinates -> joint positions S
    % Again, bone lengths are just dummy.
    Sarm = getLegSfromQFull([tz, tx, ty, qe, hx, hy], 0.6, 0.4, 0.2); 
    Sarm(1,:) = Sarm(1,:) - 0.15; 
    Sarm(2,:) = Sarm(2,:) - 0.6;
    pose(:,10:13) = Sarm;
    
    % **A trimmed isvalid() code so that  
    % MatFiles are not be loaded in every iteration
    re = myisvalid(pose);
    qTrain(i,7) = bi2de(re(10:12),'left-msb');
end

% save('random_leg.mat','qTrain');

% rejection sampling:
% when N is reasonably large, each invalid case should have more samples
% then the valid case
qTrain(qTrain(:,7)==1,7) = 0;
qTrain(qTrain(:,7)==5,7) = 4;
validInd = find(qTrain(:,7)==7);
validSize = size(validInd,1);
Ind0 = find(qTrain(:,7)==0);
Ind0 = Ind0(1:validSize);
Ind4 = find(qTrain(:,7)==4);
Ind4 = Ind4(1:validSize);
Ind6 = find(qTrain(:,7)==6);
Ind6 = Ind6(1:validSize);

qTrain_ba = qTrain([Ind0;Ind4;Ind6;validInd],:);

qTrain_ba(qTrain_ba(:,7)~=7,7) = 0;
qTrain_ba(qTrain_ba(:,7)==7,7) = 1;

% sum(qTrain_ba(:,7))
% size(qTrain_ba)

% transform the q's to their sinosoids since they are circular data
qTrain_ba_sin = zeros(size(qTrain_ba,1),9);
qTrain_ba_sin(:,1) = cos(qTrain_ba(:,1));
qTrain_ba_sin(:,2) = sin(qTrain_ba(:,1));
qTrain_ba_sin(:,3) = cos(qTrain_ba(:,2));
qTrain_ba_sin(:,4) = sin(qTrain_ba(:,2));
qTrain_ba_sin(:,5) = cos(qTrain_ba(:,3) + pi/2);
qTrain_ba_sin(:,6) = cos(qTrain_ba(:,4));
qTrain_ba_sin(:,7) = cos(qTrain_ba(:,5) + pi/2);
qTrain_ba_sin(:,8) = cos(qTrain_ba(:,6) + pi/2);
qTrain_ba_sin(:,9) = qTrain_ba(:,7);

save('randomsin_leg_left.mat','qTrain_ba_sin');
