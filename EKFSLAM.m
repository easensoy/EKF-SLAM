function EKFSLAM()
close all;
clear all;
disp('EKFSLAM Sample Program Starts')
 
time = 0;
endtime = 60; % Simulation End Time [sec]
global dt;
dt = 0.1; % Simulation Tick Time[sec]
nSteps = ceil((endtime - time)/dt);% Number of Simulation Steps
 
% Variables for Storing Calculation Result
result.time=[];
result.xTrue=[];
result.xd=[];
result.xEst=[];
result.z=[];
result.PEst=[];
result.u=[];

% State Vector [x y yaw]'
xEst=[0 0 0]';
global PoseSize;PoseSize=length(xEst);% Number of Robot Pose States[x,y,yaw]
global LMSize;LMSize=2;% Landmark State Quantity[x,y]
% True State
xTrue=xEst;
 
% Dead Reckoning State
xd=xTrue;
 
% Covariance Matrix for Prediction
R=diag([0.2 0.2 toRadian(1)]).^2;
 
% Covariance Matrix for Observation
global Q;
Q=diag([10 toRadian(30)]).^2;%range[m], Angle[rad]

% Simulation Parameter
global Qsigma
Qsigma=diag([0.1 toRadian(20)]).^2;
global Rsigma
Rsigma=diag([0.1 toRadian(1)]).^2;

% Position of Landmark [x, y]
LM=[0 15;
    10 0;
    15 20];
  
MAX_RANGE=20;% Maximum Observation Distance

alpha=1;% Mahalanobis Distance Threshold for Landmark Identification

PEst = eye(3);
initP=eye(2)*1000;
%movcount=0;
 
tic;
% Main loop
for i=1 : nSteps
    time = time + dt;
    % Input
    u=doControl(time);
    % Observation
    [z,xTrue,xd,u]=Observation(xTrue, xd, u, LM, MAX_RANGE);
    
    % ------ EKF SLAM --------
    % Predict
    xEst = f(xEst, u);
    [G,Fx]=jacobF(xEst, u);
    PEst= G'*PEst*G + Fx'*R*Fx;
    
    % Update
    for iz=1:length(z(:,1))% For Each Observation
        % Add Observations as Landmarks
        zl=CalcLMPosiFromZ(xEst,z(iz,:));% Calculate the LM Position from The Observations Themselves
        % Add State Vector and Covariance Matrix
        xAug=[xEst;zl];
        PAug=[PEst zeros(length(xEst),LMSize);
              zeros(LMSize,length(xEst)) initP];
        
        mdist=[];% List of Mahalanobis distances
        for il=1:GetnLM(xAug) % About Each Landmark
            if il==GetnLM(xAug)
                mdist=[mdist alpha];% Use the Parameter Value for The Distance of Newly Added Points
            else
                lm=xAug(4+2*(il-1):5+2*(il-1));
                [y,S,H]=CalcInnovation(lm,xAug,PAug,z(iz,1:2),il);
                mdist=[mdist y'*inv(S)*y];% Calculating the Mahalanobis Distance
            end
        end
        
        % Match with closest Mahalanobis distance
        [C,I]=min(mdist);
      
        % If the One with The Shortest Distance is Added, that Observation is Adopted as a Landmark.
        if I==GetnLM(xAug)
            %disp('New LM')
            xEst=xAug;
            PEst=PAug;
        end
        
        lm=xEst(4+2*(I-1):5+2*(I-1));% Get Mapped Landmark Data
        % Calculation of Innovation
        [y,S,H]=CalcInnovation(lm,xEst,PEst,z(iz,1:2),I);
        K = PEst*H'*inv(S);
        xEst = xEst + K*y;
        PEst = (eye(size(xEst,1)) - K*H)*PEst;
    end
    
    xEst(3)=PI2PI(xEst(3));% Angle Correction
    
    % Simulation Result
    result.time=[result.time; time];
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.xEst=[result.xEst;xEst(1:3)'];
    result.u=[result.u; u'];
    
    % Animation (Remove Some Flames)
    if rem(i,5)==0 
        Animation(result,xTrue,LM,z,xEst,zl);
        %movcount=movcount+1;
        %mov(movcount) = getframe(gcf);% Get Animation Frame
    end
end
toc

% Save Animation
%movie2avi(mov,'movie.avi');

DrawGraph(result,xEst,LM);

function [y,S,H]=CalcInnovation(lm,xEst,PEst,z,LMId)
% A Function that Computes Innovation from Matching Results
global Q;
delta=lm-xEst(1:2);
q=delta'*delta;
zangle=atan2(delta(2),delta(1))-xEst(3);
zp=[sqrt(q) PI2PI(zangle)];%観測値の予測
y=(z-zp)';
H=jacobH(q,delta,xEst,LMId);
S=H*PEst*H'+Q;

function n=GetnLM(xEst)
% A Function to Calculate the Number of Landmarks
n=(length(xEst)-3)/2;

function zl=CalcLMPosiFromZ(x,z)
% A Function that Computes the LM Position from Observations
zl=x(1:2)+[z(1)*cos(x(3)+z(2));z(1)*sin(x(3)+z(2))];

function Animation(result,xTrue,LM,z,xEst,zl)
% Function to Draw the Animation
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
% Viewing Observation Lines
if~isempty(z)
    for iz=1:length(z(:,1))
        ray=[xTrue(1:2)';z(iz,3:4)];
        plot(ray(:,1),ray(:,2),'-r');hold on;
    end
end
% SLAM Display of the Map
for il=1:GetnLM(xEst);
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.c');hold on;
end
plot(zl(1,:),zl(2,:),'.b');hold on;
plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
arrow=0.5;
x=result.xEst(end,:);
quiver(x(1),x(2),arrow*cos(x(3)),arrow*sin(x(3)),'ok');hold on;
axis equal;
grid on;
% When Saving Videos
%movcount=movcount+1;
%mov(movcount) = getframe(gcf);% Get Animation Frame
drawnow;

function x = f(x, u)
% Motion Model
global dt;
global PoseSize;
global LMSize;
 
F = horzcat(eye(PoseSize),zeros(PoseSize,LMSize*GetnLM(x)));
 
B = [dt*cos(x(3)) 0
     dt*sin(x(3)) 0
     0 dt];

x= x+F'*B*u;
x(3)=PI2PI(x(3));% Angle Correction

function [G,Fx]=jacobF(x, u)
% Computation for the Jacobian Matrix Function of the Motion Model
global dt;
global PoseSize;
global LMSize;

Fx = horzcat(eye(PoseSize),zeros(PoseSize,LMSize*GetnLM(x)));
 
jF=[0 0 -dt*u(1)*sin(x(3))
    0 0 dt*u(1)*cos(x(3))
    0 0 0];

G=eye(length(x))+Fx'*jF*Fx;

function H=jacobH(q,delta,x,i)
% A Function that Computes the Jacobian Matrix of an Observation Model
sq=sqrt(q);
G=[-sq*delta(1) -sq*delta(2) 0 sq*delta(1) sq*delta(2);
    delta(2)    -delta(1)   -1 -delta(2)    delta(1)];
G=G/q;
F=[eye(3) zeros(3,2*GetnLM(x));
   zeros(2,3) zeros(2,2*(i-1)) eye(2) zeros(2,2*GetnLM(x)-2*i)];
H=G*F;

function u = doControl(time)
%Calc Input Parameter
T=10; % [sec]
 
% [V yawrate]
V=1.0; % [m/s]
yawrate = 5; % [deg/s]
 
u =[ V*(1-exp(-time/T)) toRadian(yawrate)*(1-exp(-time/T))]';

function [z, x, xd, u] = Observation(x, xd, u, LM ,MAX_RANGE)
%Calc Observation from noise prameter
global Qsigma;
global Rsigma;
 
x=f(x, u);% Ground Truth
u=u+Qsigma*randn(2,1);%add Process Noise
xd=f(xd, u);% Dead Reckoning
%Simulate Observation
z=[];
for iz=1:length(LM(:,1))
    % Convert LM Position to Robot Coordinate System
    yaw=zeros(3,1);
    yaw(3)=-x(3);
    localLM=HomogeneousTransformation2D(LM(iz,:)-x(1:2)',yaw');
    d=norm(localLM);% Distance
    if d<MAX_RANGE % Within the Observation Range
        noise=Rsigma*randn(2,1);
        z=[z;[d+noise(1) PI2PI(atan2(localLM(2),localLM(1))+noise(2)) LM(iz,:)]];
    end
end

function DrawGraph(result,xEst,LM)
% Plot Result
 figure(1);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 4); hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;% True Landmark Position
% LM map display
for il=1:GetnLM(xEst);
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.g');hold on;
end
 
title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
grid on;
axis equal;

function angle=PI2PI(angle)
% A Function that Corrects the Angle of the Robot to the Range of π to π
angle = mod(angle, 2*pi);

i = find(angle>pi);
angle(i) = angle(i) - 2*pi;

i = find(angle<-pi);
angle(i) = angle(i) + 2*pi;

function out = HomogeneousTransformation2D(in, base, mode)
%function out = HomogeneousTransformation2D(in, base,mode)
%HOMOGENEOUSTRANSFORMATION2D Two-Dimensional Homogeneous Transformation Function
%   From Single Point Conversion to Batch Conversion of Multiple Points.
%   It is convenient for coordinate transformation of point cloud of laser or radar.
%   If only in and base are given as arguments, it translates after rotating each point.
%
%   Input1: Pre-Transform Vector [x_in_1 y_in_1;
%                        x_in_2  y_in_2;
%                               ....]
%           Even if the vector before transformation contains more than three elements, 
%           the first two are taken out and the rest are ignored.
%   Input2: Reference Vector (Such as Track Position) [x_base y_base theta_base]
%   Input3: Homogeneous Conversion Mode: Works Even if This Variable is not Entered as an Argument．
%           If no arguments are given, the default is to translate each point after rotating it．
%           When mode=0, each point is rotated and then translated.
%           If mode=1, rotate after translating.
%
%   Output1: Converted Vector [x_out y_out;
%                        x_out_2  y_out_2;
%                               ....]

% Rotation Matrix
Rot=[cos(base(3)) sin(base(3)); -sin(base(3)) cos(base(3))];

% Store the Base Coordinates for the Number of Points in the Array
Nin=size(in);
baseMat=repmat(base(1:2),Nin(1),1);

% If there is data other than x-y, put it in another variable once.
if Nin(2)>=3
    inxy=in(:,1:2);
    inOther=in(:,3:end);
    in=inxy;
end

% Homogeneous Transformation
if nargin==2 || mode==0 % Rotation → Translation
    out=baseMat+in*Rot;
else % Translation → Rotation
    out=(baseMat+in)*Rot;
end
    
% Glue Stripped Values
if Nin(2)>=3
    out=[out inOther];
end

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;