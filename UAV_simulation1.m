clear
close all

% Parameters
w=1;                    % Weight for directions
a=0.01;                 % Decay parameter
Fm=10^5;                % Force mass ratio
Tf=30;                  % Final time, s
dt=0.1;                 % Time step, s
vmax=100;               % Maximum velocity, m/s

VarSize=3;              % 3D

rmin=0;                 % Initial position, lower bound
rmax=10;                % Initial position, upper bound
Xmin=0;                 % Lower bound in X
Xmax=500;               % Upper bound in X
Ymin=0;                 % Lower bound in Y
Ymax=500;               % Upper bound in Y
Zmin=0;                 % Lower bound in Z
Zmax=10;                % Upper bound in Z
TOL=5;                  % Target distance tolerance

Nt=10;                  % Number of Targets
Ns=10;                 % Number of Population

% Target Template
empty_target.Position=zeros(1,VarSize);

% Particle Template
empty_particle.Position=zeros(1,VarSize);
empty_particle.Velocity=zeros(1,VarSize);
empty_particle.Distance=0;
empty_particle.Direction=zeros(1,VarSize);

% Create Target and Population Array
target=repmat(empty_target,Nt,1);
particle=repmat(empty_particle,Ns,1);

% Initialize Array for Found Target
foundtarget=[-1 -1 -1];

% Initialize Targets and Population
for i=1:Nt
    target(i).Position=[unifrnd(Xmin,Xmax,1),unifrnd(Ymin,Ymax,1),unifrnd(Zmin,Zmax,1)];
end

for i=1:Ns
    particle(i).Position=[unifrnd(rmin,rmax,1),unifrnd(rmin,rmax,1),unifrnd(rmin,rmax,1)];                        % Initialize Particle Position 
end

k=1;
n=1;

for t=dt:dt:Tf
    
    % Plot All Targets
    if size(target,2)==0
        break
    else
        
        for j=1:Nt
            scatter3(target(j).Position(:,1),target(j).Position(:,2),target(j).Position(:,3),'x','b')
            axis([Xmin Xmax Ymin Ymax Zmin Zmax])
            hold on
        end
        
    end
    
    % Plot Found Targets
    scatter3(foundtarget(:,1),foundtarget(:,2),foundtarget(:,3),'o','r')
    
        for i=1:Ns
            
            if size(target,2)==0
                break
            else

            particle(i).Direction=zeros(1,VarSize);


            for j=1:Nt
                tar_distance=norm(target(j).Position-particle(i).Position);             % Calculate Target Distance
                tar_direction=(target(j).Position-particle(i).Position)/tar_distance;
                particle(i).Direction=particle(i).Direction+w*tar_direction*exp(-a*tar_distance);
                if tar_distance<= TOL
                    foundtarget(n,:)=target(j).Position;
                    n=n+1;
                end
            end
            particle(i).Direction=particle(i).Direction/norm(particle(i).Direction);    
            particle(i).Velocity=particle(i).Velocity+dt*Fm*particle(i).Direction;

            if norm(particle(i).Velocity)>vmax
                particle(i).Velocity=vmax/norm(particle(i).Velocity)*particle(i).Velocity;
            end

            lastPosition=particle(i).Position;
            particle(i).Position=particle(i).Position+dt*particle(i).Velocity;

            % Apply Lower and Upper Bound Limits
            particle(i).Position(1)=max(particle(i).Position(1), Xmin);
            particle(i).Position(1)=min(particle(i).Position(1), Xmax);
            particle(i).Position(2)=max(particle(i).Position(2), Ymin);
            particle(i).Position(2)=min(particle(i).Position(2), Ymax);
            particle(i).Position(3)=max(particle(i).Position(3), Zmin);
            particle(i).Position(3)=min(particle(i).Position(3), Zmax);
            
            particle(i).Distance=particle(i).Distance+norm(particle(i).Position-lastPosition);

            % Mark Found Targets
            tar_position=[];
            for j=1:Nt
                tar_position=[tar_position;target(j).Position];
            end

            for j=1:size(tar_position,1)
                if ismember(tar_position(j,:),foundtarget,'rows')
                    target(j)=[];
                end
            end
            Nt=size(target,1);

            end
    
        end
        
    for i=1:Ns
            scatter3(particle(i).Position(:,1),particle(i).Position(:,2),particle(i).Position(:,3),'o','b');
            hold on
        end
        
    F(k)=getframe;
    k=k+1;
    hold off

end

v = VideoWriter('PSO3D.avi','Motion JPEG AVI');
v.FrameRate = 10;
open(v)
writeVideo(v,F)
close(v)