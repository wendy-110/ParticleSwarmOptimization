clear
close all

% Parameters
w=1;                    % Weight for directions
a=0.01;                 % Decay parameter
m=10;                   % Mass of particle (kg)
Tf=6;                  % Final time, s
dt=0.1;                 % Time step, s
vmax=10;                 % Maximum velocity, m/s

VarSize=3;              % 3D

rmin=-5;                % Initial position, lower bound
rmax=5;                 % Initial position, upper bound
Xmin=-7;                % Lower bound in X
Xmax=7;                 % Upper bound in X
Ymin=-7;                % Lower bound in Y
Ymax=7;                 % Upper bound in Y
Zmin=-7;                % Lower bound in Z
Zmax=7;                 % Upper bound in Z
TOL=5;                  % Target distance tolerance

Ns=10;                  % Number of Population

% Target Speed Parameters
a1=1;
a2=1;
a3=0.5;
b1=1;
b2=1;
b3=0.5;
c1=1;
c2=1;
c3=0.5;

% Interaction Force Parameters
alpha_mm1=1;
alpha_mm2=1;
alpha_mt=200;
alpha_mo=100;
beta_mm1=2;
beta_mm2=2;
beta_mt=2;
beta_mo=2;
c_env=1;        % Environmental Damping


% Particle Template
empty_particle.Position=zeros(1,VarSize);
empty_particle.Velocity=zeros(1,VarSize);
empty_particle.Distance=0;
empty_particle.Acceleration=zeros(1,VarSize);

% Create Population Array
particle=repmat(empty_particle,Ns,1);

% Initialize Target
target=[4 0 0];

% Create Obstacles
obstacles=zeros(100,3);
k=0;
for i=1:10
    for j=1:10
        k=k+1;
        y=-0.9+0.2*(j-1);
        z=-0.9+0.2*(i-1);
        obstacles(k,:)=[1.5,y,z];
    end
end

% Initialize Particle Position 
for i=1:Ns
    particle(i).Position=[unifrnd(rmin,rmax,1),unifrnd(rmin,rmax,1),unifrnd(rmin,rmax,1)];
end

m=1;
n=1;

for t=dt:dt:Tf
    
        target(1)=4+a1*cos(a2*t)+a3*t;
        target(2)=0+b1*sin(b2*t)+b3*t;
        target(3)=0+c1*cos(c2*t)+c3*t;
        
        scatter3(target(1),target(2),target(3),'h','r')
        axis([Xmin Xmax Ymin Ymax Zmin Zmax])
        hold on
        scatter3(obstacles(:,1),obstacles(:,2),obstacles(:,3),'filled','s','k')
        
        for i=1:Ns
            
            % Calculate Force Between Particle Members
            force_mm=zeros(1,VarSize);
            
            for j=1:Ns
                
                if i~=j
                    force_mm=(alpha_mm1*norm(particle(i).Position-particle(j).Position)^beta_mm1-alpha_mm2*norm(particle(i).Position-particle(j).Position)^(-beta_mm2))*(particle(i).Position-particle(j).Position)/norm(particle(i).Position-particle(j).Position);
                end
                
            end
            
            % Calculate Force Between Particles and Target
            force_mt=alpha_mt*norm(particle(i).Position-target)^beta_mt*(target-particle(i).Position)/norm(particle(i).Position-target);

            % Calculate Force Between Particle and Obstacles
            force_mo=zeros(1,VarSize);
            for j=1:100
                force_mo=force_mo-(alpha_mo*norm(particle(i).Position-obstacles(j,:))^(-beta_mo)*(obstacles(j,:)-particle(i).Position)/norm(particle(i).Position-obstacles(j,:)));
            end
            
            total_force=force_mm+force_mt+force_mo;
            particle(i).acceleration=total_force/m;
                
            particle(i).Velocity=particle(i).Velocity+dt*particle(i).acceleration;

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
        end
        
        for i=1:Ns
            scatter3(particle(i).Position(:,1),particle(i).Position(:,2),particle(i).Position(:,3),'o','b');
        end
        
        F(m)=getframe;
        m=m+1;
    hold off

end

v = VideoWriter('stage2.avi','Motion JPEG AVI');
v.FrameRate = 10;
open(v)
writeVideo(v,F)
close(v)