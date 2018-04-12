function locations = initialPoints(numPop,radius,numConfigs,stpt)

% Get function inputs
% nConfigs = numConfigs;
nConfigs = 8; %test
% nPop = numPop;
nPop = 	16; %test
% r=radius;
r = 3; %test

% Initialization
locations = zeros(nPop,2,nConfigs);
halfConfigs = nConfigs/2;
theta = 0:2*pi/nPop:2*pi-(2*pi/nPop);
midpt = r/(halfConfigs-1);

% Evenly space pts on circle
for i = 0:1:halfConfigs-1
    
    if i < halfConfigs/2 
        x_vals = (r+i*midpt)*cos(theta)+stpt(1);
        y_vals = (r-i*midpt)*sin(theta)+stpt(2);
        disp('opt1')
    else
        x_vals = (r-i*midpt)*cos(theta)+stpt(1);
        y_vals = (r+i*midpt)*sin(theta)+stpt(2);
        disp('opt2')
    end
    scatter(x_vals,y_vals);
    axis square
    hold on
    locations(:,1,i+1) = x_vals';
    locations(:,2,i+1) = y_vals';
    legend('show')    
end

for i = halfConfigs:1:nConfigs-1
   locations(:,1,i+1) = locations(:,2,i+1-halfConfigs);
   locations(:,2,i+1) = locations(:,1,i+1-halfConfigs);
   scatter(locations(:,1,i+1),locations(:,2,i+1));
end

