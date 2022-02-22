%% IMPORT JONABOT
% This function shows the selected configuration for the Redundant. It is a
% provided manipulator void funtion that takes a 6x1 vector containing the 
% Redundant manipulator joint angles (in Degrees) and joint lenght in mm.
function ImportRedundant(jointValues,SolName)

%% 
RedunBot = importrobot("Redundant_Manipulator\urdf\Redundant.urdf");

%% Assign the angles previously calculated to the JonaBot's joints position
config = homeConfiguration(RedunBot);
config(1).JointPosition = deg2rad(jointValues(1,1)); %Assign theta 1 to first joint
config(2).JointPosition = deg2rad(jointValues(1,2)); %Assign theta 2 to second joint
config(3).JointPosition = jointValues(1,3)/1000; %Assign d3 to third joint
config(4).JointPosition = deg2rad(jointValues(1,4)); %Assign theta 4 to fourth joint
config(5).JointPosition = deg2rad(jointValues(1,5)); %Assign theta 5 to fifth joint
config(6).JointPosition = deg2rad(jointValues(1,6)); %Assign theta 6 to sixth joint

%% Show the Jonabot using the configuration from the last section
show(RedunBot,config);
view(150,50) %change view angle
camup([0 1 0]) % change frame position (Y up).
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);% Change figure size

%Change axis aspect ratio
r = 3.2;
axis([-r r -r+1.5 r -r r]);

%Create angle labels to show
t1 = texlabel('theta1 = ');
a1 = num2str(jointValues(1,1));
t2 = texlabel('theta2 = ');
a2 = num2str(jointValues(1,2));
t3 = texlabel('d3 = ');
a3 = num2str(jointValues(1,3));
t4 = texlabel('theta4 = ');
a4 = num2str(jointValues(1,4));
t5 = texlabel('theta5 = ');
a5 = num2str(jointValues(1,5));
t6 = texlabel('theta6 = ');
a6 = num2str(jointValues(1,6));

%Show title and angle labels
title(['Different Robot Solutions'],'BackgroundColor',[1 1 1],'FontSize',14 ... 
,'Position', [0, 3.3, 0])

x= -3;
text(x,3.0,1.5,SolName,'Color',[0 0 0.5],'FontSize',12,'FontWeight','bold')
text(x,2.4,1.5,'Joint angles:','Color',[0 0 0.2],'FontSize',11,'FontWeight','bold')
text(x,2.0,1.5,[t1,a1])
text(x,1.6,1.5,[t2,a2])
text(x,1.2,1.5,[t3,a3])
text(x,0.8,1.5,[t4,a4])
text(x,0.4,1.5,[t5,a5])
text(x,0.0,1.5,[t6,a6])

%Show current figure
shg;

end


