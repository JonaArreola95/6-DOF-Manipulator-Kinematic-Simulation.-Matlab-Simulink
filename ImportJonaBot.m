%% IMPORT JONABOT
% This function shows the selected configuration for the JonaBot. It is a
% void funtion that takes a 6x1 vector containing the JonaBot joint angles
% (in Degrees).
function ImportJonaBot(jointAngles,SolName)
%% 
JonaBot = importrobot("JonaBot_Manipulator\urdf\JonaBot_Manipulator.urdf");

%% Assign the angles previously calculated to the JonaBot's joints position
config = homeConfiguration(JonaBot);
config(1).JointPosition = deg2rad(jointAngles(1,1)); %Assign theta 1 to first joint
config(2).JointPosition = deg2rad(jointAngles(1,2)); %Assign theta 2 to second joint
config(3).JointPosition = deg2rad(jointAngles(1,3)); %Assign theta 3 to third joint
config(4).JointPosition = deg2rad(jointAngles(1,4)); %Assign theta 4 to fourth joint
config(5).JointPosition = deg2rad(jointAngles(1,5)); %Assign theta 5 to fifth joint
config(6).JointPosition = deg2rad(jointAngles(1,6)); %Assign theta 6 to sixth joint

%% Show the Jonabot using the configuration from the last section

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
show(JonaBot,config);

%Create angle labels to show
t1 = texlabel('theta1 = ');
a1 = num2str(jointAngles(1,1));
t2 = texlabel('theta2 = ');
a2 = num2str(jointAngles(1,2));
t3 = texlabel('theta3 = ');
a3 = num2str(jointAngles(1,3));
t4 = texlabel('theta4 = ');
a4 = num2str(jointAngles(1,4));
t5 = texlabel('theta5 = ');
a5 = num2str(jointAngles(1,5));
t6 = texlabel('theta6 = ');
a6 = num2str(jointAngles(1,6));

%Show title and angle labels
title(['Different Robot Solutions'],'BackgroundColor',[1 1 1],'FontSize',13)

text(0,1.5,2.4,SolName,'Color',[0 0 0.5],'FontSize',12,'FontWeight','bold')
text(0,1.5,2.0,'Joint angles:','Color',[0 0 0.2],'FontSize',11,'FontWeight','bold')
text(0,1.5,1.6,[t1,a1])
text(0,1.5,1.2,[t2,a2])
text(0,1.5,0.8,[t3,a3])
text(0,1.5,0.4,[t4,a4])
text(0,1.5,0.0,[t5,a5])
text(0,1.5,-0.4,[t6,a6])

%Show current figure
shg;

end


