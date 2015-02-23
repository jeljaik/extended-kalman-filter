disp('processing skin and other data');

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');

expPath = './robotData/backwardTipping/dumperTippingSetup01/icub/';

left_leg_data = importdata(strcat('expPath','left_leg_ft/data.log'));
right_leg_data = importdata(strcat('expPath','left_leg_ft/data.log'));
left_foot_data = importdata(strcat('expPath','left_foot_ft/data.log'));
right_foot_data = importdata(strcat('expPath','right_foot_ft/data.log'));

right_foot_skin = importdata(strcat('expPath','skin/right_foot/data.log'));
 