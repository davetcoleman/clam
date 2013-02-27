
clear
clc

file_number = 41;
file_name = strcat('trajectory_error_',num2str(file_number),'.csv');
num_data = csvread(file_name,1,0);
csv_data = textread(file_name,'%s','whitespace',',');
key_names = csv_data(1:7);

[rows,cols] = size(num_data);

threshold = [0 0.045; 150 0.045];

plot( num_data );

legend(key_names, 'Interpreter', 'none');
lh=findall(gcf,'tag','legend');
set(lh,'location','northeastoutside');
     
axis([0,150,-0.1,0.1]);
set(gca,'FontSize',14)

%legend('Number of lookups','O(n log n)','Location','NorthWest')
xlabel('Trajectory #')
ylabel('Position Error')
title('Servo Position Error wrt Trajectory Point #');