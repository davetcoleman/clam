
clear
clc

data = csvread('valid_points.dat')
box = [ .1,.2
        .1,-.2
        .5,-.2
        .5,.2
        .1,.2 ];

plot( data(:,1), data(:,2),'o', ...
      box(:,1), box(:,2),'b-',...
      [0],[0],'s','LineWidth',2);
axis([0,.6,-.3,.3]);
set(gca,'FontSize',14)

%legend('Number of lookups','O(n log n)','Location','NorthWest')
xlabel('x position')
ylabel('y position')
title('Valid planning positions in plane');