
% delete all variables
clear;

% close existing figure windows
close ALL;

% load array from file into matlab
% note that columns can be separated by space / tab or commas in matlab
A = load('output.txt');

% file format (note y1, y2, y3 refer to voltages from the simulator
% and not control outputs here)
% time(s),y1,y2,y3,u1,wb,wf

k = 1;
t  = A(:,k); k = k + 1;

y1 = A(:,k); k = k + 1;
y2 = A(:,k); k = k + 1;
y3 = A(:,k); k = k + 1;

u1 = A(:,k); k = k + 1;

wb = A(:,k); k = k + 1;
wf = A(:,k); k = k + 1;

k = 1;

figure(k); k = k + 1;
plot(t,y1);
ylabel('y1, simulator output voltage (V)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,y2);
ylabel('y2, simulator output voltage (V)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,y3);
ylabel('y3, simulator output voltage (V)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,u1);
ylabel('u1, motor control input voltage (V)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,wb);
ylabel('back wheel velocity, wb (rad/s)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,wf);
ylabel('forward wheel velocity, wf (rad/s)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,wb,t,wf,'r');
ylabel('wheel velocity (rad/s)');
xlabel('time (s)');
legend('back wheel velocity wb','forward wheel velocity wf');