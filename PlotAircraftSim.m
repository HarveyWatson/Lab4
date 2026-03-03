% Contributors: Andrew Yates
% Course Number: ASEN 3801
% File Name: PlotAircraftSim
% Created: 3/3/26





function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%
% Inputs: time = time vector of length n
%         aircraft_state_array = 12xn array of aircraft states over time
%         control_input_array = 4xn array of control inputs over time
%         fig = 6x1 vector of figure numbers to plot over
%         col = plotting option for particular section of plots
%
% Outputs: 6 figures; 4 figures with 3 subplots each of aircraft states
%          over time (one figure of inertial position, one of euler angles, one of
%          inertial velocity, and one of angular velocity), 1 figure with 4
%          subplots showing control inputs over time, and 1 figure showing
%          three-dimensional aircraft path with positive height upward
%
% Methodology: Using MATLAB plotting capabilities given these inputs to
% create clean, readable, and informative plots



%% figure 1 - Inertial Position over Time
figure(fig(1));

subplot(3,1,1);
hold on;
plot(time,aircraft_state_array(1,:),col);
xlabel('time (s)');
ylabel('inertial position x_E (m)');

subplot(3,1,2);
hold on;
plot(time,aircraft_state_array(2,:),col);
xlabel('time (s)');
ylabel('inertial position y_E (m)');

subplot(3,1,3);
hold on;
plot(time,aircraft_state_array(3,:),col);
xlabel('time (s)');
ylabel('inertial position z_E (m)');

sgtitle('Inertial Position over Time');


%% figure 2 - Euler Angles Over Time
figure(fig(2));

subplot(3,1,1);
hold on;
plot(time,aircraft_state_array(4,:),col);
xlabel('time (s)');
ylabel('roll euler angle \phi (rad)');

subplot(3,1,2);
hold on;
plot(time,aircraft_state_array(5,:),col);
xlabel('time (s)');
ylabel('pitch euler angle \theta (rad)');

subplot(3,1,3);
hold on;
plot(time,aircraft_state_array(6,:),col);
xlabel('time (s)');
ylabel('yaw euler angle \psi (rad)');

sgtitle('Euler Angles over Time');


%% figure 3 - Inertial Velocity (Body Frame) over Time
figure(fig(3));

subplot(3,1,1);
hold on;
plot(time,aircraft_state_array(7,:),col);
xlabel('time (s)');
ylabel('x inertial velocity u^E (m/s)');

subplot(3,1,2);
hold on;
plot(time,aircraft_state_array(8,:),col);
xlabel('time (s)');
ylabel('y inertial velocity v^E (m/s)');

subplot(3,1,3);
hold on;
plot(time,aircraft_state_array(9,:),col);
xlabel('time (s)');
ylabel('z inertial velocity w^E (m/s)');

sgtitle('Inertial Velocity (Body Frame) over Time');


%% figure 4 - Angular Velocity over Time
figure(fig(4));

subplot(3,1,1);
hold on;
plot(time,aircraft_state_array(10,:),col);
xlabel('time (s)');
ylabel('roll rate p (rad/s)');

subplot(3,1,2);
hold on;
plot(time,aircraft_state_array(11,:),col);
xlabel('time (s)');
ylabel('pitch rate q (rad/s)');

subplot(3,1,3);
hold on;
plot(time,aircraft_state_array(12,:),col);
xlabel('time (s)');
ylabel('yaw rate r (rad/s)');

sgtitle('Angular Velocity over Time');


%% figure 5 - Control Inputs over Time
figure(fig(5));

subplot(2,2,1);
hold on;
plot(time,control_input_array(1,:),col);
xlabel('time (s)');
ylabel('total z control input Z_C (N)');

subplot(2,2,2);
hold on;
plot(time,control_input_array(2,:),col);
xlabel('time (s)');
ylabel('total roll control input (L_C (N\cdotm)');

subplot(2,2,3);
hold on;
plot(time,control_input_array(3,:),col);
xlabel('time (s)');
ylabel('total pitch control input (M_C (N\cdotm)');

subplot(2,2,4);
hold on;
plot(time,control_input_array(4,:),col);
xlabel('time (s)');
ylabel('total yaw control input (N_C (N\cdotm)');

sgtitle('Control Inputs over Time');


%% figure 6 - Path of Aircraft 
n = length(time);
figure(fig(6));
hold on;
plot(aircraft_state_array(1,:),aircraft_state_array(2,:),-1.*aircraft_state_array(3,:),col);
plot(aircraft_state_array(1,1),aircraft_state_array(2,1),-1.*aircraft_state_array(3,1),'gx');
plot(aircraft_state_array(1,n),aircraft_state_array(2,n),-1.*aircraft_state_array(3,n),'rx');
xlabel('x (m)')
ylabel('y (m)')
zlabel('-z (m)')
title('Aircraft Path')
legend('path of aircraft','aircraft starting point','aircraft final point')


end