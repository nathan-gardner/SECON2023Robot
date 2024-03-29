%{
    Description: Torque Motor Graph Script
    Authors: Nathan Gardner, Madison Kelly, Fatima Al-Heji, Luke McGill, Mark Beech
%}

stallTorque = input('What is the stall torque (kg-cm)? '); % stallTorque is the y intercept
stallTorque = stallTorque * (9.8/100); % convert stall torque form kg-cm to N-m
loadSpeed = input('What is the no load speed? (RPM) '); % loadSpeed is the x intercept
loadSpeed = loadSpeed * (pi/30); % convert load speed from rotations per minute to radian per second

[torque_x_vector,torque_y_vector] = torqueCurve1(stallTorque, loadSpeed);
[power_x_vector,power_y_vector] = powerCurve1(torque_x_vector, torque_y_vector);

subplot(1,2,1)
plot(torque_x_vector, torque_y_vector)
set(gca, 'XLim', [0,400])
set(gca, 'YLim', [0,max(torque_y_vector)])
title('Torque Curve')
xlabel('rotational velocity - rad/s')
ylabel('torque - N-m')

subplot(1,2,2)
plot(power_x_vector,power_y_vector)
set(gca, 'XLim', [0,400])
set(gca, 'YLim', [0,max(power_y_vector)])
title('Power Curve')
xlabel('rotational velocity - rad/s')
ylabel('Power - watt')

% function to calculate power curve
function[power_x_vector,power_y_vector] = powerCurve1(torque_x_vector,torque_y_vector)
    % Multiply the torque and the angular velocity for each point in the
    % torque curve
    power_x_vector = torque_x_vector;
    power_y_vector = torque_x_vector.*torque_y_vector;

end

function[x_vector,y_vector] = torqueCurve1(stallTorque, loadSpeed)
    slope = -stallTorque/loadSpeed;
    x_vector = linspace(0,10000);
    y_vector = slope.*x_vector + stallTorque;

end

