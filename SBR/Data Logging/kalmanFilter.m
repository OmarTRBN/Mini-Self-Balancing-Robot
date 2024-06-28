filename = 'output.txt';
[accX, accY, accZ, gyroX, gyroY, gyroZ, time] = readSensorData(filename);
nSamples = length(time);  

% Calculate roll and pitch angles from accelerometer data
Zn = atand(accY ./ sqrt(accX.^2 + accZ.^2));
rollAcc = atand(accY ./ sqrt(accX.^2 + accZ.^2));

% Integrate gyroscope data to get roll and pitch angles
rollGyro = cumtrapz(time./1000000, gyroX);

H = [1 0];
R = 0.0300;
x = [0; 0];
P = [0 0; 0 0];
Q = [0.001 0; 0 0.003];
Kn = [0; 0];
I = eye(2);
u = gyroX;

for i=2:508
    dt = (time(i)-time(i-1))/1000000;
    F = [1 -dt; 0 1];
    G = [dt; 0];
    
    x = F*x + G*u(i);
    P = F*P*F' + Q;
    
    Kn = P*H' * inv(H*P*H' + R);
    x = x + Kn*(Zn(i) - H*x);
    % P = (I - Kn*H)*P*(I - Kn*H)' + Kn*R*Kn';
    P = (I - Kn*H)*P;
    
    kalmanRoll(i) = x(1);
    kalmanBias(i) = x(2);
end

figure;
plot(time, rollAcc, 'red', 'LineWidth', 1.5);
hold on;
plot(time, rollGyro, 'blue', 'LineWidth', 1.5);
plot(time, kalmanRoll, 'green', 'LineWidth', 1.5);
hold off;

xlabel('Time');
ylabel('Roll Angle (degrees)');

legend('Accelerometer', 'Gyroscope', 'Kalman Filter');
title('Roll Angle Estimation using Kalman Filter');

