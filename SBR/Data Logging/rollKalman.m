filename = 'output.txt';
[accX, accY, accZ, gyroX, gyroY, gyroZ, time] = readSensorData(filename);
nSamples = length(time);  

% Calculate roll and pitch angles from accelerometer data
rollAcc = atand(accY ./ sqrt(accX.^2 + accZ.^2));

% Integrate gyroscope data to get roll and pitch angles
rollGyro = cumtrapz(time, gyroX);

% Kalman filter setup
dt = mean(diff(time)); % Time step
F = [1 dt; 0 1]; % State transition matrix
H = [1 0; 0 1]; % Measurement matrix (we measure both angle and angular velocity)
Q = 0.1 * eye(2); % Process noise covariance
R = 5 * eye(2); % Measurement noise covariance
P = 0.1 * eye(2); % Initial state estimate covariance
x = [0; 0]; % Initial state estimate

% Kalman filter loop for both gyro and accelerometer
kalmanRoll = zeros(size(time));
for i = 1:numel(time)
    % Prediction
    x = F * x;
    P = F * P * F' + Q;

    % Update
    K = P * H' / (H * P * H' + R);
    z = [rollAcc(i); gyroX(i)]; % Combined measurement of angle and angular velocity
    x = x + K * (z - H * x);
    P = (eye(2) - K * H) * P;

    % Store the fused estimate
    kalmanRoll(i) = x(1);
end

% Kalman filter loop for only gyro 
H = [0 1]; 
R = 5; % Measurement noise covariance
kalmanRollGyro = zeros(size(time));
for i = 1:numel(time)
    % Prediction
    x = F * x;
    P = F * P * F' + Q;

    % Update
    K = P * H' / (H * P * H' + R);
    z = [gyroX(i)]; % Combined measurement of angle and angular velocity
    x = x + K * (z - H * x);
    P = (eye(2) - K * H) * P;

    % Store the fused estimate
    kalmanRollGyro(i) = x(1);
end

% Kalman filter loop for only gyro 
H = [1 0]; 
R = 5;
kalmanRollAcc = zeros(size(time));
for i = 1:numel(time)
    % Prediction
    x = F * x;
    P = F * P * F' + Q;

    % Update
    K = P * H' / (H * P * H' + R);
    z = [rollAcc(i)]; % Measurement of angle 
    x = x + K * (z - H * x);
    P = (eye(2) - K * H) * P;

    % Store the fused estimate
    kalmanRollAcc(i) = x(1);
end

% Plot the data
figure;
plot(time, rollAcc, 'yellow', 'LineWidth', 1.5);
hold on;
plot(time, rollGyro, 'g', 'LineWidth', 1.5);
plot(time, kalmanRoll, 'b', 'LineWidth', 1.5);
plot(time, kalmanRollGyro, 'm', 'LineWidth', 1.5);
plot(time, kalmanRollAcc, 'r', 'LineWidth', 1.5);
hold off;
xlabel('Time');
ylabel('Roll Angle (degrees)');
legend('Accelerometer', 'Gyroscope', 'Kalman Filter Both', 'Kalman Gyro', 'Kalman Accelerometer');
title('Roll Angle Estimation using Kalman Filter');