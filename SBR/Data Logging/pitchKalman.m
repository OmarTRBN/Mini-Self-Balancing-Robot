filename = 'output.txt';
[accX, accY, accZ, gyroX, gyroY, gyroZ, time] = readSensorData(filename);
nSamples = length(time);  

pitchAcc = zeros(1, nSamples);
pitchGyro = zeros(1, nSamples);

% Calculate pitch angles from accelerometer data
pitchAcc = atand(-accX ./ sqrt(accY.^2 + accZ.^2));

% Integrate gyroscope data to get pitch angles
pitchGyro = cumtrapz(time, gyroY);

% Kalman filter setup
dt = mean(diff(time)); % Time step
F = [1 dt; 0 1]; % State transition matrix
H = [1 0; 0 1]; % Measurement matrix (we measure both angle and angular velocity)
Q = 0.1 * eye(2); % Process noise covariance
R = 20 * eye(2); % Measurement noise covariance
P = 0.1 * eye(2); % Initial state estimate covariance
x = [0; 0]; % Initial state estimate

% Kalman filter loop
kalmanPitch = zeros(size(time));
for i = 1:numel(time)
    % Prediction
    x = F * x;
    P = F * P * F' + Q;

    % Update
    K = P * H' / (H * P * H' + R);
    z = [pitchAcc(i); gyroY(i)]; % Combined measurement of angle and angular velocity
    x = x + K * (z - H * x);
    P = (eye(2) - K * H) * P;

    % Store the fused estimate
    kalmanPitch(i) = x(1);
end

% Plot the data
figure;
plot(time, pitchAcc, 'b', 'LineWidth', 1.5);
hold on;
plot(time, pitchGyro, 'r--', 'LineWidth', 1.5);
plot(time, kalmanPitch, 'g', 'LineWidth', 1.5);
hold off;
xlabel('Time');
ylabel('Roll Angle (degrees)');
legend('Accelerometer', 'Gyroscope', 'Kalman Filter');
title('Roll Angle Estimation using Kalman Filter');