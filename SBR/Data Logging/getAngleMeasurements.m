filename = 'output.txt';
[accX, accY, accZ, gyroX, gyroY, gyroZ, time] = readSensorData(filename);
nSamples = length(time);  

rollAcc = zeros(1, nSamples);
pitchAcc = zeros(1, nSamples);
rollGyro = zeros(1, nSamples);
pitchGyro = zeros(1, nSamples);

% Calculate roll and pitch angles from accelerometer data
rollAcc = atand(accY ./ sqrt(accX.^2 + accZ.^2));
pitchAcc = atand(-accX ./ sqrt(accY.^2 + accZ.^2));

% Integrate gyroscope data to get roll and pitch angles
rollGyro = cumtrapz(time, gyroX);
pitchGyro = cumtrapz(time, gyroY);

% for i = 1:nSamples
%     rollAcc(1, i) = atand(accY(i) / sqrt(accX(i)^2 + accZ(i)^2));
%     pitchAcc(1, i) = atand(-accX(i) / sqrt(accY(i)^2 + accZ(i)^2));
% end
% 
% for i = 2:nSamples
%     dt = time(i) - time(i-1); % Calculate time difference
%     rollGyro(i) = rollGyro(i-1) + gyroX(i) * dt; % Integrate gyroX to get rollGyro
%     pitchGyro(i) = pitchGyro(i-1) + gyroY(i) * dt; % Integrate gyroY to get pitchGyro
% end

% Plot roll angles
figure;
subplot(2,1,1);
plot(time, rollAcc, 'b', 'LineWidth', 1);
hold on;
plot(time, rollGyro, 'r--', 'LineWidth', 1);
hold off;
xlabel('Time');
ylabel('Roll Angle (degrees)');
legend('Accelerometer', 'Gyroscope');
title('Roll Angle: Accelerometer vs. Gyroscope');

% Plot pitch angles
subplot(2,1,2);
plot(time, pitchAcc, 'b', 'LineWidth', 1);
hold on;
plot(time, pitchGyro, 'r--', 'LineWidth', 1);
hold off;
xlabel('Time');
ylabel('Pitch Angle (degrees)');
legend('Accelerometer', 'Gyroscope');
title('Pitch Angle: Accelerometer vs. Gyroscope');