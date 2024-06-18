function [accX, accY, accZ, gyroX, gyroY, gyroZ, time] = readSensorData(filename)
    % Read the contents of the text file
    data = fileread(filename);

    % Split the file into lines
    lines = strsplit(data, '\n');

    % Initialize arrays to store data
    accX = [];
    accY = [];
    accZ = [];
    gyroX = [];
    gyroY = [];
    gyroZ = [];
    time = [];

    % Loop through each line and extract values
    for i = 1:numel(lines)
        line = lines{i};
        if contains(line, 'Acc X:')
            % Extract acceleration values
            accX = [accX str2double(extractBetween(line, 'Acc X: ', ' Acc Y:'))];
            accY = [accY str2double(extractBetween(line, 'Acc Y: ', ' Acc Z:'))];
            accZ = [accZ str2double(extractBetween(line, 'Acc Z: ', ' Gyro X:'))];

            % Extract gyroscope values
            gyroX = [gyroX str2double(extractBetween(line, 'Gyro X: ', ' Gyro Y:'))];
            gyroY = [gyroY str2double(extractBetween(line, 'Gyro Y: ', ' Gyro Z:'))];
            gyroZ = [gyroZ str2double(extractBetween(line, 'Gyro Z: ', ' Time:'))];

            % Extract time values
            time = [time str2double(extractAfter(line, 'Time: '))/1e6];
        end
    end
end
