clear all;
clc;
close all;

COM = 'COM3';
BAUD = 9600;
NUM_MEASUREMENTS = 256;
FPS = 10;
Fs = 1e6;

% device = serialport(COM,BAUD);
% configureTerminator(device, "LF");

dataStore = struct('ID', {}, 'Value', {});

t = timer;
t.Period = 1/FPS;          % Timer period in seconds
t.ExecutionMode = 'fixedRate'; % Execute at fixed intervals
t.TimerFcn = @reconCallback;  % Function to call when timer triggers
start(t);

% Create figure for live plotting
figure;
hold on; % Hold on to plot multiple lines
grid on;
xlabel('Index');
ylabel('Magnitude');
title('Live Measurement Plot');
xlim([0 NUM_MEASUREMENTS]); % Set x-axis limits
measurementsPlot = plot(nan, nan, 'o-'); % Initialize a plot object with empty data

while true
    dataLine = readline(serialObj);
    parsedData = sscanf(dataLine, 'A%uV%uC%u %u'); %A = which ADC, V =  voltage mux, C = current mux
    A = parsedData(1);
    V = parsedData(2) + 1;
    C = parsedData(3) + 1;
    adcVal = parsedData(4);

    firstElec = 4*(A-1) + V;
    secondElec = (firstElec < 16) * (firstElec + 1) + ~(firstElec < 16) * (1);
    
    firstStim = C + 1;
    secondStim = (C < 16) * (C + 2) + ~(C < 16) * (1);
    
    if firstElec ~= firstStim && secondElec ~= secondStim
        uniqueID = sprintf('Voltage%d:%d|Current%d:%d', firstElec,secondElec, firstStim,secondStim);
        idx = find(strcmp({dataBuffers.ID}, uniqueID));
        if isempty(idx)
            dataStore(end + 1) = struct('ID', uniqueID, 'Values', adcVal);
        else
            dataStore(idx).Values(end + 1) = adcVal;
        end
    end    
end

stop(t);    
delete(t);

function reconCallback(~, ~)
    if length(dataStore) == NUM_MEASUREMENTS
        measurements = zeros(NUM_MEASUREMENTS);
        [~, sortIdx] = sort({dataStore.ID}); 
        dataStore = dataStore(sortIdx); %might need better sorting 

        for i = 1:length(NUM_MEASUREMENTS)
            measurements(i) = calculateMagnitude(dataStore(i).Values);
        end
    
        for i = 1:length(dataStore)
            dataStore(i).Values = []; % Clear the Values for each ID
        end

        plotMeasurements(measurements);

    end
end


function magnitude = calculateMagnitude(buffer)
    buffer = buffer(:); 
    N = length(buffer); % Number of samples
    Y = fft(buffer);

    P2 = abs(Y / N); % Two-sided spectrum
    P1 = P2(1:N/2+1); % Single-sided spectrum
    P1(2:end-1) = 2 * P1(2:end-1); % Amplitude correction

    % Frequency vector
    f = Fs * (0:(N/2)) / N;

    % Find the frequency with the maximum amplitude
    [~, idx] = max(P1);
    magnitude = P1(idx); % Magnitude of the sine wave
end

function plotMeasurements(measurements)
    % Update plot data
    global measurementsPlot; % Access the global plot variable
    measurementsPlot.XData = 1:length(measurements); % Set X data to indices
    measurementsPlot.YData = measurements; % Set Y data to measurements

    % Adjust axes
    ylim([0 max(measurements) * 1.1]); % Set Y-axis limits
    drawnow; % Update the figure window
end
