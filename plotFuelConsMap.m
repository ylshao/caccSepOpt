load fuelConsMap_fullAVeh_150629
% load fuelConsMap_posAVeh_150629


%% plot fuel consumption map
figure;
surf(vVehPlotMph, aVehPlot, fuelConsPlot)
xlabel('vehicle speed [mph]')
ylabel('vehicle acceleration [m/s^2]')
zlabel('fuel consumption [g/s]')

%% plot max acceleration w.r.t. velocity
vVehSpanKmph = wVehSpan*Rtire*3600/1000; % [kMph]
aVehMaxSpan = wDotVehMaxSpan*Rtire;

figure;
plot(vVehSpanKmph, aVehMaxSpan)
xlabel('vehicle velocity [kMph]')
ylabel('max acceleration [m/s^2]')
ylim([0, max(aVehMaxSpan)*1.1])