
load fuelConsMap_posAVeh_150629
vVehMphFit = reshape(vVehPlotMph, [], 1);
aVehFit = reshape(aVehPlot, [], 1);
fuelConsFit = reshape(fuelConsPlot, [], 1);
fitFcn = fit([vVehMphFit, aVehFit], fuelConsFit, 'poly32');

%% plot fuel consumption map
figure;
h = surf(vVehPlotMph, aVehPlot, fuelConsPlot);
set(h, 'LineStyle', 'none')
xlabel('vehicle speed [mph]')
ylabel('vehicle acceleration [m/s^2]')
zlabel('fuel consumption [g/s]')


%%
VVEH_IND = 30;
AVEH_IND = 150;
selVVehMph = vVehPlotMph(1, VVEH_IND);
selWVeh = selVVehMph*MPH_2_KMPH*1000/3600/Rtire;
selAVeh = aVehPlot(AVEH_IND, VVEH_IND);
selWDotVeh = selAVeh/Rtire;
selTVeh = (ftire*Mv*g*cos(phi) + 0.5*rou*Cd*Atire*Rtire^2*selWVeh^2 + Mv*selWDotVeh*Rtire)*Rtire*1/TRAN_EFF;
constPower = selTVeh*selWVeh;
wVehArray = vVehPlotMph(1, :)'*MPH_2_KMPH*1000/3600/Rtire;
wDotVehArray = nan(numel(wVehArray), 1);
for iWVeh = 1:numel(wVehArray)
    thisWVeh = wVehArray(iWVeh);
    thisWDotVeh = (constPower - (ftire*Mv*g*cos(phi) + ...
        0.5*rou*Cd*Atire*Rtire^2*thisWVeh^2)*Rtire*1/TRAN_EFF*thisWVeh)/...
        (Mv*Rtire*Rtire*1/TRAN_EFF*thisWVeh);
    wDotVehArray(iWVeh) = min(thisWDotVeh, AVEH_MAX/Rtire);
end

vVehMphArray = wVehArray*Rtire*3600/1000/MPH_2_KMPH;
aVehArray = wDotVehArray*Rtire;


fuelConsArray = fitFcn(vVehMphArray, aVehArray);

% 
% thisVehPower = nan(size(aVehPlot, 1), 1);
% for iAVeh = 1:size(aVehPlot, 1)
%     thisWDotVeh = aVehPlot(iAVeh, 50);
%     
% end
figure;
scatter3(vVehMphArray, aVehArray, fuelConsArray)
%% plot max acceleration w.r.t. velocity
vVehSpanKmph = wVehSpan*Rtire*3600/1000; % [kMph]
aVehMaxSpan = wDotVehMaxSpan*Rtire;

figure;
plot(vVehSpanKmph, aVehMaxSpan)
xlabel('vehicle velocity [kMph]')
ylabel('max acceleration [m/s^2]')
ylim([0, max(aVehMaxSpan)*1.1])