clear all
%% fuel consumption map
wEngMap=[1000 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 4000]*2*pi/60;  % (rad/s), speed range of the engine
lbft2Nm=1.356; %conversion from lbft to Nm
tEngMap=[6.3 12.5 18.8 25.1 31.3 37.6 43.9 50.1 56.4 62.7 68.9 75.2]*lbft2Nm;  % (N*m), torque range of the engine

% (g/s), fuel use map indexed vertically by enginemap_spd and horizontally by enginemap_trq
fuelConsMap = [
 0.1513  0.1984  0.2455  0.2925  0.3396  0.3867  0.4338  0.4808  0.5279  0.5279  0.5279  0.5279 
 0.1834  0.2423  0.3011  0.3599  0.4188  0.4776  0.5365  0.5953  0.6541  0.6689  0.6689  0.6689 
 0.2145  0.2851  0.3557  0.4263  0.4969  0.5675  0.6381  0.7087  0.7793  0.8146  0.8146  0.8146 
 0.2451  0.3274  0.4098  0.4922  0.5746  0.6570  0.7393  0.8217  0.9041  0.9659  0.9659  0.9659 
 0.2759  0.3700  0.4642  0.5583  0.6525  0.7466  0.8408  0.9349  1.0291  1.1232  1.1232  1.1232 
 0.3076  0.4135  0.5194  0.6253  0.7312  0.8371  0.9430  1.0490  1.1549  1.2608  1.2873  1.2873 
 0.3407  0.4584  0.5761  0.6937  0.8114  0.9291  1.0468  1.1645  1.2822  1.3998  1.4587  1.4587 
 0.3773  0.5068  0.6362  0.7657  0.8951  1.0246  1.1540  1.2835  1.4129  1.5424  1.6395  1.6395 
 0.4200  0.5612  0.7024  0.8436  0.9849  1.1261  1.2673  1.4085  1.5497  1.6910  1.8322  1.8322 
 0.4701  0.6231  0.7761  0.9290  1.0820  1.2350  1.3880  1.5410  1.6940  1.8470  1.9999  2.0382 
 0.5290  0.6938  0.8585  1.0233  1.1880  1.3528  1.5175  1.6823  1.8470  2.0118  2.1766  2.2589 
 0.6789  0.8672  1.0555  1.2438  1.4321  1.6204  1.8087  1.9970  2.1852  2.3735  2.5618  2.7501 ];

% Draw Max Tq Line
tEngMaxMap = [tEngMap(1) 77.2920 82.0380 84.7500 86.7840 89.3604 91.1232 92.8860 94.6488 96.4116 98.1744 99.9372 101.9712];
wEngMaxMap = [1000 1010 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 4000];
% Max Tq Line Data Resampling
wEngMax = 1000:1:4000;
tEngMax = interp1(wEngMaxMap,tEngMaxMap,wEngMax);
% from RPM to rad/s
wEngMax = wEngMax*2*pi/60;
%% Toyota Prius hybrid parameters              
Jec = (0.178+0.835+0.0062+0.0127); %kg*m^2
Jgs = 0.023; %kg*m^2
Jmr = 0.023; %kg*m^2
Kratio = 4.113;
Mv = 1400; %kg
Rtire = 0.3107; %meter
Atire = 2.33; %m^2
Cd = 0.26;
rou= 1.202;
ftire = 0.00475;
g = 9.8; %N*m/sec
Jv = Mv*Rtire^2;
Voc = 201.6; %volt
Qbatt = 6.5*3600; % ampere*sec
Rbatt = 0.003*6*28;  % ohm
nm = 0.85;
ng = 0.85;
Hl = 42*1e6; % 42MJ/kg
phi =0;
S = 30;
R = 78;
K = 4.113;
MPH_2_KMPH = 1.60934;
TRAN_EFF = 0.9; % assume transmission efficiency is 90%
MU_TIRE = 0.7; % coeff of friction between tire and pavement 
%% get the max acceleration from engine power
AVEH_MAX = MU_TIRE*g; % assume 4WD 
WDOT_VEH_MAX = AVEH_MAX/Rtire; 
AVEH_MAX_MPH = AVEH_MAX*3600/1000/MPH_2_KMPH;

% Max and min engine power
PENG_MIN = tEngMap(1)*wEngMap(1);
PENG_MAX = tEngMap(end)*wEngMap(end);

% range of vehicle angular velocity and angular acceleration
wVehSpan = linspace(0, 60, 1000)'*MPH_2_KMPH*1000/3600/Rtire; % [rad/s]
wDotVehSpan = linspace(0, AVEH_MAX_MPH, 1000)'*MPH_2_KMPH*1000/3600/Rtire; % [rad/s^2]

% initialize array for storing max and min wDotVeh
wDotVehMaxSpan = nan(numel(wVehSpan), 1);
wDotVehMinSpan = nan(numel(wVehSpan), 1);
for iWVeh = 1:numel(wVehSpan)
    thisWVeh = wVehSpan(iWVeh);
    if thisWVeh < 0.02
        thisWVeh = 0.02;
    end
    WDOT_MAX_IND = nan;
    % power request to overcome tire rolling friction and grade (grade
    % angle = 0 here)
    pTireGrade = (ftire*Mv*g*cos(phi) + 0.5*rou*Cd*Atire*Rtire^2*thisWVeh^2)*Rtire*thisWVeh/TRAN_EFF;
    wDotVehMaxSpan(iWVeh) = min((PENG_MAX - pTireGrade)/(Mv*Rtire*Rtire*thisWVeh/TRAN_EFF), AVEH_MAX/Rtire);
    wDotVehMinSpan(iWVeh) = min((PENG_MIN - pTireGrade)/(Mv*Rtire*Rtire*thisWVeh/TRAN_EFF), AVEH_MAX/Rtire);
end
vVehSpanKmph = wVehSpan*Rtire*3600/1000; % [kMph]
aVehMaxSpan = wDotVehMaxSpan*Rtire;
aVehMinSpan = wDotVehMinSpan*Rtire;
% figure;
% plot(vVehSpanKmph, [aVehMaxSpan, aVehMinSpan])
% xlabel('vVeh [kMph]')
% ylabel('aVeh [m/s^2]')
% ylim([0, AVEH_MAX*1.1])
%% fuel consumption mapping calculation 
% define how many points for the maping
W_LIST_LEN = 100;
WDOT_LIST_LEN = 200;

% initialize the array for vehicle angular velocity 
wVehList = linspace(0, 60, W_LIST_LEN)'*MPH_2_KMPH*1000/3600/Rtire; % [rad/s]

% initialize the array for engine speed
wEngList = linspace(1000, 4000, 40)'*2*pi/60; % [rad/s]
fprintf('---------------------------------------------------------\n')

% initialize data store arrays
wVehPlot = nan(WDOT_LIST_LEN, W_LIST_LEN);
wDotVehPlot = nan(WDOT_LIST_LEN, W_LIST_LEN);
fuelConsPlot = nan(WDOT_LIST_LEN, W_LIST_LEN);
pVVehPlot = nan(WDOT_LIST_LEN, W_LIST_LEN);
for iW = 1:numel(wVehList)       
    % get the max and min vehicle angular acceleration for current vehicle
    % angular velocity. The wDotVeh cannot be larger than the maximum
    % acceleration that the engine can generate under current velocity.
    wDotVehMaxCur = interp1(wVehSpan, wDotVehMaxSpan, wVehList(iW), 'pchip');
    wDotVehMinCur = interp1(wVehSpan, wDotVehMinSpan, wVehList(iW), 'pchip');
    wDotVehList = linspace(-WDOT_VEH_MAX, wDotVehMaxCur, WDOT_LIST_LEN); %  full map
%     wDotVehList = linspace(0, wDotVehMaxCur, WDOT_LIST_LEN); % only positive map
    for iWDot = 1:numel(wDotVehList)
        % get current wVeh and current wDotVeh
        wVeh = wVehList(iW);
        wDotVeh = wDotVehList(iWDot);
        
        % save current wVeh and wDotVeh
        wVehPlot(iWDot, iW) = wVeh;
        wDotVehPlot(iWDot, iW) = wDotVeh;
        
        % this is to avoid 0 power request
        if wVeh < 0.02
            wVeh = 0.02;
        end
        
        % calculate vehicle request torque. TRAN_EFF to ensure that it
        % requires more torque to consider the transmission efficiency
        tVeh = (ftire*Mv*g*cos(phi) + Mv*g*sin(phi) + 0.5*rou*Cd*Atire*Rtire^2*wVeh^2 + Mv*wDotVeh*Rtire)*Rtire*1/TRAN_EFF;
        
        % display current wVeh and current wDotVeh
        fprintf('wVeh %d, wDotVeh %d, pReq %8.4f\n',...
            iW, iWDot, tVeh*wVeh)
        pVVehPlot(iWDot, iW) = tVeh*wVeh;
        % if power request is smaller than the minimum engine output power,
        % this means that the brake must kicks in. so assume the fuel
        % consumption rate is the fuel cons of min engine operating point
        if wDotVeh < wDotVehMinCur
            fuelConsPlot(iWDot, iW) = fuelConsMap(1);
        else
            % otherwise, search through engine speed to find the constant
            % power curve
            fuelConsEachOp = nan(numel(wEngList), 1);
            tEngList = nan(numel(wEngList), 1);
            for iWEng = 1:numel(wEngList)
                wEng = wEngList(iWEng);
                tEng = tVeh*wVeh/wEng;

                % save constant power curve
                tEngList(iWEng) = tEng;
                fuelConsEachOp(iWEng) = interp2(tEngMap, wEngMap, ...
                        fuelConsMap, tEng, wEng, 'spline');
            end
            
            % refine the data to find the valid portion of constant power
            % curve that is below max power curve
            tEngResamp = interp1(wEngList,tEngList,wEngMax);
            wEngResamp = wEngMax;
            
            % get logical index of the valid portion of constant power
            % curve
            isValid = (tEngMax - tEngResamp) > 0 | abs(tEngMax - tEngResamp) < 1e-5;
            
            % if all tEng is larger than MaxTq, this is sth we hope to
            % avoid
            if isempty(isValid(isValid ~= 0)) 
                [minTqDiff, isValid] = min(abs(tEngMax - tEngResamp));
                minTqDiff
%                 if (minTqDiff > 1)
%                     debug = 1;
%                 end
            end
            
            % get the fuel consumption of the valid portion of constant
            % power curve
            fuelConsResamp = interp2(tEngMap, wEngMap, ...
                fuelConsMap, tEngResamp(isValid), wEngResamp(isValid), 'spline');
%             if isempty(fuelConsResamp)
%                 debug = 1;
%             end
            
            % find the minimum fuel consumption in the valid portion of
            % constant power curve
            fuelConsPlot(iWDot, iW) = nanmin(fuelConsResamp);

%             if iW == 1 && iWDot == 1
%                 debug = 1;
%                 figure; hold on
%                 plot(wEngMax, tEngMax, 'g')
%                 plot(wEngResamp, tEngResamp, 'b')
%             end
        end
    end
end

%% Plotting
vVehPlotMph = wVehPlot*Rtire*3600/1000/MPH_2_KMPH;
aVehPlot = wDotVehPlot*Rtire;
figure;
surf(vVehPlotMph, aVehPlot, fuelConsPlot)
xlabel('vehicle speed [mph]')
ylabel('Vehicle Acceleration [m/s^2]')
zlabel('fuel consumption [g/s]')

% % save('fuelConsMap_fullAVeh_150629', 'vVehPlotMph', 'aVehPlot', 'fuelConsPlot', 'wDotVehMaxSpan', 'wDotVehMinSpan')
save('fuelConsMap_fullAVeh_150701', 'vVehPlotMph', 'aVehPlot', ...
    'fuelConsPlot', 'wDotVehMaxSpan', 'wDotVehMinSpan', 'wVehSpan', ...
    'Rtire', 'MPH_2_KMPH', 'pVVehPlot')