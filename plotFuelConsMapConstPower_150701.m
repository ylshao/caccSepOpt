load fuelConsMap_fullAVeh_150701

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
ftire = 0.00475; % rolling friction coeff
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
FUELCONS_MIN = 0.1513; % fuel consumption of the engine @ minimal power output
%% plot power VS fuel consumption 
% reshape saved data as arrays
pVVehArray = reshape(pVVehPlot, [], 1);
fuelConsArray = reshape(fuelConsPlot, [], 1);
vVehPlot = vVehPlotMph*MPH_2_KMPH*1000/3600; % [m/s]
vVehArray = reshape(vVehPlot, [], 1);
aVehArray = reshape(aVehPlot, [], 1);
% only fit power request larger than the minimal engine output power, these
% data points will have fuel consumption larger than 0.1513
isValid = fuelConsArray > FUELCONS_MIN;
pVVehPowerPosArray = pVVehArray(isValid);
fuelConsPowerPosArray = fuelConsArray(isValid);
vVehPowerPosArray = vVehArray(isValid);
aVehPowerPosArray = aVehArray(isValid);
dataNum = numel(pVVehPowerPosArray);
% plot power VS fuel consumption
figure;
plot(pVVehPowerPosArray/10^3, fuelConsPowerPosArray, '.')
xlabel('vehilce power request [kWatts]')
ylabel('fuel consumption [g/s]')

% save('fuelConsMap_fullAVeh_150701', 'pVVehArray', 'fuelConsArray', 'pVVehPosArray', 'fuelConsPPosArray', '-append')
%% curve fitting

%--------------------------------------------------------------------------
% linear model
%--------------------------------------------------------------------------
% here just try a linear fitting
linearFitFcn = fit(pVVehPowerPosArray, fuelConsPowerPosArray, 'poly1');
linearFitCoeff = coeffvalues(linearFitFcn);

% get the fuelCons using linear method
p10 = ((ftire*Mv*g*cos(phi)+ Mv*g*sin(phi))*1/TRAN_EFF)*linearFitCoeff(1);
p11 = Mv*1/TRAN_EFF*linearFitCoeff(1);
p30 = 0.5*rou*Cd*Atire*1/TRAN_EFF*linearFitCoeff(1);
p00 = linearFitCoeff(2);

linearFitFcn = @(v, a) p00 + p10*v + p11*v.*a + p30*v.^3;

fuelConsArrayLinearFit = linearFitFcn(vVehPowerPosArray, aVehPowerPosArray);

%--------------------------------------------------------------------------
% psd model
%--------------------------------------------------------------------------
% fuelCons = a1*pVeh + a2*v^2 + a3*v*a^2 + a4
dataMat = [pVVehPowerPosArray vVehPowerPosArray.^2 ...
    vVehPowerPosArray.*aVehPowerPosArray.^2 ones(dataNum, 1)];
fitCoeff = dataMat\fuelConsPowerPosArray;

% final coefficients
p00 = fitCoeff(4);
p10 = ((ftire*Mv*g*cos(phi)+ Mv*g*sin(phi))*1/TRAN_EFF)*fitCoeff(1);
p20 = fitCoeff(2);
p11 = Mv*1/TRAN_EFF*fitCoeff(1);
p30 = 0.5*rou*Cd*Atire*1/TRAN_EFF*fitCoeff(1);
p12 = fitCoeff(3);

% fprintf('p00 %8.4e, p10 %8.4e, p20 %8.4e, p11 %8.4e\n', p00, p10, p20, p11)
% fprintf('p30 %8.4e, p12 %8.4e\n', p30, p12)

% get the fuelCons using vsp method
vspFitFcn = @(v, a) p00 + p10*v + p20*v.^2 + p11*v.*a + ...
    p30*v.^3 + p12*v.*a.^2;

fuelConsArrayVspFit = vspFitFcn(vVehPowerPosArray, aVehPowerPosArray);

%--------------------------------------------------------------------------
% poly32 direct fitting
%--------------------------------------------------------------------------
% get the fuelCons using direct poly32 method
vVehPosArray = reshape(vVehPosPlot, [], 1);
aVehPosArray = reshape(aVehPosPlot, [], 1);
fuelConsPosArray = reshape(fuelConsPosPlot, [], 1);
dirFitFcn = fit([vVehPosArray, aVehPosArray], fuelConsPosArray, 'poly55');
fuelConsArrayDirFit = dirFitFcn(vVehPowerPosArray, aVehPowerPosArray);

%% see the fitting goodness
% compare fuelCons with surface plot
figure; hold on
h = surf(vVehPlot, aVehPlot, fuelConsPlot);
set(h, 'LineStyle', 'none')
scatter3(vVehPowerPosArray, aVehPowerPosArray, fuelConsArrayVspFit)
xlabel('vehicle speed [m/s]')
ylabel('Vehicle Acceleration [m/s^2]')
zlabel('fuel consumption [g/s]')
view(3)

% compare the fuelCons between the two methods
figure; hold on
plot(fuelConsArrayDirFit, 'b.')
plot(fuelConsArrayVspFit, 'r.')
plot(fuelConsArrayLinearFit, 'c.')
legend('dir', 'vsp', 'lin')
xlabel('data point')
ylabel('fuel consumption [g/s]')

MAX_VSP_ERR = max(abs(fuelConsArrayVspFit - fuelConsArrayDirFit));
MAX_LIN_ERR = max(abs(fuelConsArrayLinearFit - fuelConsArrayDirFit));
fprintf('max vsp error %8.4f, max linear error %8.4f\n', MAX_VSP_ERR, MAX_LIN_ERR)

RMS_VSP = sum((fuelConsArrayVspFit - fuelConsArrayDirFit).^2)/dataNum;
RMS_LIN = sum((fuelConsArrayLinearFit - fuelConsArrayDirFit).^2)/dataNum;
fprintf('rms vsp %8.4e, rms linear %8.4e\n', RMS_VSP, RMS_LIN)

%%
% calculate the final coeff as polynomial of vehicle velocity and accel
%   relation velVeh = omegaVeh*Rtire, accelVeh = omegaDotVeh*Rtire;
%
%   fuelCons = k*pVeh + b, where powerVeh = torqVeh*omegaVeh;
%
%   F_friction = ftire*Mv*g*cos(phi);
%   F_gradeAngle = Mv*g*sin(phi);
%   F_wind = 0.5*rou*Cd*Atire*velVeh^2;
%   F_inertial = Mv*accelVeh;
%
%   torqVeh = (F_friction + F_gradeAngle + F_wind + F_inertial)*omegaVeh;
%
%   Here use TRAN_EFF to consider the efficient of engine