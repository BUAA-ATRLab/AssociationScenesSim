function generateMeasurements(settings)
numTrajectory=settings.iMisNum;
numSumWarhead=settings.iSumGroup;
numSumHeavyDecoy=settings.iSumHeavyDecoy;
numSumLightDecoy=settings.iSumLightDecoy;
numSumDebris=settings.iSumDebris;
numSumPieces=settings.iNumPieces3;
numSumTarget=numSumWarhead+numSumPieces+sum(numSumHeavyDecoy)+sum(numSumLightDecoy)+sum(numSumDebris);

radar_noise_r = settings.radar_noise_r;
radar_noise_theta = settings.radar_noise_theta;
radar_noise_phi = settings.radar_noise_phi;
telescope_noise_theta = settings.telescope_noise_theta;
telescope_noise_phi = settings.telescope_noise_phi;

filename = ['.\Final\truth',num2str(numTrajectory),'.mat'];
load(filename);

meas_radar = cell(1,numSumTarget);
meas_telescope = cell(1,numSumTarget);
for i = 1:numSumTarget
    X = dataRadar{i}(:,1);
    Y = dataRadar{i}(:,2);
    Z = dataRadar{i}(:,3);
    r = sqrt(X.^2+Y.^2+Z.^2);
    theta = acos(Z./r);
    phi = atan(Y./X);
    
    %Radar Measurements
    meas_radar_r = r + normrnd(0,radar_noise_r,size(r,1),1);
    meas_radar_theta = theta + normrnd(0,radar_noise_theta,size(theta,1),1);
    meas_radar_phi= phi + normrnd(0,radar_noise_phi,size(phi,1),1);
    meas_radar{i} = [meas_radar_r,meas_radar_theta,meas_radar_phi];
    
    %Infrared Measurements
    meas_telescope_theta = theta + normrnd(0,telescope_noise_theta,size(theta,1),1);
    meas_telescope_phi = phi + normrnd(0,telescope_noise_phi,size(phi,1),1);
    meas_telescope{i} = [meas_telescope_theta,meas_telescope_phi];
end

filename = ['.\Final\meas',num2str(numTrajectory),'.mat'];
save(filename,'meas_radar','meas_telescope');
