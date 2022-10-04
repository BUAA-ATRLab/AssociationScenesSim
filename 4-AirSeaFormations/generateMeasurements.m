function generateMeasurements(settings)

radar_noise_r = settings.radar_noise_r;
radar_noise_theta = settings.radar_noise_theta;
radar_noise_phi = settings.radar_noise_phi;

telescope_noise_theta = settings.telescope_noise_theta;
telescope_noise_phi = settings.telescope_noise_phi;
numSumTarget = settings.num_plane + settings.num_boat;

load('.\data\truth.mat');
meas_radar = cell(1,numSumTarget);
meas_telescope = cell(1,numSumTarget);

for i = 1:numSumTarget
    if i == 1
        X = xPlane1;Y = yPlane1;Z = zPlane1;
    elseif i == 2
        X = xPlane2;Y = yPlane2;Z = zPlane2;
    elseif i == 3
        X = xPlane3;Y = yPlane3;Z = zPlane3;
    elseif i == 4
        X = xPlane4;Y = yPlane4;Z = zPlane4;
    elseif i == 5
        X = xPlane5;Y = yPlane5;Z = zPlane5;
    elseif i == 6
        X = xBoat1;Y = yBoat1;Z = zBoat1;
    elseif i == 7
        X = xBoat2;Y = yBoat2;Z = zBoat2;
    else
        X = xBoat3;Y = yBoat3;Z = zBoat3;
    end
    
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
save('.\data\meas.mat','meas_radar','meas_telescope');

