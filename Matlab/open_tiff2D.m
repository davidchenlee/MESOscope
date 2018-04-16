HEATMAP = 1;
COUNT_FREQ = 1;

scale = 25.5;
inputImage = imread('..\Maincode\_photon-counts.tif');
%image(inputImage)

%Recover the photon counts from the tiff image
PhotonCounts =  inputImage(:,:,1)/scale;

%Plot a heat map
if HEATMAP
    hm = HeatMap(PhotonCounts, 'Colormap','redbluecmap');
    close all hidden
    ax = hm.plot; % 'ax' will be a handle to a standard MATLAB axes.
    colorbar('Peer', ax); % Turn the colorbar on
    caxis(ax, [0 10]); % Adjust the color limits
end


%Tabulate the count frequency
if COUNT_FREQ
    %print out a table with the frequency of each photon-count
    [Ncounts, edges] = histcounts(PhotonCounts);
    photonNumber = edges-0.5;
    photonNumber = photonNumber(2:length(photonNumber)); %get rid of the first element
    table = [photonNumber; Ncounts]';
    
    colNames = {'count','frequency'};
    array2table(table,'VariableNames',colNames)
end



