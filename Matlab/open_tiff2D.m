close all

HEATMAP = 1;
COUNT_FREQ = 1;

scale = 1;
tiffHandle = Tiff('..\Maincode\Images\photoncount.tif');
%tiffHandle = Tiff('D:\OwnCloud\Data\20180425\damaging test slide.tif');
inputImage = read(tiffHandle);
%image(inputImage)

%Recover the photon counts from the tiff image
counts =  inputImage(:,:,1)/scale;
maxcount = double(max(max(counts)));
notBlank = maxcount ~= 0;

if notBlank
    %Plot a heat map
    if HEATMAP && notBlank
        hm = HeatMap(flip(counts,1), 'Colormap','redbluecmap'); % Flip the image for consistency with ImageJ
        close all hidden
        ax = hm.plot; % 'ax' will be a handle to a standard MATLAB axes.
        colorbar('Peer', ax); % Turn the colorbar on
        caxis(ax, [0, max(10,maxcount)]); % Adjust the color limits
    end
    
    
    %Tabulate the count frequency
    if COUNT_FREQ && notBlank
        %print out a table with the frequency of each photon-count
        [Ncounts, edges] = histcounts(counts);
        photonNumber = edges-0.5;
        photonNumber = photonNumber(2:length(photonNumber)); %get rid of the first element
        table = [photonNumber; Ncounts]';
        
        colNames = {'count','frequency'};
        array2table(table,'VariableNames',colNames)
    end
else
    disp('ERROR: Blank image')
end
close(tiffHandle);


