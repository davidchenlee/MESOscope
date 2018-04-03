%clear all
PLOT_VECTOR = 1;
HEATMAP = ~PLOT_VECTOR;
COUNT_FREQ = ~PLOT_VECTOR;


%Plot the data as a vector
vector = importdata('.\Maincode\_photon-counts.txt');

if PLOT_VECTOR
    plot(vector)
end

%Plot a heat map
if HEATMAP   
    %convert the 1D data into a 2D array
    array2D = zeros(400, 400); %preallocate an array
    for kk = 1:400
        if mod(kk,2)
            array2D(:,kk) = vector(((kk-1)*400)+(1:400));
        else
            %the scanned direction is reversed for every other line
            array2D(:,kk) = flipud( vector( ((kk-1)*400)+(1:400) ) );
        end
    end
    
    
    hm = HeatMap(array2D, 'Colormap','redbluecmap');
    
    close all hidden
    ax = hm.plot; % 'ax' will be a handle to a standard MATLAB axes.
    colorbar('Peer', ax); % Turn the colorbar on
    caxis(ax, [0 10]); % Adjust the color limits
end

%Tabulate the count frequency
if COUNT_FREQ
    %print out a table with the frequency of each photon-count
    Ncounts = histcounts(vector);
    Nmax = numel(Ncounts);
    table = [0:Nmax-1; Ncounts]';
    
    colNames = {'count','frequency'};
    array2table(table,'VariableNames',colNames)
end