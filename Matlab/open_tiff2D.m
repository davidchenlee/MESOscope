inputImage = imread('..\Maincode\_photon-counts.tif');
image(inputImage)

[counts,binLocations] = imhist(inputImage(:,:,1));



