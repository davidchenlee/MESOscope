run("Image Sequence...", "open=[Z:/Output_Z/*.tif]");
run("Z Project...", "projection=[Average Intensity]");
selectWindow("Output_Z");
close();
