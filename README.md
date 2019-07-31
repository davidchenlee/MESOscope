# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Check the power increase
- Check the sequencer
- Implement findSampleContour()
- Multi-thread demuxAllChannels_()
- Fix how to deal with multithread exception in the Filterwheel class
- Do a post-sequence clean up routine to set the pockels outputs to 0
- For the vibratome, show a progress bar for the slicing sequence
- Maybe install VTK to display tiff images

### LabView
- I detected that, when the FPGA resets at the end of the code, the shutter #2 is triggered. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter

### Hardware
- Install both Uniblitz shutters
- Test the emission aperture
- Screw down the PMT16X
- Screw down the exc filterwheel
- Test cooling down the PMT16X