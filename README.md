# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
Caution:
- In Routines::Sequencer, when calling Boolmap boolmap{ panoramicScan,...) and the used RAM reaches ~ 1GB, the memory allocation through new in TiffU8::TiffU8() throws an error.
I think this has to do with the 2GB memory limitation in x86. When saving the panoramic scan, Tifflib was throwing "No Space for output buffer" error. Maybe this is related issue. Setting /LARGEADDRESSAWARE on VS solved the problem
- There is still a memory leak, 1.5 GB after running Routines::Sequence() for 4 hours
- In Mesoscope::openShutter(), I commented out VirtualLaser::isLaserInternalShutterOpen(), which checks that the laser shutter is open, because it is not needed when running the full sequence and any comm error will stop the routine
- Keep an eye on the Z-stage bouncing that triggers the ctl&acq
Sequencer:
- enable/disable using the vibratome in Routines::sequencer
Post-processing
- Implement suppressCrosstalk() flattenField() on the GPU
- Multithread demuxAllChannels_()
Others:
- Do a post-sequence clean up routine to set the pockels outputs to 0
- Maybe switch to smart pointers for the data. Check the overhead
- For the vibratome, show a progress bar for the slicing sequence
- Maybe install VTK to display tiff images

### LabView
- I detected that shutter #2 is triggered when the FPGA resets at the end of the code. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter

### Hardware


### Things that could be improved in the future
- Compile in 64 bits
- Mount the collector lens on the detector platform
- Use a motorized stage for the PMT16X mount
- Use an apodizing filter to make the fluorescence emission even for the different beamlets
- Use a grid mask on the PMT16X to reduce the crosstalk
- Use motorized actuator on the mirrors that align the Ti-Sapphire laser to the microscope because the laser beam slightly wanders with the wavelength tuning
- Set up the beads next to the sample holder permanently
- Purchase a 5W power pockels (instead of 10W) for Fidelity to have a finer control
- Mount the bead and fluorescent slides in the sample container for checking the microscope performance regularly
- Improve the dichroic mount to allow fine rotation
- The oil in the sample container crawls up through the clamps
- Add motorized actuators to the last corner mirror before the galvo scanner
- When I ran the full Routines::sequencer() for a whole liver lobe (~ 19 hours), the RAM usage grouws from ~150 MB to ~2GB. I think the leak is in the panoramic views