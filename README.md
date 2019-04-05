# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- For scaling the pockels power, currently only linear scaling for voltage is implemented. Implement linear scaling for the laser power as well (because the laser power depends nonlinearly of the voltage). 
- Fix how to deal with multithread exception in the Filterwheel class!!!!
- Do a post-sequence clean up routine to set the pockels outputs to 0
- For the vibratome, show a progress bar for the slicing sequence
- Maybe implement an automatic laser power increase depending on the imaging depth
- Maybe install VTK to display tiff images

### LabView
- Create a ramp generator. This is only necessary if I want to increase the memory of FIFOOUT at the expense of the input buffers
- Check the delay of the internal FIFOs implemented in memory blocks
- When FIFOOUT depth is set to 131071 elements, on the VS side the max # of readable elements is 160000
  - Read FIFO VS using the 'NiFpga_AcquireFifoReadElementsU32' function for higher bandwidth. The idea is to make the host FIFO extra large and read from it directly. This only works if the FIFO can be read and write simultaneosly
  - Try to increase FIFOout depth (I tried already. 131071 elements is the max for USB-7856R)
  - Cascade FIFOs. I could compile an additional internal FIFO with 65545 elements, 64 bits
- I detected that, when the FPGA resets at the end of the code, the shutter #2 is triggered. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter
