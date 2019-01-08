# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Make Tiff correction (mirror odd frames, average frames, etc) with multithread
- save Tiff and move stages concurrently
- Maybe install VTK to display tiff images

### LabView
- Create a ramp generator. This is only necessary if I want to increase the memory of FIFOOUT at the expense of the input buffers
- Maybe decrease the clock of the photon-counter from 160 MHz to 80 MHz for a faster/easier compilation
- Check the delay of the internal FIFOs implemented in memory blocks
- When FIFOOUT depth is set to 131071 elements, on the VS side the max # of readable elements is 160000
  - Read FIFO VS using the 'NiFpga_AcquireFifoReadElementsU32' function for higher bandwidth. The idea is to make the host FIFO extra large and read from it directly. This only works if the FIFO can be read and write simultaneosly
  - Try to increase FIFOout depth (I tried already. 131071 elements is the max for USB-7856R)
  - Cascade FIFOs. I could compile an additional internal FIFO with 65545 elements, 64 bits
- I detected that, when the FPGA resets at the end of the code, the shutter #2 is triggered. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter
