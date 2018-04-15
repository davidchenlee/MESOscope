# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Implement the required pixel clock for uniform spatial sampling (Mathematica notebook)
- Implement the FIFO reading and data saving concurrently
- Save the run status and error messages in a text file
- Save the run parameters in a text file
- Maybe install TVK to display tiff images
- Allow reading multiple frames in the FPGA-to-PC FIFO
- Flush the FPGA-to-PC FIFO before starting the code
- Optimize the data-transfer memory usage


### LabView
- Remove some elements in the front panel
- Flush the buffers
- Create a ramp generator. This is only necessary if I want to increase the memory of the FIFO OUT at the expense of the other buffers
- Maybe decrease the clock of the photon-counter from 160 MHz to 80 MHz for a faster/easier compilation
- For debugging purposes, it would be nice to have an input selector on the photon-counter to choose from the PMT or the PMT simulator
- Check the delay of the internal FIFOs implemented in memory blocks
- When the FIFO OUT depth is set to 131071 elements, on the VS side the max # of readable elements is 160000
  - Read the FIFO in VS using the 'NiFpga_AcquireFifoReadElementsU32' function for higher bandwidth. The idea is to make the host FIFO extra large and read from it directly. This only works if the FIFO can be read and write simultaneosly
  - Try to increase the FIFO depth (I tried already. 131071 elements is the max for USB-7856R)
  - Cascade FIFOs. I could compile an additional internal FIFO with 65545 elements, 64 bits
- I detected that, when the FPGA resets at the end of the code, the shutter #2 switches. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter
- The 'FIFO OUT' subvi has a feedback node that I could possibly get rid of because in the 'photon count' subvi the reset is placed after the counter. Try placing it BEFORE the counter.
