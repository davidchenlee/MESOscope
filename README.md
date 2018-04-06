# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Save the scanned image as tiff. Scale the count to [0,255]. Optimize the transfer-memory usage
- Calculate the required pixel clock for uniform spatial sampling (Mathematica notebook)
- Implement the FIFO reading and data saving concurrently
- Save the run status and error messages in a text file
- Save the run parameters in a text file
- Maybe install openCV to display tiff images
- Allow reading multiple frames in the FPGA-to-PC FIFO
- Flush the FPGA-to-PC FIFO before starting the code


### LabView
- Create a ramp generator. This is only necessary if I want to increase the memory of the FIFO OUT at the expense of the other buffers
- Solve the problem about the pretrigger and the non-zero wait time after the pixel clock loop
- Maybe decrease the clock of the photon-counter from 160 MHz to 80 MHz for a faster/easier compilation
- For debugging purposes, it would be nice to have an input selector on the photon-counter to choose from the PMT or the PMT simulator
- Check the delay of the internal FIFOs implemented in memory blocks
- Implement the posibility of mirroring the sequence so that the galvo can swing up and down continuously
- When the FIFO OUT depth is set to 131071 elements, on the VS side the max # of readable elements is 160000. Try to increase the FIFO depth. If not possible, maybe try to concatenate 2 FIFOs in LV
- I detected that, when the FPGA resets at the end of the code, the shutter #2 switches. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter
