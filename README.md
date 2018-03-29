# Dscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Save the scanned image as tiff. Optimize the memory usage
- Implement the FIFO reading and data saving concurrently
- Save the run status in a text file
- Maybe install openCV to display tiff images


### LabView
- Maybe decrease the clock of the photon-counter from 160 MHz to 80 MHz for a faster/easier compilation
- For debugging purposes, It would be nice to have an input selector on the photon-counter to choose from the PMT or the PMT simulator
- change the internal FIFOs to memory blocks to allow multiple iterations