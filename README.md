# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Save the scanned image as tiff. Scale the count to [0,255]. Optimize the transfer-memory usage
- Implement the FIFO reading and data saving concurrently
- Save the run status and error messages in a text file
- Save the run parameters in a text file
- Maybe install openCV to display tiff images


### LabView
- Maybe decrease the clock of the photon-counter from 160 MHz to 80 MHz for a faster/easier compilation
- For debugging purposes, it would be nice to have an input selector on the photon-counter to choose from the PMT or the PMT simulator
- Change the internal FIFOs to memory blocks to allow multiple iterations
- When cycling the galvo ramp, implement the posibility to mirror the sequence so that the galvo can swing up and down continuously