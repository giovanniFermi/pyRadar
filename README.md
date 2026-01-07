# ADC/UART Data Capturing using xWR1843/AWR2243 with DCA1000

* For xWR1843: Capture both raw ADC IQ data and processed UART point cloud data simultaneously in Python and C (pybind11) without mmWave Studio
* For AWR2243: Capture raw ADC IQ data in Python and C (pybind11) without mmWave Studio


## Introduction

* This module consists of two main parts: mmwave and fpga_udp:
  * mmwave is modified from [OpenRadar](https://github.com/PreSenseRadar/OpenRadar), used for configuration file reading, serial port data transmission and reception, raw data parsing, etc.
  * fpga_udp is modified from [pybind11 example](https://github.com/pybind/python_example) and [mmWave-DFP-2G](https://www.ti.com/tool/MMWAVE-DFP), used for receiving high-speed raw data from DCA1000 via Ethernet using C socket code. For models like AWR2243 that lack on-chip DSP and ARM cores, it also implements firmware writing and parameter configuration operations by sending commands via USB using FTDI to control AWR2243 through SPI.

* TI's mmWave radars are mainly divided into two categories: those with only RF front-end and those with on-chip ARM and DSP/HWA. The former includes models like [AWR1243](https://www.ti.com/product/AWR1243) and [AWR2243](https://www.ti.com/product/AWR2243), while the latter includes models like [xWR1443](https://www.ti.com/product/IWR1443), [xWR6443](https://www.ti.com/product/IWR6443), [xWR1843](https://www.ti.com/product/IWR1843), [xWR6843](https://www.ti.com/product/IWR6843), [AWR2944](https://www.ti.com/product/AWR2944), etc.
  * For radar sensors with only RF front-end, control and configuration commands are typically sent via SPI/I2C interface, and raw data is output through CSI2/LVDS interface. The SPI interface can be converted to USB protocol using the FTDI chip on the DCA1000 board for direct computer control, and LVDS interface data can also be captured by the FPGA on the DCA1000 board and transmitted as UDP packets over Ethernet. This repository implements all the above operations.
  * For radar sensors with on-chip ARM and DSP, control programs can be flashed to configure the radar sensor using the on-chip ARM, process raw data using the on-chip DSP to obtain point cloud data, etc., and transmit via UART. In addition to feeding raw data to the on-chip DSP, it can also be configured to output via LVDS interface and captured by the FPGA on the DCA1000 board. This repository implements all the above operations. Of course, radar sensors with on-chip ARM and DSP also have SPI/I2C interfaces that can be used to configure the radar sensor, and the DCA1000's onboard FTDI can convert SPI to USB for computer control. [mmWave Studio](https://www.ti.com/tool/MMWAVE-STUDIO) works this way, but this repository has not yet implemented this method. Refer to [mmWave-DFP](https://www.ti.com/tool/MMWAVE-DFP) for self-implementation.


## Prerequisites

### Hardware
#### For xWR1843
* Connect the micro-USB port (UART) on the xWR1843 to your system
* Connect the xWR1843 to a 5V barrel jack
* Set power connector on the DCA1000 to RADAR_5V_IN
* Boot in Functional Mode: SOP[2:0]=001
  * Either place jumpers on pins marked as SOP0 or toggle SOP0 switches to ON, all others remain OFF
* Connect the RJ45 to your system
* Set a fixed IP to the local interface: 192.168.33.30

#### For AWR2243
* Connect the micro-USB port (FTDI) on the DCA1000 to your system
* Connect the AWR2243 to a 5V barrel jack
* Set power connector on the DCA1000 to RADAR_5V_IN
* Put the device in SOP0
  * Jumper on SOP0, all others disconnected
* Connect the RJ45 to your system
* Set a fixed IP to the local interface: 192.168.33.30

### Software
#### Windows
 - Microsoft Visual C++ 14.0 or greater is required.
   - Get it with "[Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)" (Standalone MSVC compiler) or "[Visual Studio](https://visualstudio.microsoft.com/downloads/)" (IDE) and choose "Desktop development with C++"
 - FTDI D2XX driver and DLL is needed.
   - Download version [2.12.36.4](https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.36.4%20WHQL%20Certified.zip) or newer from [official website](https://ftdichip.com/drivers/d2xx-drivers/).
   - Unzip it and install `.\ftdibus.inf` by right-clicking this file.
   - Copy `.\amd64\ftd2xx64.dll` to `C:\Windows\System32\` and rename it to `ftd2xx.dll`. For 32-bit systems, just copy `.\i386\ftd2xx.dll` to that directory.

#### Linux
 - `sudo apt install python3-dev`
 - FTDI D2XX driver and .so lib is needed. Download version 1.4.27 or newer from [official website](https://ftdichip.com/drivers/d2xx-drivers/) based on your architecture, e.g. [X86](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_32-1.4.27.tgz), [X64](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_64-1.4.27.tgz), [armv7](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-arm-v7-hf-1.4.27.tgz), [aarch64](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-arm-v8-1.4.27.tgz), etc.
 - Then you'll need to install the library:
   - ```
     tar -xzvf libftd2xx-arm-v8-1.4.27.tgz
     cd release
     sudo cp ftd2xx.h /usr/local/include
     sudo cp WinTypes.h /usr/local/include
     cd build
     sudo cp libftd2xx.so.1.4.27 /usr/local/lib
     sudo chmod 0755 /usr/local/lib/libftd2xx.so.1.4.27
     sudo ln -sf /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so
     sudo ldconfig -v
     ```


## Installation

 - Clone this repository
 - For Windows:
   - `python3 -m pip install --upgrade pip`
   - `python3 -m pip install --upgrade setuptools`
   - `python3 -m pip install ./fpga_udp`
 - For Linux:
   - `sudo python3 -m pip install --upgrade pip`
   - `sudo python3 -m pip install --upgrade setuptools`
   - `sudo python3 -m pip install ./fpga_udp`


## Instructions for Use

#### General
1. First set up the runtime environment according to [Prerequisites](#prerequisites)
2. Then install the library according to [Installation](#installation)
3. If unmentioned modules report errors during runtime, please search and add them yourself

#### For xWR1843
1. Flash the xwr18xx_mmw_demo program according to the [mmWave SDK](https://www.ti.com/tool/MMWAVE-SDK) instructions
2. Use [mmWave_Demo_Visualizer](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/) to adjust parameters and save the cfg configuration file
3. Open [captureAll.py](#captureallpy), modify as needed, fill in the cfg configuration file address and port number, then run to start data collection
4. Open [testDecode.ipynb](#testdecodeipynb) or [testDecodeADCdata.mlx](#testdecodeadcdatamlx) to parse the collected data
5. If unsatisfied with parameters, continue using [mmWave_Demo_Visualizer](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/) to adjust, or use [testParam.ipynb](#testparamipynb) to modify and verify parameter validity

#### For AWR2243
1. Flash firmware patch to external flash (this operation only needs to be performed once; firmware is retained after restart)
   - For Windows: `python3 -c "import fpga_udp;fpga_udp.AWR2243_firmwareDownload()"`
   - For Linux: `sudo python3 -c "import fpga_udp;fpga_udp.AWR2243_firmwareDownload()"`
   - If you see "MSS Patch version [ 2. 2. 2. 0]", flashing was successful
2. Open [captureADC_AWR2243.py](#captureadc_awr2243py), modify as needed, fill in the txt configuration file address, and start data collection
3. Open [testDecode_AWR2243.ipynb](#testdecode_awr2243ipynb) to parse the collected data
4. If unsatisfied with parameters, use [testParam_AWR2243.ipynb](#testparam_awr2243ipynb) to modify and verify parameter validity


## Example

### ***captureAll.py***
Example code for simultaneously capturing raw ADC sampled IQ data and pre-processed point cloud serial data from on-chip DSP (xWR1843 only).

#### 1. General Workflow for Capturing Raw Data
 1. Reset radar and DCA1000 (reset_radar, reset_fpga)
 2. Initialize radar via UART and configure corresponding parameters (TI, setFrameCfg)
 3. (Optional) Create process for receiving data processed by on-chip DSP from serial port (create_read_process)
 4. Send FPGA configuration command via Ethernet UDP (config_fpga)
 5. Send record data packet configuration command via Ethernet UDP (config_record)
 6. (Optional) Start serial reception process (only clears buffer) (start_read_process)
 7. Send start capture command via Ethernet UDP (stream_start)
 8. Start UDP packet reception thread (fastRead_in_Cpp_async_start)
 9. Start radar via serial port (theoretically can also be controlled via FTDI (USB to SPI), currently only implemented on AWR2243) (startSensor)
 10. Wait for UDP packet reception thread to finish + parse raw data (fastRead_in_Cpp_async_wait)
 11. Save raw data to file for offline processing (tofile)
 12. (Optional) Send stop capture command via Ethernet UDP (stream_stop)
 13. Stop radar via serial port (stopSensor) or send radar reset command via Ethernet (reset_radar)
 14. (Optional) Stop receiving serial data (stop_read_process)
 15. (Optional) Parse point cloud and other on-chip DSP processed data received from serial port (post_process_data_buf)

#### 2. "*.cfg" mmWave Radar Configuration File Requirements
 - Default profile in Visualizer disables LVDS streaming.
 - To enable it, please export the chosen profile and set the appropriate enable bits.
 - adcbufCfg needs to be set as follows, and the third parameter of lvdsStreamCfg needs to be set to 1. See mmwave_sdk_user_guide.pdf for details.
    - adcbufCfg -1 0 1 1 1
    - lvdsStreamCfg -1 0 1 0

#### 3. "cf.json" Data Acquisition Card Configuration File Requirements
 - For detailed information, please refer to TI_DCA1000EVM_CLI_Software_UserGuide.pdf
 - LVDS Mode:
    - LVDS mode specifies the lane config for LVDS. This field is valid only when dataTransferMode is "LVDSCapture".
    - The valid options are:
    - • 1 (4lane)
    - • 2 (2lane)
 - Packet Delay:
    - In default conditions, Ethernet throughput varies up to 325 Mbps speed in a 25-µs Ethernet packet delay.
    - The user can change the Ethernet packet delay from 5 µs to 500 µs to achieve different throughputs.
       - "packetDelay_us":  5 (us)   ~   706 (Mbps)
       - "packetDelay_us": 10 (us)   ~   545 (Mbps)
       - "packetDelay_us": 25 (us)   ~   325 (Mbps)
       - "packetDelay_us": 50 (us)   ~   193 (Mbps)

### ***captureADC_AWR2243.py***
Example code for capturing raw ADC sampled IQ data (AWR2243 only).

#### 1. General Workflow for AWR2243 Raw Data Capture
 1. Reset radar and DCA1000 (reset_radar, reset_fpga)
 2. Initialize radar via SPI and configure corresponding parameters (AWR2243_init, AWR2243_setFrameCfg) (requires root privileges on Linux)
 3. Send FPGA configuration command via Ethernet UDP (config_fpga)
 4. Send record data packet configuration command via Ethernet UDP (config_record)
 5. Send start capture command via Ethernet UDP (stream_start)
 6. Start UDP packet reception thread (fastRead_in_Cpp_async_start)
 7. Start radar via SPI (AWR2243_sensorStart)
 8. Either:
    1. (Optional, required if numFrame==0) Stop radar via SPI (AWR2243_sensorStop)
    2. (Optional, not allowed if numFrame==0) Wait for radar capture to complete (AWR2243_waitSensorStop)
 9. (Optional, required if numFrame==0) Send stop capture command via Ethernet UDP (stream_stop)
 10. Wait for UDP packet reception thread to finish + parse raw data (fastRead_in_Cpp_async_wait)
 11. Save raw data to file for offline processing (tofile)
 12. Power off radar and close configuration file via SPI (AWR2243_poweroff)

#### 2. "mmwaveconfig.txt" mmWave Radar Configuration File Requirements
 - TBD

#### 3. "cf.json" Data Acquisition Card Configuration File Requirements
 - Same as above

### ***realTimeProc.py***
Example code for real-time cyclic capture of raw ADC sampled IQ data with online processing (xWR1843 only).

#### 1. General Workflow for Capturing Raw Data
 1. Reset radar and DCA1000 (reset_radar, reset_fpga)
 2. Initialize radar via UART and configure corresponding parameters (TI, setFrameCfg)
 3. Send FPGA configuration command via Ethernet UDP (config_fpga)
 4. Send record data packet configuration command via Ethernet UDP (config_record)
 5. Send start capture command via Ethernet UDP (stream_start)
 6. Start radar via UART (theoretically can also be controlled via FTDI (USB to SPI), currently only implemented on AWR2243) (startSensor)
 7. **Cyclically** receive UDP packets + parse raw data + real-time data processing (fastRead_in_Cpp, postProc)
 8. Stop radar via UART (stopSensor)
 9. Send stop capture command via Ethernet UDP (fastRead_in_Cpp_thread_stop, stream_stop)

#### 2. "mmwaveconfig.txt" mmWave Radar Configuration File Requirements
 - Omitted

#### 3. "cf.json" Data Acquisition Card Configuration File Requirements
 - Omitted

### ***realTimeProc_AWR2243.py***
Example code for real-time cyclic capture of raw ADC sampled IQ data with online processing (AWR2243 only).

#### 1. General Workflow for AWR2243 Raw Data Capture
 - Omitted

#### 2. "mmwaveconfig.txt" mmWave Radar Configuration File Requirements
 - TBD

#### 3. "cf.json" Data Acquisition Card Configuration File Requirements
 - Omitted

### ***testDecode.ipynb***
Example code for parsing raw ADC sampled data and serial data (IWR1843 only). Open with Jupyter (VS Code with Jupyter extension recommended).

#### 1. Parsing LVDS-Received ADC Raw IQ Data
##### Parsing LVDS-received ADC raw IQ data using numpy
 - Load related libraries
 - Set corresponding parameters
 - Load saved bin data and parse
 - Plot time-domain IQ waveform
 - Compute Range-FFT
 - Compute Doppler-FFT
 - Compute Azimuth-FFT

##### Parsing LVDS-received ADC raw IQ data using functions provided by mmwave.dsp

#### 2. Parsing UART-Received On-Chip DSP Processed Point Cloud, Doppler, and Other Data
 - Load related libraries
 - Load saved serial parsed data
 - Display data configured in cfg file
 - Display on-chip processing time (can be used to determine if frame rate adjustment is needed)
 - Display temperature of each antenna
 - Display data packet information
 - Display point cloud data
 - Calculate range labels and Doppler velocity labels
 - Display range profile and noise floor profile
 - Display Doppler Bins
 - Display Azimuth (Angle) Bins

### ***testDecode_AWR2243.ipynb***
Example code for parsing raw ADC sampled data (AWR2243 only). Open with Jupyter (VS Code with Jupyter extension recommended).

#### 1. Parsing LVDS-Received ADC Raw IQ Data
##### Parsing LVDS-received ADC raw IQ data using numpy
 - Load related libraries
 - Set corresponding parameters
 - Load saved bin data and parse
 - Plot time-domain IQ waveform
 - Compute Range-FFT
 - Compute Doppler-FFT
 - Compute Azimuth-FFT

### ***testParam.ipynb***
IWR1843 mmWave radar configuration parameter validity verification. Open with Jupyter (VS Code with Jupyter extension recommended).
 - Mainly verifies whether the cfg file required by the mmWave radar and the cf.json file required by the DCA acquisition board are correctly configured.
 - Parameter constraints come from the device characteristics of IWR1843 itself. Please refer to IWR1843 datasheet, mmWave SDK user guide, and chirp programming guide for details.
 - If parameters satisfy constraints, debug information will be output in cyan; if not satisfied, output will be in purple or yellow.
 - Note that the constraints in this program are not completely accurate, so in special cases, even if all parameters satisfy constraints, there is still a chance of abnormal operation.

### ***testParam_AWR2243.ipynb***
Same as above, parameter validity verification for AWR2243 mmWave radar configuration.

### ***testDecodeADCdata.mlx***
MATLAB example code for parsing raw ADC sampled data
 - Set corresponding parameters
 - Load saved bin raw ADC data
 - Parse and reconstruct data format according to parameters
 - Plot time-domain IQ waveform
 - Compute Range-FFT (1D FFT + static clutter filtering)
 - Compute Doppler-FFT
 - 1D-CA-CFAR Detector on Range-FFT
 - Compute Azimuth-FFT

### ***testGtrack.py***
Test C-language gTrack algorithm using cppyy. This algorithm is TI's group target tracking algorithm, which takes point cloud as input and outputs trajectories.

The algorithm is designed to track multiple targets, where each target is represented by a set of measurement points.
Each measurement point carries detection information, for example, range, azimuth, elevation (for 3D option), and radial velocity.

Instead of tracking individual reflections, the algorithm predicts and updates the location and dispersion properties of the group.

The group is defined as the set of measurements (typically, a few tens; sometimes a few hundreds) associated with a real-life target.

Algorithm supports tracking targets in two or three dimensional spaces as a build time option:
 - When built with 2D option, algorithm inputs range/azimuth/doppler information and tracks targets in 2D Cartesian space.
 - When built with 3D option, algorithm inputs range/azimuth/elevation/doppler information and tracks targets in 3D Cartesian space.

#### Input/Output
 - Algorithm inputs the Point Cloud. For example, a few hundreds of individual measurements (reflection points).
 - Algorithm outputs a Target List. For example, an array of a few tens of target descriptors. Each descriptor carries a set of properties for a given target.
 - Algorithm optionally outputs a Target Index. If requested, this is an array of target IDs associated with each measurement.

#### Features
 - Algorithm uses extended Kalman Filter to model target motion in Cartesian coordinates.
 - Algorithm supports constant velocity and constant acceleration models.
 - Algorithm uses 3D/4D Mahalanobis distances as gating function and Max-likelihood criteria as scoring function to associate points with an existing track.


## Software Architecture

### TBD.py
TBD
