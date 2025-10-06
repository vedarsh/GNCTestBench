# GNC TestBench

A rudimentary GNC testbench made from the sensors and controller i've hoarded from years.

## Introduction

This repository is a codebase for my GNC testbench setup to experiment for Satellite and Drone GNC Controls. Main purpose being the validation and testing of sensors and controller in a resource constrained environment.
Consists of a QMC5883L, BME688, DS3231 and a NEO M8N module.

## Components

- QMC5883L - A low budget, decently accurate magnetometer
- BME688   - A very powerful Temperature, Humidity and pressure sensor that can also do VoC and poisonous gas detection
- DS3231   - RTC clock for time keeping and to sync timing
- NEOM8N   - A Very high end GPS module for location and heading data. Can do SBAS too. Coupled with an active antenna.

- Pico W.  - Dual Core ARM Cortex M0+ controller with good IO and WiFi capabilities.

## Implementation

The Distribution of processing is spread across two cores. The first core helps in the collection and logging of data. The second core is permanently set to process the data from the first core.

### First Core Workflow

The workflow is as follows.

1. The first core initialises an I2C port and an UART port. UART port is coupled with an UART interrupt. UART Interrupt triggers when the RX receives the data from GPS Module. 

2. UART Interrupt checks for the prefix on the data being received. The interrupt scans for the **$** sign. Once received, it scans for the NMEA text code. Current flow uses the **GPRMC**, **GPGSV** and **GPVTG** tokens.
    - The interrupt takes this string until completion and stores it in a buffer for further processing

3. The Sensors initialise and check if the sensors are in place. It errors out otherwise.

4. The loop is scheduled as a round robin scheduling to ensure the sensors that are ready can provide the latest data and the ones that are not still get polled.
    - The Loop starts with GPS. when the interrupt flag for new data ready is set, the NMEA Parser in NEOM8N directory copies  the samples of data releases the flag
        - The copied data gets processed to get the information on **Latitude**, **Longitude**, **UTC Time**, **Altitude** and other key data.
        - The processed data gets pushed to a telemetry structure.

    - The loop moves on to the Magnetometer and logs the fresh data as the sensor says its data is ready. Preserves the last data if not.

    - The loop moves on to RTC to fetch the timestamp for the data. this gets the respective timestamp for the telemetry packet.

    - The loop moves on to the IMU **TODO** once the ordered IMU arrives.

    - The last DEV None is the placeholder for the THP sensor.

5. **TODO** Serialisation for the data telemetry packet to send to the other core/ other controller to process.

### Second Core Workflow

1. Deserialise the packet received through FIFO.

2. **TODO**

### TODO

1. Hard Iron calibration of the QMC5883L.
2. Serialisation of the packet.
3. Deserialisation of the packet in the second core.



The main MCU is Pico.
