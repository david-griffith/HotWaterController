# HotWaterController [![Actions Status](https://github.com/david-griffith/HotWaterController/actions/workflows/main.yml/badge.svg)](https://github.com/david-griffith/HotWaterController/actions/)
 This repository contains a controller for a solar hot water system.
 
 The system has a ground-mounted storage tank and a circulating water pump controlling flow to solar collectors on the roof.
 Pump drive is proportional to the temperature difference between the water at the bottom of the storage tank and the temperature at the top of the solar collectors.
 The control alogrithm that is used should give a reasonably adaptive system that tries to maximise the temperature gain across the collectors, while also minimising the temperature loss in the pipework between the collector and storage tank.
 
 The controller hardware is based on a PyPortal Titano board. It uses FreeRTOS/Arduino for the software, with a generic PCF8591 I/O board for analog sensing and pump motor control. Dallas OneWire temperature sensors are used to augment the original system's hot and cold NTC sensors. Data from all the sensors is packaged up and sent to a ThingsBoard instance for remote monitoring.
 
 Circuit diagrams and PCB files for the control board are also included. The board houses the generic PCF8591 board that is used to read analog NTC thermistors and allow control of a 240V pump via a zero-crossing IC and a SCR. 
 
## Operation
After the intial startup, a number of FreeRTOS tasks look after various aspects of the system. Unlike "normal Arduino" with simple setup and loop functions, FreeRTOS tasks (standalone functions with their own internal loops) all run concurrently in a time-sliced manner.
Because any task can be interrupted in the middle of something, access to shared resources are managed via mutexes - locks that only allow one task at a time to access that resource. In this code there are mutexes around shared hardware (I2C and serial) as well as a mutex for the recirculating pump speed variable to ensure that it's state does not unexpectedly change during calculations.

### Startup
 - Initialise the hardware.
 - Start the FreeRTOS tasks. A stack monitoring thread runs in the idle task and prints to the serial port.

### Inputs
 - Reads two analog NTC sensors via a PCF8591 board, ColdSensor and HotSensor. These were the original system's sensors. A conversion function with a lookup table is used to get degrees C. Colder temperatures (below 20 degrees or so) are at the upper range of the 8 bit ADC, where due to the non-linear operation of the NTC thermistors each integer step gives a proportionally larger temperature value. As a result of this, low temperature values are a little noisy, so these raw values are smoothed with a simple averaging function.
 - Reads five Dallas OneWire temperature sensors - Ambient, Tank Inlet Temp, Tank Outlet Temp, Panel Inlet Temp, Panel Outlet Temp. These are currently only used for monitoring.
 
### Processing
 - Temperatures are read every two seconds in one task.
 - Pump speed is calculated every two seconds in another task.
 - If HotSensor - ColdSensor > 8, begin operating the circulating pump.
 - The pump runs on a 5 second duty cycle, and is PWM'd according to the temperature difference between the hot and cold sensors. 8 degree difference or higher, gives 100 percent output. 4 degrees difference or lower gives 0 percent output.
 - The temperature difference automatically adjusts from it's initial 8 degree starting point depending on the pump speed. If the pump runs at 100 percent, the difference is slowly increased from 8 degrees, multiplying by 1.01 every calculation cycle. This throttles back the pump and increases the temperature difference across the solar collectors for maximum temperature gain.
 - Between 100 and 40 percent the pump is driven proportionally according to the temperature difference.
 - If the pump runs below 40 percent the temperature difference is decreased by 0.98 every calculation cycle. This increases the pump speed so that residence time in the pipework from the collectors is minimised (too much cool off in the pipes if the flow is too low.)
 - If the duty cycle is above 90 or below 10, round to 100 or 0 respectively to avoid short pulses on the pump.
 - Once the temperature difference drops below 4 degrees the pump is shut off.
 - If the ColdSensor temperature is above 70 degrees, trigger a cool-off cycle. When the HotSensor is 30 degrees below the ColdSensor (eg late afternoon or night), turn on the pump and circulate hot water from the tank through the collectors until the ColdSensor temp drops below 60 degrees. This helps prevent over temperature events and loss of hot water via the pressure relief valve on the tank.
 - If the collector temperature is greater than 100 degrees, turn on the pump regardless of the temperature difference between hot and cold sensors.
 
 
### Outputs
 - An output task drives a small circulating pump via the PCF8591 DAC, which controls a zero-crossing IC that drives a SCR for the pump. The pump can circulate approximately 3 litres a minute through the solar collectors and draws 20 watts at 240VAC. The pump speed value from 0 - 100 percent is converted to a 5 second on-off duty cycle for this pump.
 - A display task updates the PyPortal Titano display with basic information on sensors and pump drive, as well as RSSI and the total number of wifi/MQTT connects since power on.
 
### Monitoring
 - A task connects to wifi and a Thingsboard instance via MQTT and maintains that connection.
 - It then updates the Thingsboard instance with the sensor values, pump drive, current RSSI, and the number of total wifi and MQTT connects since power on.
 - The ThingsBoard instance also sends alert emails if the tank temperature drops below a lower threshold. Setting up a ThingsBoard or MQTT server is left as an exercise for the user.
