# HotWaterController
 Controller for solar hot water system with recirc pump.
 Uses a PyPortal Titano board and FreeRTOS.
 
## Operation

### Startup
 - Connects to Wifi and an MQTT channel on a ThingsBoard instance.

### Inputs
 - Reads two analog sensors via a PCF8591 board, ColdSensor and HotSensor. These were the original system's sensors.
 - Reads five Dallas OneWire temperature sensors - Ambient, Tank Inlet Temp, Tank Outlet Temp, Panel Inlet Temp, Panel Outlet Temp. These are currently only used for monitoring.
 
### Processing
 - Temperatures are read every two seconds.
 - Pump speed is calculated every two seconds.
 - If HotSensor - ColdSensor > 8, begin operating the circulating pump.
 - Pump runs on a 5 second duty cycle, and is PWM'd according to the temperature difference between the hot and cold sensors. 8 degree difference or higher, gives 100 percent output. 4 degrees difference or lower gives 0 percent output.
 - The temperature difference automatically adjusts from it's initial 8 degree starting point depending on the pump speed.
 - If the pump runs at 100 percent, the difference is slowly increased from 8 degrees, multiplying by 1.01 every calculation cycle. This throttles back the pump and increases the temperature difference across the solar collectors for maximum temperature gain.
 - Between 100 and 30 percent the pump is driven proportionally according to the temperature difference.
 - If the pump runs below 30 percent the difference is decreased by 0.98 every calculation cycle. This increases the pump speed so that residence time in the pipework from the collectors is minimised (too much cool off in the pipes if the flow is too low.)
 - Overall this should give a reasonably adaptive system that tries to maximise solar gain and minimise loss in pipework.
 - If the duty cycle is above 90 or below 10, round to 100 or 0 respectively to avoid short pulses on the pump.
 - Once the temperature difference drops below 4 degrees the pump is shut off.
 - If the ColdSensor temperature is above 70 degrees, trigger a cool-off cycle. When the HotSensor is 30 degrees below the ColdSensor (eg late afternoon or night), turn on the pump and circulate hot water from the tank through the collectors until the ColdSensor temp drops below 60 degrees. This helps prevent over temperature events and loss of hot water via the pressure relief valve on the tank.
 - If the collector temperature is greater than 100 degrees, turn on the pump regardless of the temperature difference between hot and cold sensors.
 
 
### Outputs
 - Drives a small circulating pump via the PCF8591 DAC, which controls a zero-crossing IC that drives a SCR for the pump. The pump speed value from 0 - 100 percent is converted to a 5 second on-off duty cycle for the pump.
 - Updates the PyPortal Titano display with basic information.
 
### Monitoring
 - Sends data via MQTT to a ThingsBoard instance for display. The ThingsBoard instance also sends alert emails if the tank temperature drops below a lwoer threshold. Setting up a ThingsBoard or MQTT server is left as an exercise for the user.
