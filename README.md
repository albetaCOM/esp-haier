 # ESP Haier
 

 ESP Haier is a project to use a ESP8266 (I did not test with ESP32) to control Haier Air Conditioner (firmware version R_1.0.00/e_2.5.14) with wifi module supoort with ESPHome and Home Assisant.
 

 You only need configure esphaier.yml, put Haier.h or HaierWithFan.h in the same directory and flash. 
 
 Home Assisant will recognize as climate device.

 Added support on HaierWithFan.h for fan, dry and swing. For that ESPHome v1.15.0b1 or higher is needed. In home assistant->Supervisor->ESPHome Addon->Configurator add: esphome_version: v1.15.0b4 
 
 This project is based in the works of Instalator and his project [Haier Wifi](https://github.com/instalator/Haier_WiFi/)
 
 
 I use a Wemos D1 Mini with a "hacked" USB cable. I cut the USB cable and connect directly to Wemos:
 
 - Red -> 5V 
 - Black -> GND 
 - Green -> RX 
 - White -> TX
 
  # Known issues
 
- ~~The current temperature is not well reported. It always report 28ºC~~ -> Thanks to @instalator this issue is solved.
- ~~Pending to be able to parse the mode correctly:
     - ~~Cool mode is not correctly parsed.
     - ~~Auto mode is not correctly parsed.

  # Pending work
- Create a custom switch in yaml file in order to select dry function
