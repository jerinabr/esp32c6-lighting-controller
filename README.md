# Overview
Wi-Fi enabled CCT LED strip controller that receives commands over MQTT.

Brightness and color temperature interpolation is supported and the firmware supports "interrupting" ongoing interpolation if a new LED state request is received mid-interpolation.

This firmware is intended for the XIAO ESP32C6 but there's no reason it can't be modified and used with any ESP32* hardware.

# Connectivity
The device connects to a Wi-Fi network which is specified by the configuration. Eventually I might add thread support.

# MQTT
The device subscribes to the topic "kitchen/under-cabinet-light/cmd" and it parses any messages from this topic. The message should be JSON formatted. Here's an example of a message:
```json
{
    "power": 1,
    "brightness": 255,
    "temp": 3000
}
```
## JSON fields
### power
> Type: Integer
> 
0 (Off) or 1 (On)

### brightness
> Type: Integer
> 
0 - 255 where 0 is off and 255 is max brightness

### temp
> Type: Integer
> 
Color temperature in Kelvin
