# GPS Processing package

## Description
ROS wrapper for the Aerobtec GPS module. 
* Processes GPS time and global position data.
* Manages procedure for the robot's global orientation estimation.
* Provides service to compute the angular position of the sun.

## Software dependencies
* ROS packages
    - [`nmea_navsat_driver`](http://wiki.ros.org/nmea_navsat_driver)
    - [`std_msgs`](http://wiki.ros.org/std_msgs)
    - [`sprinter_srvs`](../sprinter_srvs/README.md)
* Python modules
    - `numpy`
    - `suncalc`
    - `datetime`
    - `collections`

## ROS Interface
* Published topics
    - `/tf_static` [[tf2_msgs/TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)]: `world`->`odom` static 2D rotation
    - `/gps/global_orientation/done` [[std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)]: feedback indicating completition of the procedure
    - `/target/pose` [[geometry_msgs/Pose2D](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose2D.html)]: command to change the robot's position during the procedure
* Subscribed topics
    - `/gps/global_orientation/do` [[std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]: triggers the procedure
    - `/target/pose/reached` [[std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)]: Pose Control node feedback
    - `/gps/fix` [[sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)]: GPS position data
    - `/gps/time_ref` [[sensor_msgs/TimeReference](http://docs.ros.org/en/api/sensor_msgs/html/msg/TimeReference.html)]: GPS time data
* Provided services
    - `/gps/get_sun_orientation` [[sprinter_srvs/GetOrientation](../sprinter_srvs/srv/GetOrientation.srv)]: returns orientation to the sun in the `world` frame

## Hardware dependencies
* Alink USB to serial converter
* Aerobtec GPS module
* micro USB cable 

| USB-converter/Alink | GPS    | Description                     |
|---------------------|--------|---------------------------------|
| yellow              | yellow | Data (only data from gps to pc) |
| red                 | orange | +5V                             |
| black               | brown  | GND-0V                          |

## Setup
1. Check if the GPS module is connected properly
*
``` bash
 `$ lsusb`
```
Output:
```
Bus 001 Device 009: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
```
*
``` bash
$ dmesg 
```
Output:
```
[170819.217804] usb 3-2: new full-speed USB device number 8 using xhci_hcd
[170819.396570] usb 3-2: New USB device found, idVendor=0403, idProduct=6001, bcdDevice= 6.00
[170819.396581] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[170819.396584] usb 3-2: Product: FT232R USB UART
[170819.396587] usb 3-2: Manufacturer: FTDI
[170819.396589] usb 3-2: SerialNumber: AEAQ01R71F
[170819.448897] usbcore: registered new interface driver usbserial_generic
[170819.448904] usbserial: USB Serial support registered for generic
[170819.450736] usbcore: registered new interface driver ftdi_sio
[170819.450743] usbserial: USB Serial support registered for FTDI USB Serial Device
[170819.450787] ftdi_sio 3-2:1.0: FTDI USB Serial Device converter detected
[170819.450809] usb 3-2: Detected FT232RL
[170819.454989] usb 3-2: FTDI USB Serial Device converter now attached to ttyUSB0
[170827.650486] usb 3-2: USB disconnect, device number 8
[170827.650886] ftdi_sio ttyUSB0: FTDI USB Serial Device converter now disconnected from ttyUSB0
```
2. Build
``` bash
$ catkin build gps_processing
```

## Usage
* Launch using launchfile
``` bash
$ roslaunch gps_processing gps_processing.launch
```  