# GPS package

## Description
Provides location messages in ROS from the Altix GPS using the Alink USB to serial converter.

## Hardware dependencies
* Alink USB to serial converter
* Aerobtec GPS module
* micro USB cable 

| USB-converter/Alink | GPS    | Description                     |
|---------------------|--------|---------------------------------|
| yellow              | yellow | Data (only data from gps to pc) |
| red                 | orange | +5V                             |
| black               | brown  | GND-0V                          |


## Software dependencies
* `nmea_navsat_driver` package

## Setup
1. Check if the GPS module is connected properly
    * `$ lsusb` output:
    ```
    Bus 001 Device 009: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
    ```
    * `$ dmesg` output:
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
2. Install dependencies
    ```
    $ sudo chmod +x gps_install.sh
    $ ./gps_install.sh
    ```

## Usage
* Launch using launchfile
    ```
    $ roslaunch gps gps.launch
    ```  
* The GPS position data is published on the `/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)) topic
