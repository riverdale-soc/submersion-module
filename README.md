| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# Submersion Module
* Start WiFi.
* Initialize ESPNOW.
* Register ESPNOW sending or receiving callback function.
* Add ESPNOW peer information.
* Send and receive ESPNOW data.

  

This needs at least two ESP devices:

* In order to get the MAC address of the other device, Device1 firstly send broadcast ESPNOW data with 'state' set as 0.
* When Device2 receiving broadcast ESPNOW data from Device1 with 'state' as 0, adds Device1 into the peer list.
  Then start sending broadcast ESPNOW data with 'state' set as 1.
* When Device1 receiving broadcast ESPNOW data with 'state' as 1, compares the local magic number with that in the data.
  If the local one is bigger than that one, stop sending broadcast ESPNOW data and starts sending unicast ESPNOW data to Device2.
* If Device2 receives unicast ESPNOW data, also stop sending broadcast ESPNOW data.

In practice, if the MAC address of the other device is known, it's not required to send/receive broadcast ESPNOW data first,
just add the device into the peer list and send/receive unicast ESPNOW data.

There are a lot of "extras" on top of ESPNOW data, such as type, state, sequence number, CRC and magic in this. These "extras" are
not required to use ESPNOW. They are only used to make this to run correctly. However, it is recommended that users add some "extras"
to make ESPNOW data more safe and more reliable.

## How to use 

### Configure the project

```
idf.py menuconfig
```

* Set WiFi mode (station or SoftAP) under Example Configuration Options.
* Set ESPNOW primary master key under Example Configuration Options.
  This parameter must be set to the same value for sending and recving devices.
* Set ESPNOW local master key under Example Configuration Options.
  This parameter must be set to the same value for sending and recving devices.
* Set Channel under Example Configuration Options.
  The sending device and the recving device must be on the same channel.
* Set Send count and Send delay under Example Configuration Options.
* Set Send len under Example Configuration Options.
* Set Enable Long Range Options.
  When this parameter is enabled, the ESP32 device will send data at the PHY rate of 512Kbps or 256Kbps
  then the data can be transmitted over long range between two ESP32 devices.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Here is the example of ESPNOW receiving device console output.

```
I (898) phy: phy_version: 3960, 5211945, Jul 18 2018, 10:40:07, 0, 0
I (898) wifi: mode : sta (30:ae:a4:80:45:68)
I (898) submersion_module: WiFi started
I (898) ESPNOW: espnow [version: 1.0] init
I (5908) submersion_module: Start sending broadcast data
I (6908) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (7908) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (52138) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (52138) submersion_module: Receive 0th broadcast data from: 30:ae:a4:0c:34:ec, len: 200
I (53158) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (53158) submersion_module: Receive 1th broadcast data from: 30:ae:a4:0c:34:ec, len: 200
I (54168) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (54168) submersion_module: Receive 2th broadcast data from: 30:ae:a4:0c:34:ec, len: 200
I (54168) submersion_module: Receive 0th unicast data from: 30:ae:a4:0c:34:ec, len: 200
I (54678) submersion_module: Receive 1th unicast data from: 30:ae:a4:0c:34:ec, len: 200
I (55668) submersion_module: Receive 2th unicast data from: 30:ae:a4:0c:34:ec, len: 200
```

Here is the example of ESPNOW sending device console output.

```
I (915) phy: phy_version: 3960, 5211945, Jul 18 2018, 10:40:07, 0, 0
I (915) wifi: mode : sta (30:ae:a4:0c:34:ec)
I (915) submersion_module: WiFi started
I (915) ESPNOW: espnow [version: 1.0] init
I (5915) submersion_module: Start sending broadcast data
I (5915) submersion_module: Receive 41th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (5915) submersion_module: Receive 42th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (5925) submersion_module: Receive 44th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (5935) submersion_module: Receive 45th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (6965) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (6965) submersion_module: Receive 46th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (7975) submersion_module: send data to ff:ff:ff:ff:ff:ff
I (7975) submersion_module: Receive 47th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (7975) submersion_module: Start sending unicast data
I (7975) submersion_module: send data to 30:ae:a4:80:45:68
I (9015) submersion_module: send data to 30:ae:a4:80:45:68
I (9015) submersion_module: Receive 48th broadcast data from: 30:ae:a4:80:45:68, len: 200
I (10015) submersion_module: send data to 30:ae:a4:80:45:68
I (16075) submersion_module: send data to 30:ae:a4:80:45:68
I (17075) submersion_module: send data to 30:ae:a4:80:45:68
I (24125) submersion_module: send data to 30:ae:a4:80:45:68
```


