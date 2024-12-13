.. _getting-started:

Getting Started
===============

This chapter provides the basic information to get started with ROS2 on your ComboFox platform.

Network Connection
-------------------

By default your ComboFox platform will try to connect to the wireless network with SSID **awcombo** .

First power up the WiFi access point included and wait until the network ComboFox is available. Optionally connect the access point to your company/home network via ethernet to enable internet access on the platform when needed.

Next you may connect your PC to ComboFox too, the password can be found on the access point's exterior.

The IP-Address of your mobile platform is **192.168.0.4**.
Once the platform has booted and your PC is connected to ComboFox you can connect to the platform using:

- SSH

Bringup ComboFox
-------------------
Please follows the following steps to turn on ComboFox platform:

1. Power up Wifi access
2. Turning on the key selector
3. Reset clicking the reset button three times
4. You are ready to command ComboFox

.. warning:: By turning on ComboFox with the key selector a startup procedure will launch the appropriate controllers, actions, topics and services. **Wheel homing and awtube arm pairing will be performed**.

Hopefully from `rviz2` or from others visualizators you should be able to see something like

.. figure:: ./img/rviz.png