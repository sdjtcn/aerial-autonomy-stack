# Bill of Materials

> The following is an example of the hardware components necessary to build a quadcopter supporting **ALL** of `aerial-autonomy-stack`'s capabilities, including perception and multi-robot swarming 

## AAS Drone

| #   | Part                                  | Description                                            | Link         |
| --- | ------------------------------------- | ------------------------------------------------------ | ------------ |
| 1   | Holybro X650 Almost-ready-to-fly Kit  | Quadcopter frame, motors, ESCs, propellers             | [URL][kit]
| 2   | Holybro H-RTK ZED-F9P Ultralight      | GNSS module (GPS, GLONASS, Galileo, BeiDou)            | [URL][gps]
| 3   | Holybro Fixed Carbon Fiber GPS mount  | GNSS module support                                    | [URL][mount]
| 4   | Holybro Microhard Telemetry Radio*    | Point-to-multipoint telemetry (1 ground + 1 per drone) | [URL][telem]
| 5   | RadioMaster Boxer RC CC2500           | Radio controller                                       | [URL][rc]
| 6   | RadioMaster R81 V2 Receiver           | Receiver for the radio controller                      | [URL][rec]
| 7   | Matek POWER MODULE PM12S-4A           | 5V and 12V supply for Doodle and Jetson                | [URL][matek]
| 8   | Tattu G-Tech 6S 8000mAh 25C 22.2V     | Lipo battery pack with XT60                            | [URL][batt]
| 9   | Pixhawk Jetson Baseboard Bundle       | Incl. NVIDIA Orin NX 16GB+SSD, Pixhawk 6X, CSI ArduCam | [URL][jetson]
| 10  | ASIX AX88772A USB2.0 Ethernet Adapter | 1 for `AIR_SUBNET`, 1 for `SIM_SUBNET` (for HITL only) | [URL][eth]
| 11  | DJI Livox Mid-360                     | LiDAR sensor                                           | [URL][liv]
| 12  | Livox three-wire aviation connector   | Power and ethernet connector                           | [URL][liv2]
| 13  | Doodle Labs RM-2450-11N3              | 2.4GHz nano radio module (1 ground + 1 per drone)      | [URL][doot1]
| 14  | Doodle Labs EK-2450-11N3              | Nano carrier/evaluation kit (1 ground + 1 per drone)   | [URL][doot2]

> *For a single drone, one can alternatively use the point-to-point [SiK telemetry radio][telem2]

[kit]:https://holybro.com/collections/x650-kits/products/x650-kits?variant=43994378240189
[gps]:https://holybro.com/collections/standard-h-rtk-series/products/h-rtk-f9p-ultralight?variant=45785783009469
[mount]:https://holybro.com/collections/gps-accessories/products/fixed-carbon-fiber-gps-mount?variant=42749655449789
[telem]:https://holybro.com/collections/telemetry-radios/products/microhard-radio?variant=42522025590973
[telem2]:https://holybro.com/collections/telemetry-radios/products/sik-telemetry-radio-1w?variant=45094904856765
[rc]:https://radiomasterrc.com/collections/boxer-radio/products/boxer-radio-controller-m2?variant=46486352298176
[rec]:https://holybro.com/collections/rc-radio-transmitter-receiver/products/radiomaster-r81-receiver
[matek]:https://www.mateksys.com/?portfolio=pm12s-4a
[batt]:https://genstattu.com/tattu-8000mah-22-2v-25c-6s1p-lipo-battery-pack-with-xt60-plug.html
[jetson]:https://holybro.com/collections/flight-controllers/products/pixhawk-jetson-baseboard?variant=44636223439037
[eth]:https://www.amazon.ca/TRENDnet-TU2-ET100-USB-Mbps-Adapter/dp/B00007IFED/
[liv]:https://store.dji.com/ca/product/livox-mid-360?vid=130851
[liv2]:https://store.dji.com/ca/product/livox-three-wire-aviation-connector?vid=117441
[doot1]:https://www.mouser.ca/ProductDetail/Doodle-Labs/RM-2450-11N3?qs=ulEaXIWI0c91eCn7VRB%2FpA%3D%3D
[doot2]:https://www.mouser.ca/ProductDetail/Doodle-Labs/EK-2450-11N3?qs=ulEaXIWI0c%2FLOqPeL4gNgg%3D%3D
