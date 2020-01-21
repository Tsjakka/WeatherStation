# Weather Station
An ESP32 / Arduino project to create a high-quality sensor unit for use with Fine Offset weather stations

For several years I enjoyed my Alecto 4000 weather station (more commonly known as Fine Offset WH1080). It has a good display and an easy to mount outdoor unit with anemometer and wind direction. I could connect it to my Raspberry Pi so I was able to get weather information over the internet at any time.
The downside of the system is the relatively short lifespan of the outdoor unit. Based on my experience with two systems I estimate it lasts around three years. I also couldn’t attest to the accuracy of wind measurements, because the outdoor unit was mounted on a short pole and will surely have been influenced by its close proximity to the house.
Last year the outdoor unit of my second system got stuck and I started to contemplate an alternative. As it happened, a long time ago I was given a professional anemometer that was struck by lightning and needed to be replaced. I decided that the time had come to give that one a try and see if it still worked and if I could use it in a new, better weather system.

Since the anemometer does not come with a display or anything, I thought about combining the display of the Alecto with the anemometer. I saw basically two paths that I could take; 1. using the original outdoor transmitter and connecting the anemometer by converting its output to something the transmitter can understand or 2. replacing the whole outdoor unit with something else (and preferably better). 

After some failed attempts at solution 1, I decided to go for option 2. Discussing this solution with a friend, I decided to recreate the outdoor unit using the following parts:

* An ESP32 microcontroller
* A Bosch BME280 temperature/pressure/humidity sensor
* A DCF1 DCF time receiver
* A Hope RMF69HW RF transceiver
* The Thiess 3323.21.002 anemometer
* The original Alecto rain meter

In a schematic:

![Topology](https://github.com/Tsjakka/WeatherStation/blob/master/Photos/Topology.jpg)

The first 4 parts are to be housed in the original Alecto housing. The whole thing to be powered by 230V or optionally a solar panel.

The software will be made in the Arduino software development environment. For all these devices Arduino libraries are available, which makes development much easier.

After ordering the parts, I started connecting and integrating the individual devices one by one, using a breadboard. It took many months to integrate all parts into the system, creating both the electrical connections and the software. Especially tricky was getting the RF communication to the Alecto display to work. Although there is quite a lot of info and sample code available on the internet regarding this, all examples involve the opposite way of using it, where the original outdoor sensor is used and the signal is received by another device, such as a Raspberry Pi.

![Breadboard](https://github.com/Tsjakka/WeatherStation/blob/master/Photos/IMG_2620.JPG)

When I had the system running on the breadboard, I started working on a PCB that fits the housing of the Alecto. This turned out to be a lot of work too, since each connection has to be soldered manually. In the Photos directory you can find more images of this.

![PCB](https://github.com/Tsjakka/WeatherStation/blob/master/Photos/PB140030.JPG)

During this process I lost a lot of time because of a badly soldered connection. This led me to replace the RF chip twice, which didn’t fix it. The problem was that when verifying the connections, I pressed my Ohm meter against the bad connection, which was then shorted. As soon as I removed my Ohm meter the connection became unstable again.

The Arduino software, which can be downloaded from here, contains a lot of software features:

* Reading data from all included sensors (wind direction and -speed, rain, temperature, humidity, pressure and time).
* Calculating aggregated (wind) data using algorithms based on the recommendations of the World Meteorological Society.
* Building the RF packets that are sent to the display.
* Sending the data packets over RF every 48 seconds.
* Over the air (OTA) updating of the software.

Also the OTA functionality turned out to be tricky. One of the problems was that you cannot use certain GPIO when using the WiFi functionality of the ESP32. Refer to [RandomNerdTutorials](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/) for a lot of info on connecting peripherals to the ESP32. The result was that I had to resolder several connections. Because I could not do that for the GPIOs used for the anemometer, I decided to turn on the WiFi connection only after rebooting the ESP32 twice within 20 seconds. Another issue is that WiFi reception is considerably worse when the ESP32 is connected to the PCB.

In the end, I got everything working properly. Here you can see the weather station in action:

![Outside](https://github.com/Tsjakka/WeatherStation/blob/master/Photos/IMG_2628.JPG)

Although it was frustrating at times, I enjoyed this project as a creative and learning experience. The results can be seen on my weather site, http://www.blixemboschweer.nl. I hope some of you will make use of the results, perhaps even by building your own weather station. Please report on your experience if you can.

Greetings,
Tsjakka

![email](https://github.com/Tsjakka/WeatherStation/blob/master/Photos/blixemboschweer.gif)
