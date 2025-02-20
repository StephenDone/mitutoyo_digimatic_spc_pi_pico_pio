# mitutoyo_digimatic_spc_pi_pico_pio

This repo contains micropython code to run on a Raspberry PI Pico.

It utilises the PIO (programmable I/O) state machines within the Pico to communicate with Mitutoyo Digimatic SPC compatible equipment, such as dial indicators.

Use of the PIOs in the Pico reduces the load on the main processor. When reading values from the dial gauges below as fast as possible, each SPC channel consumes less than 2% of the Pico's main processor time.
In fact, more processor time is used to print the values. This leaves plenty of spare cycles to update a display or send the data onward via a serial port etc.

The code is event driven, with no blocking or polling.

If you find this code useful, or find a bug, please drop me a message by creating an issue.

## My Single Channel Prototype Board
<img src="https://github.com/user-attachments/assets/bef29472-0931-4f79-bb28-a6d60f364e4d" width=100/>
<img src="https://github.com/user-attachments/assets/f736b668-a89a-4fa4-8c6c-ba765cfed66e" width=100/>

## My Quad Channel Prototype Board
<img src="https://github.com/user-attachments/assets/8b685e3f-20d0-4c36-9c64-7b5516c7dbee" width=215/>
<img src="https://github.com/user-attachments/assets/94d20fcb-0250-46db-8784-6080e5e96a46" width=200/>
<br/>
<img src="https://github.com/user-attachments/assets/1b2be72f-2da5-44e2-8ac0-32b8af398a61" width=400/>

## A Proper PCB
I've designed a 6-channel board with EasyEDA and have sent it to JLPCB. If it works, I'll update this section.

![PCB](https://github.com/user-attachments/assets/72e5d316-2d85-4483-93ce-5d9b92593cae)

## Other Resources
<a href="http://www.matronics.com/cncscale/DiagramRev2.pdf">http://www.matronics.com/cncscale/DiagramRev2.pdf</a>

<a href="http://www.imajeenyus.com/electronics/20140109_digimatic_interface/index.shtml">http://www.imajeenyus.com/electronics/20140109_digimatic_interface/index.shtml</a>

<a href="https://circuitpython-mitutoyo.readthedocs.io/en/latest/api.html">https://circuitpython-mitutoyo.readthedocs.io/en/latest/api.html</a>

<a href="https://learnarduinonow.com/2011/10/14/digimatic-spc-to-arduino.html">https://learnarduinonow.com/2011/10/14/digimatic-spc-to-arduino.html</a>

<a href="https://www.instructables.com/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/">https://www.instructables.com/Interfacing-a-Digital-Micrometer-to-a-Microcontrol/</a>


