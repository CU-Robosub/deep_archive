# PCB_IntLibs
Integrated Libraries for Altium PCB Projects
### Current Monitor Int Lib 
Contains AD8217 and three unique sense resistors
### Kill Switch Int Lib 
Contains IRLR6225 MOSFET and 0603 Resistor
### Merge Circuit Int Lib 
Contains LTC4357 and IPT004 MOSFET
### ESC/ESC Sample Int Lib
Contains AD8217, IRLR6225 MOSFET, and Molex 76825-0002
### Power/Power Int Lib
Needs to be created

## Merge Board Information:
Merge board will contain the Merge Circuit Int Lib with inputs tied to Molex Battery Connectors and output tied to Molex Board to Board Connectors
### Merge Circuit
1. [LTC4357 Datasheet](http://cds.linear.com/docs/en/datasheet/4357fd.pdf)
2. [IPT004 Datasheet](https://www.infineon.com/dgdl/Infineon-IPT004N03L-DS-v02_00-EN.pdf?fileId=db3a30433e9d5d11013e9e0f382600c2)
### Connectors
1. [Molex Mini Fit Sr.](https://www.molex.com/molex/products/family?key=minifit_sr&channel=products&chanName=family&pageTitle=Introduction&parentKey=wire_to_board_connectors)
2. [Board to Board Connectors: I hope these work](https://www.molex.com/molex/products/family?key=extreme_ten60power_highcurrent_connector&channel=products&chanName=family&pageTitle=Introduction&parentKey=board_to_board_connectors)


## ESC Board Information:
A proper ESC Board will contain current monitors with their outputs tied to a microcontroller analog in pin, and proper resistors for the output current of the motors (See AD8217 Datasheet for calculation). It will also contain Kill Switches on each ESC. 
### Current Monitor
1. [AD8217 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD8217.pdf)
### Kill Switch
1. [IRLR6225 Datasheet](https://www.infineon.com/dgdl/irlr6225pbf.pdf?fileId=5546d462533600a40153566d99bc26c1)
2. [How to use a MOSFET as a switch](http://www.electronics-tutorials.ws/transistor/tran_7.html)
### Connectors
1. [Molex Mega-Fit 2 Pin Connector](https://www.molex.com/molex/products/datasheet.jsp?part=active/0768250002_PCB_HEADERS.xml)
2. [Board to Board Connectors: I hope these work](https://www.molex.com/molex/products/family?key=extreme_ten60power_highcurrent_connector&channel=products&chanName=family&pageTitle=Introduction&parentKey=board_to_board_connectors)

## Power Board Information:
### 5V Converter
1. [R-78-E-0.5 Datasheet](https://www.recom-power.com/pdf/Innoline/R-78Exx-0.5.pdf) 
2. [AD8217 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD8217.pdf)
### 12V and 19V Converter (mostly passive component changes)
1. [LTC3780 Datasheet](http://cds.linear.com/docs/en/datasheet/3780ff.pdf)
2. [LTC3780 LTSpice](http://www.linear.com/product/LTC3780)
3. [BUK6212 Datasheet](https://assets.nexperia.com/documents/data-sheet/BUK6212-40C.pdf)
4. [MMSZ5230B Datasheet](https://www.diodes.com/assets/Datasheets/ds18010.pdf)
5. [MBRS340 Datasheet](https://www.mouser.com/ds/2/308/MBRS340-1120985.pdf)
6. [IN5819 Datasheet](https://www.diodes.com/assets/Datasheets/ds23001.pdf)
7. [SRR1240 Datasheet](https://www.bourns.com/docs/Product-Datasheets/SRR1240.pdf)
8. [AD8217 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD8217.pdf)
### 48V Converter
1. [LTC3958 Datasheet](http://cds.linear.com/docs/en/datasheet/3958fa.pdf)
2. [LTC3958 LTSpice](http://www.linear.com/product/LT3958)
3. [IHLP-6767DZ-11 Datasheet](http://www.vishay.com/docs/34306/lp67dz11.pdf)
4. [AD8217 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD8217.pdf)
###
1. [Board to Board Connectors: I hope these work](https://www.molex.com/molex/products/family?key=extreme_ten60power_highcurrent_connector&channel=products&chanName=family&pageTitle=Introduction&parentKey=board_to_board_connectors)

