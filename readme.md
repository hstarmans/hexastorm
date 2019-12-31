# Install Notes
Install migen
```python
pip3 install -e 'git+http://github.com/m-labs/migen.git#egg=migen'
```
The idea is to use the Litex cores.

The following examples exist;

## Blinky
Blinks a led at the IceZero boards.

## Spi loopback
A SPI slave with loop back is created; MISO equals MOSI. Bytes sent from the Raspberry equal those received.

## Spi counter
The max length a word the Raspberry Pi driver can sent is 8 bit. If you want to receive longer words, you will need to count. In this example, a word is sent and the count is replied.

## Spi mapping
The Raspberry pi sends two words over SPI. The first word is the command word. The second word is the data write word.
The command table is as follows;
```
 command 1 --> reply with 2
 command 2 --> reply with 8
 else      --> reply with 0
```
The data write word is not used.

## Blink Mem
The memory is initialized with 1 or 0.
The FPGA reads from the memory. If the memory is 1 the led is on and off otherwise.
This example contains a mistake. The memory is not synthesized.

## Memtest 
Same as above alone now the memory is also synthesized. If you check the output of Icezero you see that 
a block is created.


## Spi memory mapping
The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
As before, Raspberry pi sends two words over SPI. The first word is the command word. 
The second word is the data write word.
The command table is as follows;
```
command 1 --> write value in register 0 and obtain 0
command 2 --> write value in register 1 and obtain 0
command 3 --> obtain the result of the sum of register 0 and 1.
else      --> reply with 0
```
Note that you can flow over the sum, as both arguments and the sum is 8 bit.


<!--
Planning;
 kijk of je geheugen test werkt
 kijk of je geheugen gebruikt ja
 http://xess.com/static/media/pages/pygmyhdl/examples/4_blockram/block_ram_party.html


Je memory test doet het niet;
 schrijf getal 1 naar adres 1
 schrijf getal 2 naar adres 2
 lees van adres 1
 als gelijk aan 1 led aan en draai huidige test
 vergelijk dan met migen code

Wat kun je doen?
  - de code werkt op een FPGA, maar je memory werkt zo niet.. dit moet anders
  - zie https://git.p-fb.net/pef/olimex/blob/master/adc.py
  - maak eerst een blink met memory
  - je kunt de code opruimen en beter leesbaar maken, door het gebruik van een statemachine
  - als ik een pakket verstuurd heb wil ik weten dat ie is aangekomen
  - ik wil een test maken waarin je data stuurt naar de fpga, verwerkt, en dan weer opnieuw stuurt.
         hoe snel kan dit?

Wat is de basis van het apparaat?
  - je kunt vragen wat is je toestand
  - je kunt een fout herkennen als die optreedt en krijgt informatie over die fout

## Laser scanner
# de host stuurt een willekeurig woordt, het krijgt ik wil data of ik wil geen data terug
# als de status is geef data, dan stuurt de raspberry data
# als de slave voldoende ontvangen heeft, dan leest de slave de data uit, de status wordt ik wil geen data
# als de slave niet voldoende ontvangen heeft, dan zegt de slave ik wil data
 -->
