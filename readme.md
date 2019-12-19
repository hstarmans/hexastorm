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
 command 1 --> reply with 2
 command 2 --> reply with 8
 else      --> reply with 0
The data write word is not used.

## Spi memory mapping
The memory is initiated with the value 10 in register 0 and the value 20 in register 20.
As before, Raspberry pi sends two words over SPI. The first word is the command word. The second word is the data write word.
The command table is as follows;
    command 1 --> write value in register 0
    command 2 --> write value in register 1
    command 3 --> obtain the result of the sum of register 0 and 1.
    else      --> reply with 0



# je moet een reset hebben als er een fout optreed

# de host stuurt een willekeurig woordt, het krijgt ik wil data of ik wil geen data terug
# als de status is geef data, dan stuurt de raspberry data
# als de slave voldoende ontvangen heeft, dan leest de slave de data uit, de status wordt ik wil geen data
# als de slave niet voldoende ontvangen heeft, dan zegt de slave ik wil data