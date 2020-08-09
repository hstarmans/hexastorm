# Migen Test

The repository includes several code examples for m]igen. 
The goal of these test is to replace the microcontroller of [ldgraphy](https://github.com/hstarmans/ldgraphy) with a FPGA.

## Install Notes
Install migen
```python
pip3 install -e 'git+http://github.com/m-labs/migen.git#egg=migen'
```
On the Pi install pigpio and enable the daemon upon boot.
```
sudo apt install pigpio
sudo systemctl enable pigpiod
```

## Examples

### Blinky
Blinks a led at the IceZero boards.

### Spi loopback
This doesn't work. Should probaly be removed.
A SPI slave with loop back is created; MISO equals MOSI. Bytes sent from the Raspberry equal those received.

### Spi counter
The max length a word the Raspberry Pi driver can sent is 8 bit. If you want to receive longer words, you will need to count. In this example, a word,  is sent and the count is replied.

### Spi mapping
The Raspberry pi sends two words over SPI. The first word is the command word. The second word is the data write word. The data write word is not used.
The command table is as follows;
```
 command 1 --> reply with 2
 command 2 --> reply with 8
 else      --> reply with 0
```

### Memtest 
A simple code example to check if I can trigger yosys to create an sram block. 
If you check the output of Icezero you see that a block is created.

### Spi memory mapping
The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
The linux host sends one word made up out of two bytes over SPI. The first byte is the command. 
The second byte is optionally the data or ignored. The command table is as follows;
```
    command 1 --> write data in register 0 and reply with (0,1)
    command 2 --> read value from register 1 and reply with (0,value)
    command 3 --> change address and reply with (0,3)
    else --> reply with (0,0)
```

### SPI state machine
Has a lot of the complexity present in laser scanner but works with a simple LED.
In short, you can write over SPI to the memory. You can turn on the machine.
If there is data in the memory, it will start read date in memory sequentially. The led is on if a bit is 1 and off otherwise.
If there is no data or the memory is full the user is informed by an error.

### SPI laserscanner
The SPI laserscanner has the following tests; 
    - spin motor, time out after 10 seconds
    - spin motor and enable laser, no time out
    - spin motor, enable laser, if photodiode high end loop or timeout after 10 seconds.
        ergo; if motor is still on after 4 seconds test probably failed. This is how functioning of diode is tested.
Furthermore there is a scanmode. 



<!--
hidden section
    You still need to add possibility to disable or enable moving.
 -->
