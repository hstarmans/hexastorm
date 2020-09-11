# Examples
The folder includes several examples for migen. These examples are used to gain experience with migen.
The goal of these tests is to replace the microcontroller of [ldgraphy](https://github.com/hstarmans/ldgraphy) with a FPGA.

### Blinky
Blinks a led at the Hexastorm board.

### SPI counter
Counts the number of interactions over SPI.

### SPI mapping
Linux sends two words over SPI, the command word and data write word respectively. The data write word is not used.
The command table is as follows;
```
 command 1 --> reply with 2
 command 2 --> reply with 8
 else      --> reply with 0
```

### Memtest 
A simple code example to check if I can trigger [yosys](http://www.clifford.at/yosys/) to create an embedded block ram block. 
If you check the output of Icezero you see that a block is created.
The specs of the embedded block rams are listed [here](http://www.latticesemi.com/~/media/LatticeSemi/Documents/DataSheets/iCE/iCE40LPHXFamilyDataSheet.pdf)

### SPI memory mapping
The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
The on chip memory will still typically be empty.
The linux host sends one word made up out of two bytes over SPI. The first byte is the command. 
The second byte is optionally the data or ignored. The command table is as follows;
```
    command 1 --> write data in register 0 and reply with (0,1)
    command 2 --> read value from register 1 and reply with (0,value)
    command 3 --> change address and reply with (0,3)
    else --> reply with (0,0)
```

### SPI state machine with LED
Has a lot of the complexity present in laser scanner but works with a simple LED.
In short, you can write over SPI to the memory. You can turn on the machine.
If there is data in the memory and the machine is turned on, it will read date in the memory sequentially. The led is on if a bit is 1 and off otherwise.
If there is no data or the memory is full the user is informed by an error.