# Background
The ICEBoard has an external SRAM. An example on how to use this SRAM has been given by Clifford Wolf in [icotools] (https://github.com/cliffordwolf/icotools/tree/master/examples/icezero)
. The example requires a SB_IO and a SB_PLL40_PAD instance. This examples can be recreated with clock domain renamer.
The current idea is to use the block rams provided on the chip. The BRAM chips are 4K bit large. The ICE40HX4K has 81920 bits. So it should have something like 20 of these block rams. The beaglebone, used in LDGraphy, had
8K byte per core and 12K byte shared. If your line is 8 mm, you then use 800 pixels per line at 10 micron.