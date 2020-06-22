COMMANDS
  1 STATUS  --> retrieves status
  2 START   --> start motor and stabilization 
  3 STOP    --> stop motor and stabilization 
  5 READ_D  --> read debug symbol
  6 WRITE_L --> add line with direction to stack
  7 READ_L  --> read line with direction from stack

STATES;
  0 OFF
  1 STABLE

ERRORS:
  0 NO ERROR
  1 MIRROR_SYNC   // Mirror failed to sync

length command equals length reply

COMMAND:
    COMMAND_BYTE + DATA_BYTES
    DATA_BYTES are split up as follows
      status; empty
      start; emtpy
      stop ; empty
      read debug; empty
      write line ; first byte is direction, last data_bytes are information in line
 
REPLY
    STATE_ERRORBYTE + DATA_BYTES
    DATA_BYTES are split up as follows
      status; data bytes are sequence 1 to 8
      start; emtpy
      stop ; empty
      read debug; first four empty, last four are constant value
      write line ; first byte whether or not write is accepted, 1 accepted, 2 not accepted

CONSTANTS;
     RPM = 2400                    # revolutions per minute
     SPINUP_TICKS = 1.5            # seconds
     MAX_WAIT_STABLE_TICKS = 1.125 # seconds
     FACETS = 4
     SCANLINE_DATA_SIZE = 790      # pixels in a line
     TICKS_PER_PRISM_FACET = 12500 # ticks per prism facet
     TICKS_START = 4375            # laser start in off state
     SINGLE_FACET                  # wether to use single facet
     DIRECTION
     JITTER ALLOW = int(round(TICKS_PER_PRISM_FACET / 3000 ))
     JITTER THRESH = int(round(TICKS_PER_PRISM_FACET / 400 ))

PROCESS VARIABLES:
     STATE
     ERROR
     LINES IN MEMORY
     LINES WRITTEN

MEMORY:
    Say that memory is limited to 32 lines
    you have two cursors, read and write
        if write>read:
            read lines are empty
            write-read are full
        ir write<read
            read-write lines are empty
            write+32-read are full
    start state; read is 0, write is 1
        if after (read+=1)%32, write == read --> memory empty
                 (write+=1)%32, write == read --> memory full

CONSTANTS;
    SCANLINE_HEADER_SIZE = 1
    SCANLINE_DATA_SIZE = 8
    SCANLINE_ITEM_SIZE = SCANLINE_HEADER_SIZE + SCANLINE_DATA_SIZE
    STATUS_BYTE = 0
    ERROR_BYTE = 1
    CONSTANT_BYTE = 2
    CONSTANT_LENGTH = 4
    MEMORY_SIZE = 32  # number of lines that can be stored in memory