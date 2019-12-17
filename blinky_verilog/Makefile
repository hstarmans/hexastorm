synthesize: demo.bin

demo.blif: top.v
	yosys -q -p "synth_ice40 -blif demo.blif" top.v

demo.asc: demo.blif demo.pcf
	arachne-pnr -d 8k -P tq144:4k -p demo.pcf $< -o $@

demo.bin: demo.asc
	icepack $< $@

.PHONY: synthesize