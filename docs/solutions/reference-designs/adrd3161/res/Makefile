all: wireviz

wireviz: cable-can.gv cable-encoder-bldc-abn.gv cable-encoder-bldc-vesc.gv cable-encoder-bldc.gv cable-encoder-stepper.gv 

%.gv: %.yml
	wireviz -fg $^

