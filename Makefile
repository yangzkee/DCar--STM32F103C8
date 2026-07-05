.PHONY: all clean flash flash-openocd help

all:
	$(MAKE) -C DFCom_Example

clean:
	$(MAKE) -C DFCom_Example clean

flash:
	$(MAKE) -C DFCom_Example flash

flash-openocd:
	$(MAKE) -C DFCom_Example flash-openocd

help:
	$(MAKE) -C DFCom_Example help
