BOARD     := mega2560
SERIALDEV := /dev/ttyACM0
LIBRARIES := AndroidAccessory 

include arduino.mk

gtkterm:
	echo $(LIBRARYPATH)
	gtkterm -p $(SERIALDEV)
	
screen: monitor