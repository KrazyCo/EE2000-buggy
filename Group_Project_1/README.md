# STM32 buggy project

To reconnect the STM32F091RC back to the buggy the following connections need to be made
There are 4 main pin headers, 1 Blue, 1 Black and 2 Red ones which are different in length.
The blue pin header must be connected to the left side of the board CN6, the section where analogue and power connections are available.
The black pin header must be connected on the left side of the board CN7 so that the red wire aligns with the pin E5V
The longer red header must be plugged into the upper right section of the board CN9 with the orientation being so that the yellow wire is on D15/SCL and the white wire on D8. 
The shorter red header is plugged in the bottom right section CN9 with the orientation being so that two free pins align with the TX and RX pins
The brown wire for the speakers attached to the only transistor in the buggy must be connected to PB1 which is found on CN10's outer pins.
Before powering on the buggy using 4 x AA batteries you must check that the on board regulator is on E5V not U5V.