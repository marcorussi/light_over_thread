# light_over_thread
This projects is a simple demo about using Thread networking for controlling one or more lights from a single controller. Three Nordic pca10056 boards are used as two servers and one client. Each server role represents a controlled light while the client role is the controller. The project has been designed and tested for one controller only in the network.
The controller can turn the lights ON or OFF or dim them up and down. By default, all connected lights are controlled at the same time but it is possible to control one of them singularly. To do this, press a dedicated button on the desired light (server) and then press a related button on the controller within 5 seconds. Pressing the same button on the controller moves back to the multi-control. See board LEDs and buttons assignments here below:
Server LED assignments:
* BSP_LED_0: Network state:
		* Blinking – Disconnected
		* Solid - Connected
* BSP_LED_1: Blinking: "single control" enabled and waiting for related message from client 
* BSP_LED_2: Not used
* BSP_LED_3: Controlled light 
Server button assignments:
* BSP_BUTTON_0: Enable "single control"
* BSP_BUTTON_1: Not used
* BSP_BUTTON_2: Not used
* BSP_BUTTON_3: Not used
Client LED assignments:
* BSP_LED_0: Network state:
		* Blinking – Disconnected
		* Solid - Connected
* BSP_LED_1: Not used
* BSP_LED_2: Not used
* BSP_LED_3: Not used
Client button assignments:
* BSP_BUTTON_0: Send a multicast "single control" request or exit from "single control" state
* BSP_BUTTON_1: Send a multicast or unicast light toggle command (ON/OFF)
* BSP_BUTTON_2: Send a multicast or unicast dimming down light command
* BSP_BUTTON_3: Send a multicast or unicast dimming up light command

BSP_BUTTON_1 of the controller sends messages to toggle lights state between ON and OFF. BSP_BUTTON_2 and BSP_BUTTON_3 dim lights respectively down and up.
The "single control" state is started by pressing BSP_BUTTON_0 on the server side. The related provisioning CoAp service is added and the controller (client) should send a "light control" multicast request within 5 seconds. Indeed adter this time-out, the provisioning resource is removed. If the controller sends the request (by pressing BSP_BUTTON_0) and the server receives it successfully then the server replies with a specific message containing its IPv6 address. The client receives it and stores it as peer device address so next light control messages will be sent as unicast messages to that peer device. Unicast messages control a single light and multicast messages are used for controlling all the lights and sending a "single control" request. In "single control" state, pressing BSP_BUTTON_0 again exits from this state by deleting the peer device. Next light control messages will be sent as multicast.

**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to /opt directory. 
Download the Nordic SDK for Thread from https://www.nordicsemi.com/eng/Products/nRF5-SDK-for-Thread. Unpack it and move it to /opt directory. 
Clone this repo in your local directory:

	$ git clone https://github.com/marcorussi/light_over_thread.git

If you use a Nordic pca10056 board for the server side (light), you must change board BSP defines:
1) Navigate in 'components/boards';
1) Copy the pca10056_mod.h file provided in this project;
2) Add the following lines in boards.h file' just after the related pca10056 code:

```
#elif defined(BOARD_PCA10056_MOD)
  #include "pca10056_mod.h"
```

Find the Makefile for client and server both, verify and modify following names and paths as required according to your enrironment and preferences:

```
PROJECT_NAME     := light_server
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build
SDK_ROOT := /opt/nRF5_SDK_for_Thread_v0.8.0_fc4eda1
PROJ_DIR := ./
NRFJPROG_DIR := /opt/nRF5x-Command-Line-Tools_9_4_0_Linux-x86_64/nrfjprog/
LINKER_SCRIPT  := $(PROJ_DIR)/nrf52840.ld
```

**Build and Flash**

Build each project and flash two different boards as below:
	$ cd light_client
	$ make
	$ make flash
	$ cd ../light_server
	$ make
	$ make flash

You can flash more than one board as server.




