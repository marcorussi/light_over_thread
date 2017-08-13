# light_over_thread
This projects is a simple demo about using Thread networking for controlling one or more lights from a single controller. Three Nordic pca10056 boards are used as two servers and one client roles. 

The application has been developed on top of the Nordic SDK for Thread version 0.10.0 which in turn contains a pre-built version of the [openthread](https://github.com/openthread/openthread) library (see SDK release notes for more details).

---

**Behaviour**

Based on the Simple CoAp Client and Server examples provided with the SDK, the application uses a couple of CoAp services over a default Thread network. 
Each server role represents a controlled light while the client role is the controller. The project has been designed and tested for one controller only in the network. The controller can turn the lights ON or OFF or dim them up and down. By default, all connected lights are controlled at the same time but it is possible to control one of them singularly. To do this, press a dedicated button on the desired light (server) and then press a related button on the controller within 5 seconds. Pressing the same button on the controller moves back to the multi-control. See board LEDs and buttons assignments here below:

Server LED assignments:
* BSP_LED_0: Network state: Blinking – Disconnected, Solid - Connected
* BSP_LED_1: Blinking: *single control* enabled and waiting for related message from client 
* BSP_LED_2: Not used
* BSP_LED_3: Controlled light 

Server button assignments:
* BSP_BUTTON_0: Enable *single control*
* BSP_BUTTON_1: Not used
* BSP_BUTTON_2: Not used
* BSP_BUTTON_3: Not used

Client LED assignments:
* BSP_LED_0: Network state: Blinking – Disconnected, Solid - Connected
* BSP_LED_1: Not used
* BSP_LED_2: Not used
* BSP_LED_3: Not used

Client button assignments:
* BSP_BUTTON_0: Send a multicast *single control* request or exit from *single control* state
* BSP_BUTTON_1: Send a multicast or unicast light toggle command (ON/OFF)
* BSP_BUTTON_2: Send a multicast or unicast dimming down light command
* BSP_BUTTON_3: Send a multicast or unicast dimming up light command

BSP_BUTTON_1 of the controller sends messages to toggle lights state between ON and OFF. BSP_BUTTON_2 and BSP_BUTTON_3 dim lights respectively down and up.
The *single control* state is started by pressing BSP_BUTTON_0 on the server side. The related provisioning CoAp service is added and the controller (client) should send a *single control* multicast request within 5 seconds. Indeed after this time-out, the provisioning resource is removed. If the controller sends the request (by pressing BSP_BUTTON_0) and the server receives it successfully then the server replies with a specific message containing its IPv6 address. The client receives it and stores it as peer device address so next light control messages will be sent as unicast messages to that peer device. Unicast messages control a single light while multicast messages are used for controlling all the lights and sending a *single control* request. In *single control* state, pressing BSP_BUTTON_0 again exits from this state by deleting the peer device. Next light control messages will be sent as multicast.


*UART channel*

The client has a UART channel for sending special commands in JSON format to turn lights on and off as below:
* lights on: {"command":[{"light":"on"}]}.
* lights off: {"command":[{"light":"off"}]}.

Note that this UART feature is not well implemented and the JSON string is not properly parsed but just brutally compared. The terminal character '.' is for indicating the end of command to stop buffering and executing the command.

---

**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to `/opt` directory. 
Download the Nordic SDK for Thread from https://www.nordicsemi.com/eng/Products/nRF5-SDK-for-Thread. Unpack it and move it to `/opt` directory. 
Clone this repo in your local directory:

	$ git clone https://github.com/marcorussi/light_over_thread.git

If you use a Nordic pca10056 board for the server side (light), you must change board BSP defines:
1) Navigate in `components/boards` in the SDK folder;
1) Copy the `pca10056_mod.h` file provided in this project;
2) Add the following lines in `boards.h` file just after the related pca10056 code:

```
#elif defined(BOARD_PCA10056_MOD)
  #include "pca10056_mod.h"
```

Find the Makefile (for both client and server), verify and modify following names and paths as required according to your enrironment and preferences:

```
PROJECT_NAME     := light_server
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build
SDK_ROOT := /opt/nRF5_SDK_for_Thread_v0.10.0_e1c3d11
PROJ_DIR := ./
NRFJPROG_DIR := /opt/nRF5x-Command-Line-Tools_9_4_0_Linux-x86_64/nrfjprog/
LINKER_SCRIPT  := $(PROJ_DIR)/nrf52840.ld
```

Be sure you have the ARM GCC toolchain installed (gcc-arm-none-eabi-4_9-2015q3 version is advised) and modify the path to it in `/components/toolchain/gcc/Makefile.posix` in the SDK:

```
GNU_INSTALL_ROOT := /usr/local/gcc-arm-none-eabi-4_9-2015q3
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
```

---

**Build and Flash**

Build each project and flash two different boards as below:

	$ cd light_client
	$ make
	$ make flash
	$ cd ../light_server
	$ make
	$ make flash

You can flash more than one board as server.




