# Watch AMA app

## Overview
This app demonstrates Bluetooth&#174; A2DP source, A2DP sink, AVRCP Controller/Target,
Apple Media Service (AMS), Apple Notification Center Service (ANCS),
and HFP Hands-free Unit.

Features demonstrated:

 - AIROC&#8482; A2DP Source APIs
 - AIROC&#8482; A2DP Sink APIs
 - AIROC&#8482; AVRCP (Controller/Target) APIs
 - AIROC&#8482; GATT APIs
 - Apple Media Service and Apple Notification Center Services (AMS and ANCS)
 - Handling of the UART AIROC&#8482; protocol
 - SDP and GATT descriptor/attribute configuration
 - AIROC&#8482; SCO/RFCOMM initiator APIs
 - HFP Hands-free Unit role

## Instructions
To demonstrate the app, follow these steps:

1. Build and download the application to the AIROC&#8482; board.
2. Open the ClientControl application and open the "WICED HCI" port for the device.
   Default baud rate configured in the application is defined by the BSP HCI\_UART\_DEAULT\_BAUD #define,
   usually either 3M or 115200 depending on the board UART capabilities.
3. Use the ClientControl application to send various commands as mentioned below.
4. Run the BTSpy program to view protocol and application traces.


BR/EDR Audio Source and AVRC Target:

- The Watch AMA app can demonstrate how use to BR/EDR Audio Source and AVRC TG profiles.
- Audio Source can use I2S interrupt or SW timer to decide the timing to read PCM.
  For media type as 'I2S input', it will use I2S interrupt, and you need to configure 4 gpios as AIROC&#8482; I2S PINs.
  For media type as 'Wav file' or 'Sine wave', it will use SW timer by calling wiced\_audio\_use\_sw\_timing(1).
  In general, if using the "WICED HCI" UART to transmit audio, it must either allocate I2S pins on unused pins for
  I2S interrupt OR use wiced\_audio\_use\_sw\_timing(1) to enable SW timer.
- Use the buttons in the ClientControl AV Source tab.
- To play a sine wave sample, set the audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  and select the Media type as 'Sine Wave' in the UI. In this case, built-in sine wave audio is played.
- To play music from a .wav file, select the Media type as File, browse and select a .wav file,
  and set the audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  In this case, audio for the .wav file is routed over the "WICED HCI" UART to the AIROC&#8482; board.<br>
  sinc\_44100\_16\_L440\_R1000\_50s\_stereo.wav in app folder can be used as the input of 44.1KHz 16bits stereo samples.
- To play music from the Line-In jack, select the Media type as 'I2S input' and set the
  audio frequency to the desired value (48kHz, 44.1kHz, etc.)
  In this case, audio from Line-In is encoded into I2S signals and routed to the AIROC&#8482; board.
- Put an audio sink device such as Bluetooth&#174; headphone/speaker in pairable mode.
- Click on the "Start" button from the "BR/EDR Discovery" combo box in ClientControl to find the audio sink device.
- Select the peer device in the BR/EDR Discovery combo box.
- Click the "Connect" button under the AV Source tab.
- Click the "Start Streaming" button. Music will start playing on the peer device.
- The WatchAma app uses the AVRCP Target role. Once connected to a headset/speaker,
  the app can send notifications for play status changes (Play, Pause, Stop) and
  settings changes (Repeat, Shuffle) to the peer AVRCP controller (such as a headset/speaker).<br/>
  Note: the songs shown in the AVRC TG UI and some settings such Repeat/Shuffle are for testing
  AVRC commands only, they do not indicate the actual media played and will not change the media played.

BR/EDR AVRCP Controller:

- The Watch AMA app can demonstrate how to use the AVRC CT profile.
- Disconnect all devices if any are connected.
- Make an audio source device such as an iPhone discoverable/pairable from the Bluetooth&#174; Settings UI on the phone.
- Using the "BR/EDR Discovery" "Start" button, search and select the device.
- Use the buttons in the ClientControl AVRC CT tab to Connect and accept pairing.
- Play music on the audio source device and control the music via buttons in the AVRC CT tab.
- In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
- Absolute volume change can be done via the drop down Volume or Vol Up/Down buttons.
- Note that iPhone does does not support Vol buttons.
- Note that music will continue to play on the audio source device.

BR/EDR Audio Sink and AVRC Controller:

- The WatchAma app can demonstrate how use to BR/EDR Audio Sink with the AVRC Controller profile.
- Select "Discoverable" and "Connectability" after open device
- Use an audio source device such as phone to scan and connect to WatchAma device
- A2DP Sink profile will be connected with the audio source deivce. Most phones will also connect to the AVRCP Controller profile.
- Play music on an audio source device and control the music via the buttons in the ClientControl AVRC CT tab
  once the AVRCP Controller profile is connected.

iOS ANCS and AMS GATT Services:

- The Watch AMA app can demonstrate how to use AMS and ANCS iOS services as shown below.
- Disconnect all devices if any are connected.
- Select Pairable if it is not checked.
- Click the "Start Adverts" button in the GATT tab.
- Set MAX_PHONE_CONNECTIONS for support more than one iPhone.
- From an iPhone app such as 'LightBlue', find and connect to the 'WatchAma' app.
- Allow pairing with the iPhone.
- AMS:
  - Play media on each iPhone. Play,Pause,Prev,Next,Vol Up,Vol Down notification message will be displayed on the UI.
  - Use buttons like Play,Pause,Prev,Next,Vol Up,Vol Down in the ClientControl AVRC CT tab to control the music.
  - Note that music will continue to play on iPhone.
- ANCS:
  - Incoming calls and messages to the each iPhone notification message will be displayed on the UI.
  - Make an incoming call to each iPhone. See a call notification displayed on the UI to accept or reject the call. Similarly, missed call notifications are seen.
  - Send an SMS message to each iPhone to see a message notification.

LE Client:

- The Watch AMA app can demonstrate LE Client functionality as shown below.
- Make sure there is a Bluetooth&#174; device with GATT services that is advertising. For example use an app
  such as 'LightBlue' on your phone and create a 'Virtual Peripheral' such as 'Blood Pressure'.
- To find GATT devices:
  - Click on the "Start" button for the "LE Discovery" combo box.
  - Click on the "Stop" button to end discovery.
- To connect a LE device:
  - Choose a device from the "LE Discovery" drop down combo box and click the "Connect" button.
- To discover services: Click on the "Discover Services" button
- To discover characteristics: Enter the handles in the edit box and click
  on "Discover Characteristics"
- To discover descriptors: Enter the handles in the edit box and click on
  "Discover Descriptors"
- Enter the Handle and Hex Value to write to the remote device using buttons:
  - "Write" : Write a hex value to the remote handle
  - "Write no rsp" : Write a hex value without a response to the remote handle
  - "Value Notify" : Write a notification value to the remote handle
  - "Value Indicate" : Write an indication value to the remote handle

HFP Hands-free Unit:

- Targets CYW920721M2EVK-01 CYW920721M2EVK-02 CYW920721M2EVB-03 and CYW9M2BASE-43012BT support HFP Hands-free Unit by default.
- To create a hands-free connection with a remote Audio Gateway (AG) device (such as a mobile phone), use ClientControl and choose the Bluetooth&#174; address of the remote AG device from the BR/EDR combo box.<br/>
  Click the "Connect" button under HF tab.
- OR Put the device in discoverable and connectable mode and search for the device from the AG device and connect.
- The following HF operations can be performed using the ClientControl HF tab
   -  Connect / Disconnect the HF or SCO connection
   -  Answer / Hang-up the call
   -  Dial / Redial the number
   -  Control Held calls ( Only support "Release all held", "Release active accept other", "Place active on hold and accept other", "Add held to conversation( It’s functionality depends on the telecom network operator, if the telecom network side support the feature, the function will work. AG always supports this feature and responses OK)")
   -  Mic / Speaker gain control

AMA Service:

- The Watch AMA application can support the voice recognition service
  provided by Amazon after pairing with the Alexa app on a mobile device.
- Pairing
  - Switch to the GATT tab -> Click the Start Adverts button.
  - Open the Alexa app -> Switch to the Devices page -> Click the + icon on the top-right corner.
  - Select Add Device -> Select Speaker or Headphones -> Select WatchAmaLE.
- Voice Recognition
  - Switch to the AVRC CT tab -> Click the Long Press button.
- The AMS and ANCS should be connected incidentally after pairing with an iPhone by the Alexa app.

## BTSTACK version

BTSDK AIROC&#8482; chips contain the embedded AIROC&#8482; Bluetooth&#174; stack, BTSTACK. Different chips use different versions of BTSTACK, so some assets may contain variant sets of files targeting the different versions in COMPONENT\_btstack\_vX (where X is the stack version). Applications automatically include the appropriate folder using the COMPONENTS make variable mechanism, and all BSPs declare which stack version should be used in the BSP .mk file, with a declaration such as:<br>
> COMPONENTS+=btstack\_v1<br>
or:<br>
> COMPONENTS+=btstack\_v3

## Common application settings

Application settings below are common for all BTSDK applications and can be configured via the makefile of the application or passed in via the command line.

##### BT\_DEVICE\_ADDRESS
> Set the BDA (Bluetooth&#174; Device Address) for your device. The address is 6 bytes, for example, 20819A10FFEE. By default, the SDK will set a BDA for your device by combining the 7 hex digit device ID with the last 5 hex digits of the host PC MAC address.

##### UART
> Set to the UART port you want to use to download the application. For example 'COM6' on Windows or '/dev/ttyWICED\_HCI\_UART0' on Linux or '/dev/tty.usbserial-000154' on macOS. By default, the SDK will auto-detect the port.

##### ENABLE_DEBUG
> For HW debugging, configure ENABLE\_DEBUG=1. See the document [AIROC&#8482;-Hardware-Debugging](https://infineon.github.io/btsdk-docs/BT-SDK/AIROC-Hardware-Debugging.pdf) for more information. This setting configures GPIO for SWD.<br>
>
   - CYW920819EVB-02/CYW920820EVB-02: SWD signals are shared with D4 and D5, see SW9 in schematics.
   - CYBT-213043-EVAL/CYBT-253059-EVAL: SWD signals are routed to P12=SWDCK and P13=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-223058-EVAL/CYW920835M2EVB-01/CYBT-243053-EVAL/CYBLE-343072-EVAL-M2B/CYBLE-333074-EVAL-M2B/CYBLE-343072-MESH/Vela-IF820-INT-ANT-DVK/Vela-IF820-EXT-ANT-DVK: SWD signals are routed to P02=SWDCK and P03=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-263065-EVAL/CYBT-273063-EVAL: SWD signals are routed to P02=SWDCK and P04=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-343026-EVAL/CYBT-353027-EVAL/CYBT-333047-EVAL: SWD signals are routed to P11=SWDCK and P15=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYBT-413055-EVAL/CYBT-413061-EVAL: SWD signals are routed to P16=SWDCK and P17=SWDIO. Use expansion connectors to connect VDD, GND, SWDCK, and SWDIO to your SWD Debugger probe.
   - CYW989820EVB-01: SWDCK (P02) is routed to the J13 DEBUG connector, but not SWDIO. Add a wire from J10 pin 3 (PUART CTS) to J13 pin 2 to connect GPIO P10 to SWDIO.
   - CYW920719B2Q40EVB-01: PUART RX/TX signals are shared with SWDCK and SWDIO. Remove RX and TX jumpers on J10 when using SWD. PUART and SWD cannot be used simultaneously on this board unless these pins are changed from the default configuration.
   - CYW920721M2EVK-02/CYW920721M2EVB-03: The default setup uses P03 for SWDIO and P05 for SWDCK. Check the position of SW15 if using JLink with the DEBUG connector.
   - CYW920706WCDEVAL: SWD debugging requires fly-wire connections. The default setup P15 (J22 pin 3 or J24 pin 1) for SWDIO and P11 (J23 pin 5
    or J22 pin 4) for SWDCK.
   - CYW920736M2EVB-01: SWD hardware debugging requires fly-wire connections. The only option is using P14 for SWDCK and P15 for SWDIO. These route to Arduino header J2, A1 and A0. These can be fly-wired to Arduino header J4, D4 and D5. From there the signals connect to the KitProg3 SWD bridge. In addition, the debug macros (SETUP\_APP\_FOR\_DEBUG\_IF\_DEBUG\_ENABLED and BUSY\_WAIT\_TILL\_MANUAL\_CONTINUE\_IF\_DEBUG\_ENABLED) are placed in sparinit.c in code common to all applications for this device. Most applications for this device call bleprofile\_GPIOInit() in subsequent code, overwriting the SWD pin configuration. To use hardware debugging after the call to bleprofile\_GPIOInit(), place the debug macros in code after that call.
   - CYW943012B2EVK-01: SWD signals are shared with D4 and D5.
   - CYW920820M2EVB-01 & CYW920819M2EVB-01: The default setup uses P03 for SWDIO and P02 for SWDCK. Check the position of SW15 if using JLink with the DEBUG connector.
   - CYW989820M2EVB-01: SWD hardware debugging requires a fly-wire connection to use P14 for SWDIO. P2 is connected directly to SWDCK / ARD_D4. Fly-wire P14 / ARD_D8 on J3.10 to J4.3 / ARD_D5 to connect SWDIO.

   - SWD hardware debugging is not supported on the following:
      - CYW920721M2EVK-01
      - CYW920835REF-RCU-01
      - CYW9M2BASE-43012BT
      - CYBT-423054-EVAL
      - CYBT-423060-EVAL
      - CYBT-483056-EVAL
      - CYBT-483062-EVAL
      - CYW955572BTEVK-01
      - CYW943022BTEVK-01

##### DIRECT_LOAD
> BTSDK chips support downloading applications either to FLASH storage or to RAM storage. Some chips support only one or the other, and some chips support both.

> If a chip only supports one or the other, this variable is not applicable, applications will be downloaded to the appropriate storage supported by the device.

> If a chip supports both FLASH and RAM downloads, the default is to download to FLASH, and the DIRECT_LOAD make variable may be set to 1 in the application makefile (or in the command line make command) to override the default and download to RAM.

> Currently, the following chips support both FLASH and RAM download and can set DIRECT_LOAD=1 if desired:
>
   - CYW20835
   - CYW20706

## Building and downloading code examples

**Using the ModusToolbox&#8482; Eclipse IDE**

1. Install ModusToolbox&#8482; 2.4.1 (or higher).
2. In the ModusToolbox&#8482; Eclipse IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**).
3. Pick your board for BTSDK under AIROC&#8482; Bluetooth&#174; BSPs.
4. Select the application in the IDE.
5. In the Quick Panel, select **Build** to build the application.
6. To program the board (download the application), select **Program** in the Launches section of the Quick Panel.

**Using command line**

1. Install ModusToolbox&#8482; 2.4.1 (or higher).
2. On Windows, use Cygwin from \ModusToolbox\tools_2.x\modus-shell\Cygwin.bat to build apps.
3. Use the tool 'project-creator-cli' under \ModusToolbox\tools_2.x\project-creator\ to create your application.<br/>
   > project-creator-cli --board-id (BSP) --app-id (appid) -d (dir) <br/>
   See 'project-creator-cli --help' for useful options to list all available BSPs, and all available apps per BSP.<br/>
   For example:<br/>
   > project-creator-cli --app-id mtb-example-btsdk-empty --board-id CYW920706WCDEVAL -d .<br/>
4. To build the app call make build. For example:<br/>
   > cd mtb-examples-btsdk-empty<br/>
   > make build<br/>
5. To program (download to) the board, call:<br/>
   > make qprogram<br/>
6. To build and program (download to) the board, call:<br/>
   > make program<br/><br>
   Note: make program = make build + make qprogram

If you have issues downloading to the board, follow the steps below:

- Press and hold the 'Recover' button on the board.
- Press and hold the 'Reset' button on the board.
- Release the 'Reset' button.
- After one second, release the 'Recover' button.

Note: this is only applicable to boards that download application images to FLASH storage. Boards that only support RAM download (DIRECT_LOAD) such as CYW9M2BASE-43012BT or CYW943022BTEVK-01 can be power cycled to boot from ROM.

## Over The Air (OTA) Firmware Upgrade
Applications that support OTA upgrade can be updated via the peer OTA app in:<br>
>\<Workspace Dir>\mtb\_shared\wiced\_btsdk\tools\btsdk-peer-apps-ota<br>

See the readme.txt file located in the above folder for instructions.<br>
To generate the OTA image for the app, configure OTA\_FW\_UPGRADE=1 in the app
makefile, or append OTA\_FW\_UPGRADE=1 to a build command line, for example:
> make PLATFORM=CYW920706WCDEVAL OTA\_FW\_UPGRADE=1 build<br>

This will the generate \<app>.bin file in the 'build' folder.

## SDK software features

- Dual-mode Bluetooth&#174; stack included in the ROM (BR/EDR and LE)
- Bluetooth&#174; stack and profile level APIs for embedded Bluetooth&#174; application development
- AIROC&#8482; HCI protocol to simplify host/MCU application development
- APIs and drivers to access on-board peripherals
- Bluetooth&#174; protocols include GAP, GATT, SMP, RFCOMM, SDP, AVDT/AVCT, LE Mesh
- LE and BR/EDR profile APIs, libraries, and sample apps
- Support for Over-The-Air (OTA) upgrade
- Device Configurator for creating custom pin mapping
- Bluetooth&#174; Configurator for creating LE GATT Database
- Peer apps based on Android, iOS, Windows, etc. for testing and reference
- Utilities for protocol tracing, manufacturing testing, etc.
- Documentation for APIs, datasheets, profiles, and features
- BR/EDR profiles: A2DP, AVRCP, HFP, HSP, HID, SPP, MAP, PBAP, OPP
- LE profiles: Mesh profiles, HOGP, ANP, BAP, HRP, FMP, IAS, ESP, LE COC
- Apple support: Apple Media Service (AMS), Apple Notification Center Service (ANCS), iBeacon, Homekit, iAP2
- Google support: Google Fast Pair Service (GFPS), Eddystone
- Amazon support: Alexa Mobile Accessories (AMA)

Note: this is a list of all features and profiles supported in BTSDK, but some AIROC&#8482; devices may only support a subset of this list.

## List of boards available for use with BTSDK

- [CYW20819A1 chip](https://github.com/Infineon/20819A1)
    - [CYW920819EVB-02](https://github.com/Infineon/TARGET_CYW920819EVB-02), [CYW920819M2EVB-01](https://github.com/Infineon/TARGET_CYW920819M2EVB-01), [CYBT-213043-EVAL](https://github.com/Infineon/TARGET_CYBT-213043-EVAL), [CYBT-223058-EVAL](https://github.com/Infineon/TARGET_CYBT-223058-EVAL), [CYBT-263065-EVAL](https://github.com/Infineon/TARGET_CYBT-263065-EVAL), [CYBT-273063-EVAL](https://github.com/Infineon/TARGET_CYBT-273063-EVAL)
- [CYW20820A1 chip](https://github.com/Infineon/20820A1)
    - [CYW920820EVB-02](https://github.com/Infineon/TARGET_CYW920820EVB-02), [CYW989820M2EVB-01](https://github.com/Infineon/TARGET_CYW989820M2EVB-01), [CYW989820EVB-01](https://github.com/Infineon/TARGET_CYW989820EVB-01), [CYBT-243053-EVAL](https://github.com/Infineon/TARGET_CYBT-243053-EVAL), [CYBT-253059-EVAL](https://github.com/Infineon/TARGET_CYBT-253059-EVAL), [CYW920820M2EVB-01](https://github.com/Infineon/TARGET_CYW920820M2EVB-01), [Vela-IF820-INT-ANT-DVK](https://github.com/Infineon/TARGET_Vela-IF820-INT-ANT-DVK), [Vela-IF820-EXT-ANT-DVK](https://github.com/Infineon/TARGET_Vela-IF820-EXT-ANT-DVK)
- [CYW20721B2 chip](https://github.com/Infineon/20721B2)
    - [CYW920721M2EVK-01](https://github.com/Infineon/TARGET_CYW920721M2EVK-01), [CYW920721M2EVK-02](https://github.com/Infineon/TARGET_CYW920721M2EVK-02), [CYW920721M2EVB-03](https://github.com/Infineon/TARGET_CYW920721M2EVB-03), [CYBT-423060-EVAL](https://github.com/Infineon/TARGET_CYBT-423060-EVAL), [CYBT-483062-EVAL](https://github.com/Infineon/TARGET_CYBT-483062-EVAL), [CYBT-413061-EVAL](https://github.com/Infineon/TARGET_CYBT-413061-EVAL)
- [CYW20719B2 chip](https://github.com/Infineon/20719B2)
    - [CYW920719B2Q40EVB-01](https://github.com/Infineon/TARGET_CYW920719B2Q40EVB-01), [CYBT-423054-EVAL](https://github.com/Infineon/TARGET_CYBT-423054-EVAL), [CYBT-413055-EVAL](https://github.com/Infineon/TARGET_CYBT-413055-EVAL), [CYBT-483056-EVAL](https://github.com/Infineon/TARGET_CYBT-483056-EVAL)
- [CYW20706A2 chip](https://github.com/Infineon/20706A2)
    - [CYW920706WCDEVAL](https://github.com/Infineon/TARGET_CYW920706WCDEVAL), [CYBT-353027-EVAL](https://github.com/Infineon/TARGET_CYBT-353027-EVAL), [CYBT-343026-EVAL](https://github.com/Infineon/TARGET_CYBT-343026-EVAL), [CYBT-333047-EVAL](https://github.com/Infineon/TARGET_CYBT-333047-EVAL)
- [CYW20835B1 chip](https://github.com/Infineon/20835B1)
    - [CYW920835REF-RCU-01](https://github.com/Infineon/TARGET_CYW920835REF-RCU-01), [CYW920835M2EVB-01](https://github.com/Infineon/TARGET_CYW920835M2EVB-01), [CYBLE-343072-EVAL-M2B](https://github.com/Infineon/TARGET_CYBLE-343072-EVAL-M2B), [CYBLE-333074-EVAL-M2B](https://github.com/Infineon/TARGET_CYBLE-333074-EVAL-M2B), [CYBLE-343072-MESH](https://github.com/Infineon/TARGET_CYBLE-343072-MESH)
- [CYW43012C0 chip](https://github.com/Infineon/43012C0)
    - [CYW9M2BASE-43012BT](https://github.com/Infineon/TARGET_CYW9M2BASE-43012BT), [CYW943012BTEVK-01](https://github.com/Infineon/TARGET_CYW943012BTEVK-01)
- [CYW43022C1 chip](https://github.com/Infineon/43022C1)
    - [CYW943022BTEVK-01](https://github.com/Infineon/TARGET_CYW943022BTEVK-01)
- [CYW20736A1 chip](https://github.com/Infineon/20736A1)
    - [CYW920736M2EVB-01](https://github.com/Infineon/TARGET_CYW920736M2EVB-01)
- [CYW30739A0 chip](https://github.com/Infineon/30739A0)
    - [CYW930739M2EVB-01](https://github.com/Infineon/TARGET_CYW930739M2EVB-01)
- [CYW55572A1 chip](https://github.com/Infineon/55572A1)
    - [CYW955572BTEVK-01](https://github.com/Infineon/TARGET_CYW955572BTEVK-01)


## Folder structure

All BTSDK code examples need the 'mtb\_shared\wiced\_btsdk' folder to build and test the apps. 'wiced\_btsdk' includes the 'dev-kit' and 'tools' folders. The contents of the 'wiced\_btsdk' folder will be automatically populated incrementally as needed by the application being used.

**dev-kit**

This folder contains the files that are needed to build the embedded Bluetooth&#174; apps.

* baselib: Files for chips supported by BTSDK. For example CYW20819, CYW20719, CYW20706, etc.

* bsp: Files for BSPs (platforms) supported by BTSDK. For example CYW920819EVB-02, CYW920706WCDEVAL etc.

* btsdk-include: Common header files needed by all apps and libraries.

* btsdk-tools: Build tools needed by BTSDK.

* libraries: Profile libraries used by BTSDK apps such as audio, LE, HID, etc.

**tools**

This folder contains tools and utilities need to test the embedded Bluetooth&#174; apps.

* btsdk-host-apps-bt-ble: Host apps (Client Control) for LE and BR/EDR embedded apps, demonstrates the use of AIROC&#8482; HCI protocol to control embedded apps.

* btsdk-host-peer-apps-mesh: Host apps (Client Control) and Peer apps for embedded Mesh apps, demonstrates the use of AIROC&#8482; HCI protocol to control embedded apps, and configuration and provisioning from peer devices.

* btsdk-peer-apps-ble: Peer apps for embedded LE apps.

* btsdk-peer-apps-ota: Peer apps for embedded apps that support Over The Air Firmware Upgrade.

* btsdk-utils: Utilities used in BTSDK such as BTSpy, wmbt, and ecdsa256.

See README.md in the sub-folders for more information.

## Software Tools
The following tool applications are installed on your computer either with ModusToolbox&#8482;, or by creating an application in the workspace that can use the tool.

**BTSpy:**<br>
>   BTSpy is a trace viewer utility that can be used with AIROC&#8482; Bluetooth&#174; platforms to
    view protocol and application trace messages from the embedded device. The
    utility is located in the folder below. For more information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel, or by right-clicking
    a project in the Project Explorer pane and selecting the ModusToolbox&#8482;
    context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Location:  \<Workspace Dir>\wiced_btsdk\tools\btsdk-utils\BTSpy

**Bluetooth&#174; Classic and LE Profile Client Control:**<br>
>   This application emulates host MCU applications for LE and BR/EDR profiles.
    It demonstrates AIROC&#8482; Bluetooth&#174; APIs. The application communicates with embedded
    apps over the "WICED HCI UART" interface. The application is located in the folder
    below. For more information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel, or by right-clicking
    a project in the Project Explorer pane and selecting the ModusToolbox&#8482;
    context menu.<br>
    It is supported on Windows, Linux, and macOS.<br>
    Location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-host-apps-bt-ble\client_control

**LE Mesh Client Control:**<br>
>   Similar to the above app, this application emulates host MCU applications
    for LE Mesh models. It can configure and provision mesh devices and create
    mesh networks. The application is located in the folder below. For more
    information, see readme.txt in the same folder.<br>
    This utility can be run directly from the filesystem, or it can be run from
    the Tools section of the ModusToolbox&#8482; QuickPanel (if a mesh-capable
    project is selected in the Project Explorer pane), or by right-clicking
    a mesh-capable project in the Project Explorer pane and selecting the
    ModusToolbox&#8482; context menu.<br>
    The full version is provided for Windows (VS\_ClientControl) supporting all
    Mesh models.<br>
    A limited version supporting only the Lighting model (QT\_ClientControl) is
    provided for Windows, Linux, and macOS.<br>
    Location:  \<Workspace Dir>\wiced_btsdk\tools\btsdk-host-peer-apps-mesh\host

**Peer apps:**<br>
>   Applications that run on Windows, iOS or Android and act as peer Bluetooth&#174; apps to
    demonstrate specific profiles or features, communicating with embedded apps
    over the air.<br>
    LE apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-peer-apps-ble<br>
    LE Mesh apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-host-peer-apps-mesh\peer<br>
    OTA apps location:  \<Workspace Dir>\wiced\_btsdk\tools\btsdk-peer-apps-ota

**Device Configurator:**<br>
>   Use this GUI tool to create source code for a custom pin mapping for your device. Run this tool
    from the Tools section of the ModusToolbox&#8482; QuickPanel, or by
    right-clicking a project in the Project Explorer pane and selecting the
    ModusToolbox&#8482; context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Note: The pin mapping is based on wiced\_platform.h for your board.<br>
    Location:  \<Install Dir>\tools_2.x\device-configurator

Note: Not all BTSDK chips support Device Configurator. BSPs using the following devices do not currently support Device Configurator: CYW20706, CYW20736

**Bluetooth&#174; Configurator:**<br>
>   Use this GUI tool to create and configure the LE GATT Database and the BR/EDR SDP Database, generated as source code for your
    application.<br>
    Run this tool from the Tools section of the ModusToolbox&#8482; QuickPanel, or
    by right-clicking a project in the Project Explorer pane and selecting
    the ModusToolbox&#8482; context menu.<br>
    It is supported on Windows, Linux and macOS.<br>
    Location:  \<Install Dir>\tools_2.x\bt-configurator

## Tracing
To view application traces, there are 2 methods available. Note that the
application needs to configure the tracing options.<br>

1. "WICED Peripheral UART" - Open this port on your computer using a serial port
utility such as TeraTerm or PuTTY (usually using 115200 baud rate for non-Mesh apps, and 921600 for Mesh apps).<br>
2. "WICED HCI UART" - Open this port on your computer using the Client Control
application mentioned above (usually using 3M baud rate). Then run the BTSpy
utility mentioned above.

## Using BSPs (platforms)

BTSDK BSPs are located in the \mtb\_shared\wiced\_btsdk\dev-kit\bsp\ folder by default.

#### a. Selecting an alternative BSP

The application makefile has a default BSP. See "TARGET". The makefile also has a list of other BSPs supported by the application. See "SUPPORTED_TARGETS". To select an alternative BSP, use Library Manager from the Quick Panel to deselect the current BSP and select an alternate BSP. Then right-click the newly selected BSP and choose 'Set Active'.  This will automatically update TARGET in the application makefile.

#### b. Custom BSP

To create a custom BSP from a BSP template for BTSDK devices, see the following KBA article: [KBA238530](https://community.infineon.com/t5/Knowledge-Base-Articles/Create-custom-BTSDK-BSP-using-ModusToolbox-version-3-x-KBA238530/ta-p/479355)

## Using libraries

The libraries needed by the app can be found in in the mtb\_shared\wiced\_btsdk\dev-kit\libraries folder. To add an additional library to your application, launch the Library Manager from the Quick Panel to add a library. Then update the makefile variable "COMPONENTS" of your application to include the library. For example:<br/>
   COMPONENTS += fw\_upgrade\_lib


## Documentation

BTSDK API documentation is available [online](https://infineon.github.io/btsdk-docs/BT-SDK/index.html)

Note: For offline viewing, git clone the [documentation repo](https://github.com/Infineon/btsdk-docs)

BTSDK Technical Brief and Release Notes are available [online](https://community.infineon.com/t5/Bluetooth-SDK/bd-p/ModusToolboxBluetoothSDK)

<br>
<sup>The Bluetooth&#174; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.</sup>
