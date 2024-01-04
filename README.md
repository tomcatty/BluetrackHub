# Introduction
This project is a Bluetooth controlled Digital Command Control (DCC) Center for model trains of N and HO gauge. See https://www.nmra.org/index-nmra-standards-and-recommended-practices for the DCC standard. It also provides 15x relay outputs for control of switches/signals etc on a layout.

See below for a demonstration with the companion BlueTrack Controller app (link pending):

[![Video](https://img.youtube.com/vi/xS7GTewaBds/maxresdefault.jpg)](https://www.youtube.com/watch?v=xS7GTewaBds)

It runs on the BlueTrack Hub hardware available from Seeed Fusion Gallery (link pending), which incorporates the nRF52833 System On Chip (SoC). 

> [!CAUTION]
> This is for hobbyist use only, no claims of safety or functionality are made. 

# Development Environment
See the nRF52 DK user guide at https://infocenter.nordicsemi.com/topic/ug_nrf52832_dk/UG/dk/intro.html and SES getting started guide at https://infocenter.nordicsemi.com/topic/ug_gsg_ses/UG/gsg/intro.html. 

The following versions were used to build this project:
* SES V7.32 (sign up for licence at http://license.segger.com/Nordic.cgi)
* J-Link Software and Documentation pack for MacOS V7.92e (note M1 needs to use Universal)
* https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools V10.23.0, the incorporated J-Link Software and Documentation pack was not used
* https://www.nordicsemi.com/Products/Development-tools/nrf-util V7.6.0
  * Note the required packages require installation of rf5sdk-tools (V1.0.1 used) on the command line

# Building the Project
## Generate complete .hex file for programming

Compile the `nRF5SDK160098a08e2/examples/ble_peripheral/ble_app_bluetrack/pca10040/s132/ses` and `nRF5SDK160098a08e2/examples/dfu/secure_bootloader_bluetrack/pca10040_s132_ble_debug/ses` project in SES, using the Release configuration.

From root of the project, execute the following:

1. `nrfutil nrf5sdk-tools settings generate --family NRF52 --application nRF5SDK160098a08e2/examples/ble_peripheral/ble_app_bluetrack/pca10040/s132/ses/Output/Release/Exe/ble_app_bluetrack_pca10040_s132.hex --application-version 1 --bootloader-version 2 --bl-settings-version 2 --app-boot-validation NO_VALIDATION --sd-boot-validation NO_VALIDATION --softdevice nRF5SDK160098a08e2/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex settings.hex`
1. `mergehex --merge nRF5SDK160098a08e2/examples/ble_peripheral/ble_app_bluetrack/pca10040/s132/ses/Output/Release/Exe/ble_app_bluetrack_pca10040_s132.hex nRF5SDK160098a08e2/examples/dfu/secure_bootloader_bluetrack/pca10040_s132_ble_debug/ses/Output/Release/Exe/secure_bootloader_bluetrack_ble_s132_pca10040_debug.hex nRF5SDK160098a08e2/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex settings.hex --output bluetrack.hex`

## Generate .zip file for DFU

From root of the project, execute the following:

`nrfutil nrf5sdk-tools pkg generate --application  nRF5SDK160098a08e2/examples/ble_peripheral/ble_app_bluetrack/pca10040/s132/ses/Output/Release/Exe/ble_app_bluetrack_pca10040_s132.hex --application-version 1 --bootloader nRF5SDK160098a08e2/examples/dfu/secure_bootloader_bluetrack/pca10040_s132_ble_debug/ses/Output/Release/Exe/secure_bootloader_bluetrack_ble_s132_pca10040_debug.hex --bootloader-version 2 --hw-version 2 --sd-req 0xCB --sd-id 0xCB --softdevice nRF5SDK160098a08e2/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex --key-file priv.pem hw_2_sd_0xCB_bl_2_app_1.zip`

> [!NOTE]
> I acknowledge it is bad practice to place a private key in a public repository, this is included to allow others to continue developing the iOS application. 

## Hardware 

## Programming the Hardware

1. Ensure the BlueTrack Hub is powered.
1. Connect the nRF52 DK to your PC ane power it up by ensuring the Power switch (SW6) is set to ON.
1. Connect the IDE cable from Debug out (P19) on the nRF52 to DEBUG_IN (P600) on the BlueTrack Hub (see picture below).

> [!IMPORTANT]
> Make sure the IDE cable pinout is correct when connecting to DEBUG_IN (P600) on the BlueTrack Hub, see picture below.

![`BlueTrackProgrammingConnection`](resources/ProgrammingSetup.jpg)

From the root of the project, execute the following (note `--recover` is specified as the BMD-350-A-R comes with protection enabled):

`nrfjprog --program bluetrack.hex --recover --verify --reset`

# Design Description
This project is based on nRF SDK 16.0.0 and uses the S132 Softdevice 7.0.1, and was originally branched from the ble_blinky sample project. All changes from SDK 16.0.0 can be identified by comparing to the initial commit.
Below are some specific notes on on aspects of the design.

## Hardware Description
### Assembly
See CircuitMaker project (link pending) for full design, including schematic. It can be assembled by Seeed (link pending) using the design files and BOM in the Seeed Fusion Gallery. Additional assembly that may be required depending on what can be arranged with Seeed:
* Attachment of the motor drivers (LMD18200T/NOPB) to the heatsinks (6396BG) using attachment kit 4880G, this needs to be done prior to through hole soldering.
* Ensure all links prescribed by the schematic are installed (these are included in the BOM)
* Optional assembly into a box (items listed in Notes in BOM in the Seeed Fusion Gallery)
* Wall wart power supply needs to be purchased (WSX135-1770 for N gauge, WSX150-1600 for HO)
* nRF52 DK and IDE cable (NRF52833-DK, FFSD-05-D-06.00-01-N) need to be purchased to program the board

### nRF52 SOC
The precertified BMD-350-A-R module is implemented, and the internal LFCLK source (RC circuit) is used so no external crystal design is required. 

### DCC Drivers
A LMD18200 motor driver H bridge provides the DCC signal to the main and programming track. The MAIN DCC output is directed to a RJ-H / RJ9 connector (https://ncedcc.zendesk.com/hc/en-us/articles/201799499-NCE-DCC-Cables-Explained) to connect to a NCE Control Bus (note, not Cab Bus!) Cable to interface with NCE Boosters (PB5 or PB105). Red and green are the center contacts. As per https://sites.google.com/site/markgurries/home/technical-discussions/boosters/mixing-booster-brands and S-9.1.2, the 12V bipolar DCC output is sufficient to drive into a booster. NCE even recommends it for many boosters (https://ncedcc.zendesk.com/hc/en-us/articles/200590789-Booster-Diagrams). The user can of course rewire the cable to work with Digitrax if they so wish (https://ncedcc.zendesk.com/hc/en-us/article_attachments/200486819/Booster_Connections.pdf). 

Both motor drives have associated 

### Power Supply
Either a 15V or 13.5V wall wart power supply can be used. LMD18200T has Rdson of 0.33ohms/0.4ohms typical/maximum, for 3A current this give a drop of 1.98V/2.4V, for 0.5A this gives 0.33V/0.4V. Target N gauge voltage at the rails is 12V, and for HO is 15V (complies with S-9.1), this requires a ~14V and ~17V power supply. HO is generally accepted as 14V, so a ~16V supply is required. However Digitrax only uses ~15V for HO (https://www.digitrax.com/media/apps/products/command-stations-boosters/db210/documents/DB210%2BDB220_rev0.pdf), and only provides a 15V supply for HO running (https://www.digitrax.com/products/starter-sets/evox/, https://www.digitrax.com/products/power-supplies/ps615/). See Digitrax manual for how the output voltages are set https://www.digitrax.com/media/apps/products/command-stations-boosters/db210/documents/DB210%2BDB220_rev0.pdf on this product.

The internally generated supply voltage is 3V to allow for programming by the nRF52 DK and is generated by an LDO. Shown below is the typical and maximum current. The typical current excludes the yellow and red LEDs, as well as the green LED and relay current. 

| Component                   | Current (mA) |
| --------------------------- | ------------ |
| Resistor Ladder             | 1.154767796  |
| Orange LED                  | 15           |
| Red LED (x2)                | 30           |
| Yellow LED                  | 15           |
| Blue LED                    | 4            |
| Green LED + Relay           | 35           |
| nRF52                       | 7.1          |
| Brake                       | 1.726505321  |
| DCC (50% duty cycle, x2)    | 1.726505321  |
| Maximum                     | 110.7077784  |
| Typical                     | 30.70777844  |

## ble_app\_bluetrack
### Flash Memory
Space is reserved for the bootloader and bootloader data to ensure the compiled .hex does not overwrite the bootloader.

### Main Loop
The main loop consists of the inbuilt power management function (which also allows log output to be flushed out) as well as kicking the scheduler.

### Debug Output
NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED has to be set to 0 to allow NRF_LOG to work with Segger RTT https://devzone.nordicsemi.com/f/nordic-q-a/46685/no-log-output-to-debug-terminal-in-ses-or-rtt-viewer

### GIO Configuration
The application uses the gpiote hardware driver to manage the interfaces. 
Every output is under control of the CPU except the advertising LED which is under control of gpiote tasks, the main track DCC output, and the programming track DCC output (under PWM control). 
The THERMAL_N input generates an interrupt which disables the DCC and notifies an error. This must be checked after interrupt setup in case it was low before setup.

### Timer Configuration
One hardware timer is used for the advertising LED. The application timer uses the scheduler.
The advertising LED timer has a dummy handler which is never enabled.

### PPI Configuration
The PPI is used to link the timers to the gpiote tasks. 
On BLE connection, the PPI channel for the advertising LED is disabled and the LED is forced on. On BLE connection, the PPI channel is re-enabled.

### PWM Configuration
Two PWM modules are used for the main and programming outputs. The PWM modules draw data from RAM using EasyDMA, which frees up the CPU. At the end of the sequence, the modules output the last PWM setting, so make sure this is a DCC “one”. On initialisation and first start a single “one” is populated, this is just so the handlers fire and take over the DCC control. 

### ADC Configuration
ADC is set to 1/3 gain with 0.6V reference and 14 bit resolution.
10us sampling time is fine as the source resistance is 1.74kohm.

|            | Current Output Sensitivity (uA/A)    | Pin Input Sensitivity (V/A)    | ADC Input Sensitivity (count/A)    | ADC Input Resolution (mA/count) | 
| ---------- | ------------------------------------ | ------------------------------ | ---------------------------------- | ------------------------------- |
| Minimum    | 300                                  | 0.522                          | 4751.36                            | 0.210466056 | 
| Nominal    | 377                                  | 0.65598                        | 5970.875733                        | 0.16747962 | 
| Maximum    | 450                                  | 0.783                          | 7127.04                            | 0.140310704 | 

The ADC is sampled (by sample_convert) every 500us so we can take at least 10 samples during a 5ms period when collecting feedback (S-9.2.3).
Note buffer_convert was not found to work reliably, root cause unknown (maybe the memory was not statically allocated?).
The 2.2V Zener protection MMSZ4680T1G draws 50uA between 2.09V and 2.31V, and 4uA at 1V, this reduces the measured ADC voltage by a fixed offset, but this is insignificant compared to the sensitivity variation.

### Safety Design
The THERMAL_N signal generates an interrupt which disables the DCC and notifies an error.
The SAADC is set up to sample both MAIN_I_SENSE_PIN and PROG_I_SENSE_PIN with 14 bit resolution. The feedback timer runs approximately every 0.5ms to sample the ADC, and if the sample is above the limit it will disable the DCC and notify an error. 
For 1.6A output current limit (https://www.digikey.com.au/product-detail/en/triad-magnetics/WSX150-1600/237-2145-ND/6165640)

| Rsense (ohms)                                              | 1740 | 
| Gain                                                       | 0.333333333 | 
| Reference (V)                                              | 0.6 | 
| Resolution (bits)                                          | 14 | 
| Maximum Value (counts)                                     | 16383 | 
| Current Limit (A)                                          | 1.6 | 
| Limit (based on minimum sensitivity) (counts)              | 7602 | 
| Limit (based on minimum sensitivity) (V)                   | 0.835180664 | 
| Lowest Current Limit (based on maximum sensitivity) (A)    | 1.066641972 | 

When a safety check trips, the hub enters error mode, which means that the DCC outputs are permanently disabled until the hub goes through a power on reset (i.e. switched on and off), and the error characteristic is notified. The red error LED is solidly lit in this case.

### Stop
The stop characteristic when turned on sets the stop LED as well as turns on the brake. When turned off the stop LED is turned off, and the brake is disengaged (only if an error has not occurred). In either case, the characteristic is notified. This notification is done within the handler. When BLE disconnects, the stop characteristic is set, and the stop LED and brake set.

### Relay Actuation
An application timer fires every 1s, and at each firing it turns off a relay that was turned on the previous firing (if one was turned on), and then turns on a single relay that has been marked to actuate by the address characteristic. This means only a single relay can be actuated at a time to protect the power supply, as well as making sure a single relay is actuated for 1s to make sure the circuit it controls actuates. The relays are designed to connect a 12V AC 0.5A supply.

### DCC Design
When the hub initialises, we do not send 20 reset packets and 10 idle packets, as suggested by S-9.2.4.

#### Programming Track Selection
The hub starts off in programming_track_state = MAIN. The write handler signals the state is to be changed by updating programming_state_mode to either MAIN or PROGRAMMING; a state change is delayed until there are no commands to draw from the buffers. Despite this delay, the PROG_LED_PIN is updated (on for PROGRAMMING, off for MAIN) and the characteristic notified immediately. A check for 250mA current sustained for 100ms upon entering programming mode is not performed, which is optional (should) in S-9.2.3. When BLE disconnects, the hub sets programming_track_mode = MAIN.

#### DCC Command Write Handler
The DCC command write handler processes incoming DCC commands only when neither service_command_pending or service_command_in_progress are raised, this is to ensure the dcc_command_buffer clears to eventually permit a service command to be executed, and once the service command is started this prevents dcc_command_buffer overflowing if new commands are sent when a service command is in progress. The write handler schedules the enqueueing of the DCC command. This scheduled task does the following:
* Repeats the command 10 times, and obtains a copy of the command sent out on the rails
* If we have a reset or hard reset packet in operations mode sent to broadcast, short, or long addresses, queue up 10 idle packets to ensure the decoders don't enter service mode unintentionally
* If programming_track_mode == MAIN (i.e. we are in or about to be in MAIN programming_track_state), update periodic speed commands:
  * If it is a stop command, set occupied to false on the speed_command_array as well as the speed_command_array_temp index
    * If this speed command had just been drawn by the DCC packet transmitted handler, it is guaranteed that the enqueued stop command will come after this speed command, if it is just about to be drawn it will not be drawn as occupied is now false
    * If this was a broadcast address, remove all periodic speed commands
  * If it is a speed command, update both speed_command_array and speed_command_array_temp
    * Occupied is lowered before updating, and set only when the data is valid, so it is safe if this update is interrupted by the DCC packet transmitted handler which repopulates speed_command_array_temp
  * If there is not enough space in the speed command array, an error is raised
Note that all periodic speed commands are cleared on BLE disconnect.
If there is no room in the queue, the command is dropped silently.
#### Service Command Write Handler
Service commands are only executed when the command buffer is empty. The write handler simply stores the command in global variables and raises service_command_pending when a valid command is received, provided neither service_command_pending or service_command_in_progress are already raised.

#### Feedback
If a DCC command packet requires feedback (it is assumed every service command will request feedback, enforced by the service command logic). Once feedback is in progress, no other packet can request feedback until the dcc_command_buffer is empty. When dcc_command_buffer is empty, the feedback window ends. To start a feedback window, the appropriate flags are set. 

Every feedback timer handler, the ADC is always sampled. If feedback is in progress and the feedback window has not ended, the sample is evaluated to determine if the value read is sufficiently above the baseline as to constitute a positive acknowledgement. This is defined as minimum 60mA in S-9.2.3. This baseline is collected in the first ADC sample of the window.

| Rsense (ohms)                                                              | 1740 | 
| Gain                                                                       | 0.333333333 | 
| Reference (V)                                                              | 0.6 | 
| Resolution (bits)                                                          | 14 | 
| Maximum Value (counts)                                                     | 16383 | 
| Minimum Additional Load (specified in S-9.2.3) (mA)                        | 60 | 
| Minimum Delta (based on minimum sensitivity) (counts)                      | 285 | 
| Minimum Detectable Additional Load (based on  maximum sensitivity) (mA)    | 39.98855065 |

If feedback is in progress and the feedback window is ended (i.e. dcc_command_buffer is empty), the service command is advanced. If it was a WRITE service command, acknowledge is simply notified back. If it is a READ service command, for byte read if there was no acknowledge the READ command is resent with the next value, otherwise the acknowledged value is sent back with acknowledge=1. If all READ values have been sent, acknowledge is notified with 0. For bit read, the next bit is read, otherwise if all bits have been read acknowledge is notified with value 1 and the bits are accumulated and sent. After this logic, feedback_in_progress is set to false. When acknowledge is notified, service_command_in_progress is set to false.

#### DCC Packet Transmitted Handler
There are 3 buffers DCC commands are drawn from:
1. dcc_command_buffer: this is populated by the BlueTrack service directly by the dcc_command characteristic, or indirectly by the service_command characteristic, there is a limit of 52 commands that can be queued at a time
1. speed_command_array_temp: this is populated by copying over entries in the speed_command_array whenever the dcc_command_buffer is empty, which in turn is populated whenever a speed command is sent, as determined when populating the dcc_command_buffer from the dcc_command characteristic handler, note the speed_command_array is purged on Bluetooth disconnection. The size of this array limits the number of locomotives that can be supported concurrently, this is set to 128, there is sufficient RAM to support more if required.
1. idle_packet: this is populated whenever all the buffers are empty

This handler is triggered whenever the PWM module finishes transmitting the last DCC packet. There is a separate handler for the main PWM and programming PWM. Several things happen:
* The sync pin is raised at the start of the handler
* Commands are drawn from dcc_command_buffer, speed_command_array_temp, and idle_packet in that order
  * We can only draw commands from dcc_command_buffer when programming_track_state is in MAIN in the main PWM handler, or PROGRAMMING in the programming PWM handler
  * We can only draw commands from speed_command_array_temp in the main PWM handler
    * If there is not a valid packet at the current index, copy over the speed command to the temporary array, and increment the index to the next occupied packet, wrapping around if none is found
  * Idle packets can always be sent
* If the dcc_command_buffer is genuinely empty, we do some housekeeping:
  * If there is a pending service command, schedule its execution
  * Update programming_track_state
    * If programming_track_mode is MAIN, and programming_track_state is not MAIN, switch programming_track_state to MAIN
    * If programming_track_mode is PROGRAMMING, and programming_track_state is not PROGRAMMING, switch programming_track_state to PROGRAMMING
* For the active packet:
  * Feedback is started if it is requested, not already in progress, we have drawn from dcc_command_buffer, and a service command is in progress
    * feedback_in_progress and adc_baseline_flag set for feedback_timer handler and ADC handler
    * feedback_window_end and acknowledge initialised
  * The data is encoded into PWM data structures in RAM for EasyDMA.
    * Note an additional “one” is added at the end to guarantee a “one” is repeated after the PWM sequence finishes, as there may be some time before the handler completes after the sequence
  * The occupied property is cleared
  * If the active packet is from dcc_command_buffer: 
    * update the consumer index to make sure the next command in dcc_command_buffer gets priority over speed_command_array_temp and idle_packet
    * if the dcc_command buffer is now empty and there is a service command in progress, indicate that the feedback window should be ended (note the assumption that service commands always include a feedback command at some point, and the only time feedback is requested is for a service command)

### Bluetooth LE Design
#### Configuration
The app_timer module must be initialised for the SDK to work with the softdevice. 

The BLE stack is configured using the default settings. 

Use internal LFCLK source (RC circuit) so no crystal design is required.

The GAP parameters are set to comply with R12 of Accessory Design Guidelines for Apple Devices. Minimum and maximum connection interval is set to 45ms and 100ms respectively to ensure good responsiveness.

GATT is configured using the default settings. The SERVICE_CHANGED characteristic is enabled (NRF_SDH_BLE_SERVICE_CHANGED = 1). Note that when a BLE_GATTS_EVT_SYS_ATTR_MISSING event is received, the system attributes need to be set to initialise this characteristic. 
SERVICE_CHANGED is important to implement as iOS will often retain the GATT server structure of devices it has connected with (even if there is no bonding). This is implementation specific behaviour. iOS will also enable indications on the SERVICE_CHANGED characteristic. To always ensure the central client has the correct GATT server structure, as soon as the Hub intercepts a BLE_GATTS_EVT_WRITE event to the SERVICE_CHANGED CCCD handle (empirically 0xD on the S112 Softdevice 7.0.1) of length 2 and data 0x0002 (little endian) it will indicate this characteristic. (see https://devzone.nordicsemi.com/f/nordic-q-a/23034/how-do-i-know-if-my-cccd-handles-is-set-and-how-many-cccd-handles-usually-are-there and https://devzone.nordicsemi.com/f/nordic-q-a/1463/how-to-know-if-the-notification-indication-of-a-characteristic-is-enabled). 
Note that another way to “trick” iOS into keeping in sync with the GATT server structure is to “randomise” the BLE address for every connection. As iOS sees a different address, it will “walk the tree” of the GATT server again. This is what is done by Nordic themselves for unbonded DFU (they increment the address for DFU so iOS correctly detects new services when the device exits DFU, see https://devzone.nordicsemi.com/f/nordic-q-a/42411/problems-after-dfu-on-ios) however this is not robust, as you don’t know if that address has already been used by another device.

The connection parameters module is initialised with default parameters.

The Hub simply connects to the central client, and does not bond with it, simply indicating BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP. On connection/disconnection, m_conn_handle is set to a valid value, or invalidated. On disconnection, advertising is restarted.

The BLE_LED is solid when connected, and flashes when disconnected and advertising.

Errors encountered notifying characteristics that are related to the central client not enabling notifications are ignored.

#### Services
There are three services, one custom BlueTrack service, one standard Device Information Service, and one custom unbonded buttonless DFU service (this is termed “experimental" in https://github.com/NordicSemiconductor/IOS-Pods-DFU-Library).
* The BlueTrack service is initialised with the write handlers provided by the application.
* The DIS is initialised with the model name and manufacturer name, and the security is set to open so they can be freely read. 
* All code that writes to the advertising name is IFDEF’ed out in the buttonless DFU service as it is too complicated to compile and adds no value. Any attempt to do this will return an unsupported response to the central. The power management handler is not used in the application as asynchronous resets are not a problem. The DFU handler also doesn’t do anything.

#### Advertising
The hub sends both advertising data and scan response data. 

The advertising data consists of:
* Flags
* Manufacturer data, including:
  * Drekker Development Pty. Ltd. Company identifier
  * 4 byte hardware version (from UICR)
  * 2 byte softdevice version (read from softdevice)
  * 4 byte bootloader version (read from settings file)
  * 4 byte application version (read from settings file)
  * 8 byte device identifier (from the FICR)
The scan response data consists of:
* The device name
* BlueTrack service UUID (as “more available”)

Advertising is always started on application start.

#### Support for Multiple Connections
16 simultaneous peripheral connections are supported. 20 is possible but there is insufficient RAM for this. A maximum GAP event length of 7.5ms is defined in NRF_SDH_BLE_GAP_EVENT_LENGTH (6 * 1.25ms). Using the radio EVENT signal from the softdevice toggling a GIO, this was measured to be less (around 4ms). Therefore, to support 16 connections, the connection interval must be at least 120ms. Minimum of 165ms is selected for some buffer, as the CPU needs to run the DCC and the ADC measurements, and advertising occurs every 152.5ms. This also allows some time for smaller connection intervals on connection with a new device, as the desired connection parameters only take effect 5 seconds after connection (this allows for higher datarates during service and characteristic discovery). A maximum of 180ms is selected for responsiveness, and a timeout of 2s is selected to comply with Apple specifications. A smaller timeout would be more desirable, as in case of collisions the softdevice will make sure a peripheral connection that is about to timeout will get priority, this means a connection may not be serviced until just before the timeout period in case of collisions. 

## secure_bootloader_bluetrack_ble
Based on secure bootloader without bonds. Used the debug version to enable logger functionality. The following modifications are made:
* Does not enter DFU with the button
* New public key
* Service_changed is indicated, and the address is not incremented by +1
* Debug output is fixed as per Bluetrack
* Use internal RC clock for LFCLK
* No application data section
* Advertising interval changed to 20ms to comply with Apple guidelines (other connection settings already comply)
* GPIO is not used as PINRESET (if not set, this GIO can’t be used as a relay)

For this to work, it is required to manually compile uECC. see https://github.com/kmackay/micro-ecc, download ARM, and “make” yourself, this is described in the Nordic SDK documentation. 

The bootloader version is set in the settings file initially by having the bootloader write it when there is invalid settings (which is the case in production, as everything is erased). After this point, bootloader settings is managed by the update process. The bootloader code included in an update will be relied upon to update its settings page if needed.

The hardware version is written to the UICR with the bootloader .hex in production.

Note that nrfutil removes the UICR writes and other memory writes when it creates the .zip file (it cuts it out when it generates the .bin file, see https://devzone.nordicsemi.com/f/nordic-q-a/36877/tool-for-generating-binary-of-dfu-with-real-size).

The bootloader advertises the hardware version, softdevice version, bootloader version, and app version in the manufacturing data, as well as the ID. 

GIO configuration:
* BLE led indicates connected (on) and disconnected (off)
* Programming LED is always lit
* Stop LED is lit when no DFU in progress, and flashes when in progress
* Error LED lights up when something goes wrong
* All other GIO is configured as outputs and turned off (except the brake, which is turned on)

### Hardware Signature
This is disabled for open source release. 

Previously, on bootloader boot, the 64 byte (NRF_CRYPTO_ECDSA_SECP256R1_SIGNATURE_SIZE) hardware signature stored in UICR.CUSTOMER[4] – UICR.CUSTOMER[18] generated from the FICR Device ID is checked against the public key. If this check fails, the device is bricked, and will sit there with the error LED written. This feature exists to make sure no-one can build the hardware without the ability to generate a valid signature with the private key.
SHA256 hashing of the FICR Device ID is used, and the ECDSA used is SECP256R1 (NIST 256-bit).



