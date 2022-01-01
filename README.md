# Brushless_Caliburn

/*
 * Narfduino Firmware
 * Brushless | Solenoid Pusher | Single Trigger | 3-way Select Fire Switch | Serial Configuration
 * 
 * 
 * (c) 2019,2020 Michael Ireland / Ireland Software
 * 
 * Creative Commons: Attribution, Non-Commercial
 * Free for Non-Commercial use
 * 
 * v1.0 - Initial release based on Swordfish firing logic
 * v2.0 - Updated for v4 Firing logic.
 * v2.1 - Increased configuration options to include:
 *        * Select Fire Switch reassignment
 *        * Safe, Ramp, Binary, and Zerg modes
 *        * User-defined battery calibration set-points
 *        * Configurable motor delay and bias
 *        
 * 
 * Notes:
 * * It would be prudent to use a HC-05 bluetooth module. You will need to configure the BT module in AT mode.
 * * Designed for Narfduino. If porting, take care to ensure you have the right dead-time for your fets.
 * 
 */
 
#define PIN_TRIGGER 16   
#define PIN_PUSHER_RUN 5
#define PIN_PUSHER_STOP 15
#define PIN_BATT_MON A7
#define PIN_ESC 10
#define PIN_SELECT_FIRE_A 12
#define PIN_SELECT_FIRE_B 13

Connect to serial port at 57600bps and send ? to display configuration commands.
