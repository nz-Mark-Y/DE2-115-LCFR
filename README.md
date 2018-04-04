# DE2-115-LCFR #
Low Cost Frequency Array implemented using Free RTOS on  a Nios II software processor. 
Manages 8 loads by monitoring frequency and frequency rate of change. Loads can also be manually turned on or off as per specifications.

## Requirements ##
* Quartus II version 13.0 or above
* Nios II Software Build Tools (Eclipse)
* DE2-115 development board and cables (USB and power)
* PS/2 Keyboard
* VGA cable

## Setup ##
1. Clone this repository to a local directory. Ensure its absolute path does not include any spaces.
2. Configure the DE2-115 board using the Quartus II Programmer and the .sof file in this repository.
3. Build and run the LCFR project in the software folder.

## Usage ##
1. Slide switches SW0 to SW7 represent the switches for the loads. The down position is off and the up position is on.
2. Red LEDs G0 to G7 represent the loads. If the LED is on then that corresponding load is on.
3. Green LEDs R0 to R7 represent the inverted loads. If the LED is on then that corresponding load is off.
4. KEY 3 is the push button which toggles the system between maintenance mode and regular mode. In maintenance mode all of the green LEDs will be off, irrespective of the red LEDs. The console will also display a message saying that the system is in maintenance mode. In this mode the PS2 keyboard can be used to input data.
5. In maintenance mode, use the numberpad of the keyboard to enter numbers (digits 0-9, and decimal point). Pressing ENTER will store the inputted number as either minimum allowable frequency or maximum allowable frequency rate of change. Pressing any other key or an invalid decimal point will be stored as a 0.
6. The first number entered will be stored as minimum allowable frequency. The second number entered will be stored as maximum allowable frequency rate of change. If a third number is entered then it will be stored as minimum allowable frequency - and so on, the value being written to is toggled on each ENTER press.
