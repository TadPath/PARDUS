# Example PCS Programs


This directory contains sample programs written in the PCS programming language. Some programs run completely locally while others require connection to a pardserver and either one or both of an Arduino and a pardcapture server. See each program description below for more details and/or read the .pcs source code for even more information.

Many of the programs are provided as both .pcs source files and also as pre-compiled .dove programs. You can load and run either type of file but the .dove programs are completely static builds meaning they should run without any dependence on external header or library files.

For the very latest updates see the [PARDUS project Codeberg repo](https://codeberg.org/TadPath/PARDUS) 

___


# Notes on the programs

## array_test.pcs
This demonstrates the use of arrays including auto-casting and the ability for recursive dereferencing.

## beep_test.pcs
This demonstrates the use of the non-blocking polyphonic beep command. It shows how to play notes sequentially as well as together as a chord and vary the volume. A more advanced musical application can be seen in the program BWV_114_local.pcs.

## bubblesort.pcs
A classical bubble sort algorithm implemented in PCS to demonstrate recursion and the Turing complete nature of PCS.

## BWV_114_local.pcs
This demonstrates the use of the polyphonic PCS beep command to play music on your local PC sound system with no connection to a pardserver or Arduino required. It is a two-voice arrangement of the "Minuet in G Major" (BWV Anh. 114) composed by the German composer and organist Christian Petzold).

## BWV_114.pcs
This demonstrates the use of pulse-width modulation (PWM) M-level actuators, in the form of piezo buzzers attacked to Arduino, playing music (the same two-voice arrangement of the "Minuet in G Major" played by the script BWV_114_local.pcs).

This was designed for use with an Arduino Nano or Uno (the pin mappings for the buzzers are suited to those boards). If you want to use a Mega2560 you will need to change the pin mappings in the script accordingly (for example use pin 12 on the Mega instead of pin 9 and use pin 10 on the Mega instead of pin 3).

This requires the header files music.pch and stdpard.pch, both of which are included in this directory. It also requires you to have 2 passive piezo buzzers - one connected with its positive lead to Arduino Nano or Uno digital pin 9 (the other lead to ground) and the other with its positive lead connected to digital pin 3 (the other lead to ground). **Don't connect cone speakers directly to Arduino pins or you could permanently damage the Arduino.**

## BWV_861_fugue.pcs
This AI-written script demonstrates the use of (PWM) M-level actuators in the form of piezo buzzers playing music. Most AI scripts here used the free Google search 'AI mode' AI. However this script used Claude AI to do 'phrase reduction' to reduce the 4 voices in Bach's original down to only 2 voices to enable it to be played on just the bass and treble actuator on an Arduino Uno or Nanao (the pin mappings for the buzzers are suited to those boards). If you want to use a Mega2560 you will need to change the pin mappings in the script accordingly (for example use pin 12 on the Mega instead of pin 9 and use pin 10 on the Mega instead of pin 3).

This requires the header files music.pch and stdpard.pch, both of which are included in this directory. It also requires you to have a 2 passive piezo buzzers - one connected with its positive lead to Arduino Nano or Uno digital pin 9 (the other lead to ground) and the other with its positive lead connected to digital pin 3 (the other lead to ground). ⚠️ **Don't connect cone speakers directly to Arduino pins or you could permanently damage the Arduino.**

## drawing1.pcs
This demonstrates the use of an integer array as a graphics array to draw shaped and text features on the VGA display area.

## drawing2.pcs
This demonstrates the use of an integer array as a graphics array to draw monochrome and colour glyphs in the packed bits format used by PCS (raw formats are also available but not demonstrated here). It shows how you can use hexadecimal representations of integers in PCS code.

## drawing3.pcs
This demonstrates the use of the programmable character generator (PCG) in PCS preview graphics. When you run it it will print some characters and wait for you to hit return - then it will re-program some of those characters to custom characters specified in binary and re-print the results.

## Fourier_preview.pcs
This required connection to a pardserver with at least one pardcapture server attached. It demonstrates how to setup a live preview loop in the VGA display canvas and how to use the preview overlay graphics to overlay a raw greyscale bitmap as a picture-in-picture. Not only that but it also shows how to do Fourier spectrum generation so you get a live Fourier preview. Press 'q' to quit.

## hello_world.pcs
This is the traditional minimalist 'Hello world' program in PCS.

## hello_world2.pcs
This is similar to hello_world.pcs but with a bit more formatting to show how general text is more usually dealt with.

## imgviewer.pcs
This shows how to use byte-precise file reading to enable the reading of p5 and p6 binary PNM images as well as p2 and p3 ASCII images  (illustrated at top right). It also uses advanced character management so that it can detect and skip comments in the PNM headers. This program will read the p2 and p3 example images provided here: preview.ppm and preview.pgm. In addition, it will read the binary PNM images provided here: puma_p5.pgm, preview_p5.pgm, apple_p6.ppm and preview_p6.ppm - some of which also contain comments in the headers. Other features illustrated in this program are: dynamic memory allocation, getting timed-out input from the user, controlled setting of font size and colour for user messages, gracefully handling 'file not found' errors and the use of the graphics mode for the renderer which ensures 1:1 aspect ratio display. This will also work on the command line with pcs-cli but it will not be able to automatically shrink the size of the printed 'pixels' so images emitted to terminal emulators may display quite large.

## interactive_motion_af.pcs
This is a script I use on the PUMA CNC stage to allow interactive motion of the stage and autofocus triggering. An example of a practical microscopy script.
⚠️ **Warning:** This script uses the header file 'cnc.pch' which contains Arduino pin-mappings and direction flags that have been tested and verified for my particular build of CNC stage and may not be valid for any other hardware. Do not run this script without verifying and /or editing those pin mappings and direction flags to suit your hardware or you could damage your hardware or your microscope specimen or both.

## jump_table.pcs
This demonstrates the how the non-local, stack-aware goto command can jump out of nested code using string a array variable as the label identifier.

## marauders.pcs
A fully playable video game that illustrates the general programming abilities of PCS (illustrated at top left). This includes interactive real time user input from the keyboard, animated UTF-8 colour graphics on the script console window, the use of PCS to make a pseudorandom number generator with its general maths commands (but note tht PCS also has a dedicated PCG random number generator within it so this is not necessary if you need pseudorandom numbers), the use of the beep command to add arcade-style sound effects and the ability to read a custom image as a front splash screen.
This is more of an AI - human collaboration than a fully AI generated program. I started the AI off by writing a simple 'turret commander program' that allows the user to move the turrent left and right and fire missiles. The AI then took hat code and added various features under my supervision to build the complete game. The AI wrote a dedicated readme guid to the game which you can find here labelled 'ReadMe_IPM.md'.
The front splash looks for an image on disc called splash.bin (provided here as well). If it finds it then it will render that image as the splash screen. Otherwise it will render a generic star field splash graphic.
Make sure the script console area is enlarged before running this game (likewise make sure your terminal emulator window is enlarged before starting the game if running this with pcs-cli) because otherwie the graphics will overshoot and wrap and not look correct).

## milliseconds.ps
A script that demonstrated the use of the Millis system variable and milliduration command for time keeping.

## MIM_Test.pcs
An example practical microscopy script for measuring minimum incremental motion (MIM) of the PUMA CNC stage.
⚠️ **Warning:** This script uses the header file 'cnc.pch' which contains Arduino pin-mappings and direction flags that have been tested and verified for my particular build of CNC stage and may not be valid for any other hardware. Do not run this script without verifying and /or editing those pin mappings and direction flags to suit your hardware or you could damage your hardware or your microscope specimen or both.

## pacmunch.pcs
I gave AI the full listing for 'interplanetary marauders' and asked it to program a 'pac-man' style game and this is what it came up with (illustrated top middle) - after some to-and-fro with me. It is a fully playable game with ghost logic, 'power pellets' that enable the character to turn tables on the ghost, arcade-style sound effects, etc. This was mostly written by Google's free 'AI mode' (before Gemini Hydrogen came out) and the sound effects were added later by Claude Sonnet. It's not bad for an AI effort - especially considering this is a 'new' language and all it had to go on was the example PCS script I gave it and the llms.txt file in this repository plus a few manual entries I posted on its interface for some commands. To play, use the [,] and [.] keys to move left and right and use the [a] and [z] keys to move up and down.

## pcs_throughput.pcs
A speed benchmark for some PCS operations written by Claude Opus to test the PCS compiler.

## robovox_demo.pcs
Demonstrates speech synthesis with an Arduino Nano - so you need to be logged into a pardserver with that Arduino connected. This uses the robovox PCS library and so also illustrates libraries in PCS (running the library source file robovox_lib.pcs will create the linkable library robovox_lib.pcl. This also needs the robovox header file robovox.pch). This was made by Claude Opus with feedback and training from me over 7 iterations. I still think there is room for improvement but it serves well enough as a demo. You will need an Arduino (any 8-bit AVR Arduino will do but the pin mappings in the library are for the Arduino Nano / Uno using digital pins 9 and 3. To get the most out of this you need to use a cone speaker rather than bare pasive piezos but then you also need to use some kind of amplifier. ⚠️ **Don't connect cone speakers directly to Arduino pins or you could permanently damage the Arduino.** I use a mixing amplifier that lets me connect upto three PWM pins to the same speaker but you could use a separate amplifier and speaker for each pin if that is what you have.

## sensors.pcs
Demonstrates how to use the Sensor actuators of PARDUS, including how to 'hot-swap' sensor pins and switch from analogue to digital inputs (and vice versa). This requires connection to an Arduino via logging into a pardserver.

## tracker.pcs
Demonstrates the use of trackers in PCS. Requires connection to a pardserver and a pardcapture server. When run press the 's' key to start/stop/reset the live preview. Press 'q' to quit. While the liver preview is active place your object to be tracked in the red square and hit a key to initialise a tracker onto it - the red square should then follow the target - unless the tracker fails.

## unicode_dashboard.pcs
Demonstrates how to build simple graphical feedback controls in the scripe console.

## Weiner_filter.pcs
Demonstrates the use of complex arithmetic, array operations and Fourier operations in PCS by implementing a simplified Weiner filter on a small image array hard coded into the script. The program convolves the image then de-convolved it via the simplified Weiner filter. All output is via text rather than graphical images for this small demo.

-----

# Other Files Here and in Headers and Libs

## cnc.pch
An example of a practical PCS header file specifying Arduino pin mappings and direction flags for my CNC stage. As specified in the **Warning** messages above, these may not be suitable for your hardware so do not run scripts with this header file without first verifying that the pin mappings and direction flags suit your hardware. This is required for my microscopy scripts here.  Many of the programs in this directory are 'pure logic' programs that do not interface with the Arduino or a camera and so they do not use this header.

## music.pch
An example of a practical PCS header file specifying frequencies for musical notes. It is for use with PWM M-level actuators.

## robovox.pch
The header file for the robovox speech synthesiser library described above.

## stdpard.pch
A header file containing common definitions that are used when addressing various types of actuators (and colour channels for imaging pardcap). Many of the programs here are 'pure logic' programs that do not interface with the Arduino or a camera and so they do not use this header.

## preview.pgm, preview_p5.pgm, puma_p5.pgm, preview.ppm, preview_p6.ppm, and apple_p6.ppm
These are test PNM images for use with imgviewer.pcs.

## ReadMe_IPM.md
A markdown manual for the interplanetary marauders game, written by an AI.

## splash.bin
A binary file containing the splash image for the interplanetary marauders game. For this to work it has to be placed in the directory where the pardclient interpreter program was run from (or you must edit the PCS script to include the path information of where this file is located).


PJT 27.11.2026
