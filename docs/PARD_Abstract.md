P43
Pardus: An Affordable Open Source Hardware and Software Robotic Platform for Standard Microscopes

PJ Tadrous
TadPath Diagnostics, London, UK

Purpose:
To develop an affordable open source platform to automate a routine microscope. There are many applications (e.g. live cell imaging, 3D deconvolution) for which specialist digital pathology and whole slide imaging (WSI) scanners are not optimal or applicable. Also, WSI scanners are prohibitively expensive for people without access to huge budgets or central resources. While there is much open source software, open source hardware has, until now, been largely dedicated to ‘re-imaginings’ of the microscope itself rather than automating routine diagnostic pathology scopes.

Methods:
Pardus™ (Programmable, Affordable, Remote-controlled Drive Utility Standard) is an open source hardware and software combination I developed to meet this need. Pardus allows a standard microscope to be motorised to 3 (or 4, i.e. objective turret) axes with inexpensive off-the-shelf components (Nema17 geared stepper motors, A4988 drivers controlled from a Raspberry Pi (RPi) computer) and some custom 3D printed parts. Use of the RPi makes the scope controllable via WiFi, Bluetooth (e.g.via smart phone) and by internet browser from any location. The system implements bi-directional communication to an (optional) separate image processing computer via the third party ZeroMQ library (also open source). The scope can be fully programmed in C (and other languages) as well as with a custom script language that requires no knowledge of formal computer programming.

Results:
Results from Pardus fitted to a 1975 Zeiss Standard microscope show a high degree of stage repositioning accuracy (upto <5 microns in X,Y). This system performed effective automated digital image capture, multi-frame averaging, background correction, autofocussing, tissue detection and slide scanning as well as a machine vision task and 3D deconvolution.

Conclusions:
Pardus allows the use of standard microscopes for slide scanning, machine vision and other automated / remote microscopy tasks as an affordable open source solution.

Link: https://www.pathsoc.org/_userfiles/pages/files/meetings/archive/WM2020AbsFile.pdf

Cite: Tadrous, P. J. (2020, March). Pardus: An Affordable Open Source Hardware and Software Robotic Platform for Standard Microscopes. In JOURNAL OF PATHOLOGY (Vol. 250, pp. 15-15). 111 RIVER ST, HOBOKEN 07030-5774, NJ USA: WILEY.          
