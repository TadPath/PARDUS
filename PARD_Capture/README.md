# PARD Capture
Scientific image acquisition software, FLOSS GPL v3


![PARD Capture Title](Images/PARD_Capture_Title.jpg)
---

----------------------- ------------------------------------
> :warning: **NOTE**: **The PARDUS project is NOT accepting pull requests at this stage of development. Any pull requests will be either blocked, ignored and/or deleted. I welcome bug reports, feature requests and issues / queries.**
----------------------------------------------------------------

                       
Introduction
------------
PARD Capture is a free (FLOSS) GUI (GTK3+) C program that uses the v4l2 API to acquire images from a video device such as a USB camera or any other device that will interface as /dev/video0. Currently it is only for Linux. There are many free image grabber programs out there so why use PARD Capture?

* Designed for scientific quantitative image capture
* Real-time preview with option for rolling multi-frame integration for night vision / low light preview
* Facility to do dark frame subtraction on each frame as it is being captured
* Facility to create a master dark image for the above correction.
* Facility to create a master flat image for flat field correction.
* Facility to apply flat field correction (as well as dark image subtraction) during image acquisition.
* Option to use only the Y component of a raw uncompressed YUYV image stream.
* Option to use all components of a YUYV or MJPEG video stream.
* Option to use the I component of an HSI transform during acquisition.
* Facility to do multi-frame averaging with or without dark frame and flat field corrections.
* Facility to use a custom mask for all image pre-processes during acquisition
* Facility to save images in raw doubles format so as not to loose the advantages gained by multi-frame averaging and the the various pre-process corrections described above.
* Facility to save images and results in standard FITS format with double precision floating point pixel data for transport to other software as required.
* Facility to save images in a range of other formats (see manual).
* Facility to keep a plain text log of all messages issued for a particular imaging session.
* Facility to control all camera settings available on any particular camera.
* Facility to save / load camera settings and custom settings for an imaging session to make it easy to repeat experiments and communicate protocols.
* Facility to take image series (each image may be a multi-frame average with all the above corrections applied as required).
* Facility to do time-lapse sequences.
* Facility for delayed start to image capture.
* For a complete list of options, see the user's manual

How to Install
--------------
PARD Capture is only available as C source code from this (the official) repository. You will need to compile it to run it. I have only tested it with gcc. There is no package manager install route for PARDUS capture at this time. The program itself comes as a single self-contained source file but to compile it into an executable program you will need to ensure first that you have installed all the libraries it depends on (i.e. its dependencies).
You will need the developer libs for v4l2, gtk3, libjpeg and libpng. The audio GUI signals require the PulseAudio libraries which may already come with your Linux as standard but, if not, you must install it. The below table shows the libraries you need to install with their names as used in Arch, Fedora, Debian and Ubuntu Linux distros:

| Library  | Arch          | Ubuntu/Debian    | Fedora                 |
| -------- | ----          | -------------    | ------                 |
| v4l2     | v4l-utils     | libv4l-dev       | libv4l-devel           |
| JPEG     | libjpeg-turbo | libjpeg-dev      | libjpeg-turbo-devel    |
| GTK      | gtk3          | libgtk-3-dev     | gtk3-devel             |
| PNG      | libpng        | libpng-dev       | libpng-devel           |
| Pulse    | libpulse      | libpulse-dev     | pulseaudio-libs-devel  |

Table: Dependencies for PARD Capture. Install these using your Linux installation method before compiling the PARD Capture software.

Note: Arch Linux installs the developer parts of a lib as standard with each lib. Other Linux distros may need to specifically install the ‘dev’ or ‘devel’ versions of each lib.

Once all the dependencies are installed compile the PARD Capture code itself (called pardcap.c) with gcc using the following command:

gcc `pkg-config --cflags gtk+-3.0` -Wall -o pardcap pardcap.c `pkg-config --libs gtk+-3.0` -lm -lpng -ljpeg -lpulse-simple -lpulse

Once compiled, make the program executable using the chmod command:

chmod +x ./pardcap

You may then optionally put pardcap in a place that is on your PATH environment variable so you don’t need a local copy of it wherever you intend to run it.

How to Run PARD Capture
-----------------------
The following assumes you have PARD Capture on your system ready to run. At the time of writing this manual only a Linux version of PARD Capture is available.


Background
----------
This program is the first stage of publication of the open source PARDUS system.
PARD Capture is the image frame grabber component of the PARD Server. In addition to PARD Capture the PARD server has a network server loop (so all the functions can be controlled remotely over a network) a hardware interface so multiple motors, actuators and limit switched can be controlled in addition to image capture / optical feedback) and a command script interpreter (so complex image acquisition and robotic activities can be run as an algorithm that may adapt to input and image process results). For more detail see the home page of this repository.
I made the first working PARDUS system back in 2019 but the Pandemic got in the way of publication and I have been working on my [PUMA microscope](https://github.com/TadPath/PUMA) project instead.
Now I have released the first experimental camera system for the PUMA microscope - the OptArc AF51 - and I needed a frame grabber that would do justice to both the camera and the microscope. Realising I had already done most of the programming for that in the PARDU project I took the opportunity to bring the code up-to-date and begin the release of PARDUS by making this stand alone PARD Capture program derived from the original PARD Server.

Full User Manual
----------------
A complete user manual together with illustrations and an index is available free of charge to download from the [support pages of the OptArc website](https://www.optarc.co.uk/support/) which sells the AF51 camera. The user manual is actually the manual for the camera but chapter 6 therein constitutes a full user manual for the PARD Capture software and the manual index has a dedicated section to the software. Additional video tutorials may become available from time-to-time on the [PUMA Microscope YouTube channel](https://youtube.com/@PUMAMicroscope) so keep an eye on that.
DOI: All sales from the [OptArc.co.uk](https://www.optarc.co.uk/) store contribute to supporting my projects.

Funding
-------
This is a self-funded project. I do all the work in my own time.





PJT

First Written: 12.11.2022

Last Edit: 12.11.2022
