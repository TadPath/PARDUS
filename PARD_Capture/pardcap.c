/**********************************************************************/
/* TadPath PARD Capture (PardCap) Stand Alone v1.0                    */
/*                                                                    */
/* PardCap is a stand alone version of the image capture functions of */
/* the PARDUS robotics control system. 'Stand Alone' distinguishes    */
/* this version from the full 'PARD Server' (see below 'Intoroduction */
/* to PARDUS'). This 'Stand Alone' verison contains only the basic    */
/* image capture components of the PARD Server. It lacks the network  */
/* server the communications protocol for the PARD Daemon and the     */
/* automtion script interpreter.                                      */
/*                                                                    */
/* Copyright (c) Dr Paul J. Tadrous 2000-2022   All rights reserved.  */
/*                                                                    */
/* This program is distributed under the terms of the GNU General     */
/* Public License (GPL v3 or later).                                  */
/*                                                                    */
/* This program is free software: you can redistribute it and/or      */
/* modify it under the terms of the GNU General Public License as     */
/* published by the Free Software Foundation, either version 3 of the */
/* License, or (at your option) any later version.                    */
/*                                                                    */
/* This program is distributed in the hope that it will be useful,    */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of     */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU   */
/* General Public License for more details.                           */
/*                                                                    */
/* You should have received a copy of the GNU General Public License  */
/* along with this program. If not, see                               */
/* <https://www.gnu.org/licenses/>.                                   */
/*                                                                    */
/* The linux version of this software uses v4l2 functions to control  */
/* the video device in /dev/video0                                    */
/* This version has been optimised for use with cameras sold via the  */
/* OptArc.co.uk online store that supports the PUMA open source       */
/* microscopy project so it may not be fully compatible other video   */
/* devices but many of the features should still function with other  */
/* cameras - trial and error will bring the specifics to light.       */
/*                                                                    */
/* INTRODUCTION TO PARDUS                                             */
/* ======================                                             */
/*                                                                    */
/* Programmable Affordable Remote-controlled Drive Utility Standard   */
/* (PARDUS) is an open source hardware and software standard for      */
/* driving multiple stepper motors +/- limit switches with optical    */
/* feedback (including via a camera) and with remote (over network)   */
/* control and script automation. Originally developed to use the     */
/* Raspberry Pi as hardware controller and server, the modern version */
/* seeks to use a standard Linux computer as server for greater       */
/* scaleability. The PARDUS system is composed of three components:   */
/*                                                                    */
/* 1. The PARD Server                                                 */
/* 2. The PARD Daemon                                                 */
/* 3. The PARD Client                                                 */
/*                                                                    */
/* The PARDUS system was first developed in 2019 and published in     */
/* abstract form at the Winter Meeting of the Pathological Society of */
/* Great Britain and Ireland, ref: Pardus: An Affordable Open Source  */
/* Hardware and Software Robotic Platform for Standard Microscopes    */
/* Tadrous, P. J. Journal of Pathology 250: 15. ISSN: 0022-3417;      */
/* 1096-9896 2020                                                     */
/*                                                                    */
/* The practical application shown in that publication was remote     */
/* control of a Zeiss Standard microscope but the PARDUS system is    */
/* for remote and automated operation of motorised robotics systems   */
/* in general and not limited to microscopes.                         */
/*                                                                    */
/* See the project GitHub page for more details:                      */
/* https://github.com/TadPath/PARDUS                                  */
/*                                                                    */
/* P. J. Tadrous 31.01.2020 (last edit 23.11.22)                      */
/**********************************************************************/

// Linux:
//
// You will need to have certain developer versions of some libraries
// installed to compile this. See below for what to install. 
//
// Compile with:
//
// gcc `pkg-config --cflags gtk+-3.0` -Wall -o pardcap pardcap.c `pkg-config --libs gtk+-3.0` -lm -lpng -ljpeg -lpulse-simple -lpulse

// Required Libs:
//
// You will need the developer libs for v4l2, gtk3, libjpeg and libpng.
// The audio GUI signals require the PulseAudio libraries which may 
// already come with your linux as standard but, if not, see below.
// Note: Arch linux gets the developer parts of a lib as standard with
// the lib. Other types of linux need to specifically install the dev or
// devel versions of the lib.
// Install the following packages according to the installation method
// used in your linux distro. Use the packages and versions that are
// appropriate to your architecture. The below examples are for Arch,
// Fedora, Debian and Ubuntu:
//
//  Distro:     Arch            Ubuntu/Debian          Fedora
//  ====================================================================
//           v4l-utils          libv4l-dev         libv4l-devel
//           libjpeg-turbo      libjpeg-dev        libjpeg-turbo-devel
//           gtk3               libgtk-3-dev       gtk3-devel
//           libpng             libpng-dev         libpng-devel
//           libpulse           libpulse-dev       pulseaudio-libs-devel
//


// Version number
#define PARDCAP_VERN "1.0.23.11.22"

// This is needed for general programming
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>

// This is needed to test for endianness for writing output in the FITS
// format. With some systems might need to #include <netinet/in.h>
// instead of <arpa/inet.h>
#include <arpa/inet.h>

////////////////////////////////////////////////////////////////////////
// This is needed for audible feedback from the GUI
#include <pulse/simple.h>
#include <pulse/error.h>
 
#define PABUFSIZE  400
#define DPABUFSIZE 400.0
pa_simple *PA_s = NULL;
uint8_t sine_buffer[PABUFSIZE];
int Audio_sounding;  // A flag to avoid overlapping threads
int Audio_status;    // Used for GUI audio signal feedback (useful if
                     // working in dark conditions without a monitor)
// Values for Audio_status:
#define AS_NULL   0  // Audio output is not possible
#define AS_INIT   1  // Audio API initialised

int Use_audio;
// Values for Use_audio:
#define AU_NO   0  
#define AU_YES  1 
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
// Need to decompress MJPEG stream images for preview and to save
// images to disc in JPEG format other than those stills taken directly
// from the MJPEG stream.
#include <jpeglib.h>
#include <jerror.h>
#include <setjmp.h> // To trap fatal errors from jpeglib

// The following is an extension of the default jpeg error handler used
// to allow us to intercept a fatal 'exit(1)' error from libjpeg. This
// code is copied from the libjpeg-turbo GitHub repository here:
// https://github.com/leapmotion/libjpeg-turbo/blob/master/example.c
struct my_error_mgr {
  struct jpeg_error_mgr pub;    // "public" fields 

  jmp_buf setjmp_buffer;    // for return to caller
};
typedef struct my_error_mgr * my_error_ptr;
// Here's the routine that will replace the standard error_exit method:
METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
  // cinfo->err really points to a my_error_mgr struct,
  // so coerce pointer
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  // Always display the message. 
  // We could postpone this until after returning, if we chose.
  (*cinfo->err->output_message) (cinfo);

  // Return control to the setjmp point 
  longjmp(myerr->setjmp_buffer, 1);
}
////////////////////////////////////////////////////////////////////////


// This is needed for saving images in png format
#include <png.h>

// This is needed for the GUI
#include <gtk/gtk.h>
#ifndef G_SOURCE_FUNC
#define G_SOURCE_FUNC(f) ((GSourceFunc) (void (*)(void)) (f))
#endif

// These are needed for v4l2 camera functionality
#include <assert.h>
#include <fcntl.h>              
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

// Camera defs
typedef struct {
    char cs_opened;
    char cs_initialised;
    char cs_streaming;
 } CamStatus;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4')
#endif

// Here is how the camera (video device) status behaves.
//
// OPENED
//   INITIALISED (buffers allocated according to frame size, etc.)
//      STREAMING_ON
//         Only at this stage can you attempt to grab images
//         from the device (e.g. to save or display as a live preview).
//      STREAMING_OFF
//   UNINITIALISED (buffers freed)
// CLOSED
//
// So, in order to be initialised, the camera must first be opened, etc.
// Checks in the program must follow this scheme for everything to work
// propery. So, for example, the image grabbing function must not
// attempt to progress if camera is not streaming.
// It is advisable to leave the stream on for at least a few seconds and
// grab and discard some frames (i.e. skip frames) before grabbing an
// image to ensure stabilisation of the image and any auto functions
// like auto exposure and auto white balance.

#define CS_OPENED           1  
#define CS_INITIALISED      2  
#define CS_STREAMING        3  

// Values that the image grabber function returns
#define GRAB_ERR_NONE       1 // Success!
#define GRAB_ERR_SELECT     2 // Error selecting a frame from the stream
#define GRAB_ERR_TIMEOUT    3 // Too long geting a frame from the stream
#define GRAB_ERR_READIO     4 // Error getting a frame by read io method
#define GRAB_ERR_MMAPD      5 // MMAP DQBUFF Error
#define GRAB_ERR_MMAPQ      6 // MMAP QBUFF Error
#define GRAB_ERR_USERPD     7 // User pointer DQBUFF Error
#define GRAB_ERR_USERPQ     8 // User pointer QBUFF Error
#define GRAB_ERR_NOSTREAM   9 // The camera is not streaming images
#define GRAB_ERR_BUSY      10 // The grabber is being used by another
                              // program or function - try again later.

int grab_n_save(void); // Takes a picture with intent to save it (as
                       // opposed to grabbing for the live preview only)
// Return values for grab_n_save() function:
#define GNS_OKIS    0 // All went well. A picture was saved.
#define GNS_OKNB    1 // All went well. A picture was saved but was NOT
                      // flat field corrected even though the user asked
                      // for flat field correction.
#define GNS_OKNS    2 // All went well. A picture was NOT saved because
                      // tissue content thresholding was requested and
                      // the tissue content was below the trheshold.
#define GNS_ECAM    3 // Failed due to problem running the camera.
#define GNS_EGRB    4 // Failed due to an error returned by the grab
                      // routines (the value of the global 'grab_report'
                      // variable will contain the error details as one
                      // of the GRAB_ERR_... #defines, which see).

#define MAX_CAM_SETTINGS    256 // The maximum number of camera setting
                                // widgets that can be displayed. This
                                // includes labels, tick boxes, etc. so
                                // is more than the number of settings.

// end of camera defs

// Values for returnvalue of csetfile_check()
#define PCHK_ALL_GOOD   0 // No errors detected
#define PCHK_TERMINUS   1 // Program not terminated correctly
#define PCHK_E_SYNTAX   2 // Syntax error detected
#define PCHK_E_COMMND   3 // Unrecognised command
#define PCHK_E_FORMAT   4 // Incorrect file format

// Values for returnvalue of csetfile_run()
#define PRUN_ALL_GOOD   0 // No errors detected
#define PRUN_ERROR      1 // An error occurred


// Structure for BMP format header information
// Use specific length integers to ensure cross platform compatibility.
typedef struct {
    int16_t  Type;
    uint32_t FSize;
    int16_t  Res1;
    int16_t  Res2;
    uint32_t Offs;
    uint32_t IHdSize;
    uint32_t Width;
    uint32_t Height;
    int16_t  Planes;
    int16_t  Bitcount;
    uint32_t Compresn;
    uint32_t ImgSize;
    uint32_t Xpixelsm;
    uint32_t Ypixelsm;
    uint32_t ClrsUsed;
    uint32_t ClImport;
} BMPHead;

// For BMP IO
#define BM8 1 // 8bpp greyscale BMP file
#define BMP 2 // 24bpp colour BMP file
#define WORDSZ    4



// Dark field subtraction
int dffile_loaded;          // Lets functions know if a dark field
                            // correction reference file is loaded and
                            // what type it is. 
int DFht,DFwd;              // The dimensions of the currently loaded
                            // dark field image - to check if they are
                            // identical to the current main image.
double *DF_Image;           // To hold the dark field reference image. 
int dfcorr_status;          // Lets the program know whether the user
                            // has ticked the box for DF correction to
                            // be applied. Other conditions need to be
                            // met before this choice can be actioned.
int df_pending=0;           // Lets settings control know if the user
                            // selected a dark field image so it can
                            // attempt to load/process it upon 'Apply'
int do_df_correction;       // Whether to actually dark field correct an
                            // image or not in any given instance of
                            // image capture.
                            
// Values for do_df_correction
#define DODF_NO     0  // Don't do it (usually because the currently
                       // selected 'save as' format is incompatible with
                       // the colour format of the currently loaded
                       // dark field image.
#define DODF_RGB    1  // Yes, correct a colour image with the currently
                       // loaded colour dark field image.
#define DODF_Y      2  // Yes, correct a greyscale image with the
                       // currently loaded greyscale dark field image.
// Values for dfcorr_status are
#define DFCORR_OFF  0 // Don't (or can't) do dark field correction
#define DFCORR_ON   1 // Do dark field correction 

// Values for dffile_loaded (also used for dffile_loaded so flat field
// when reading raw doubles from disc so these MUST have the same
// #defined numerical values as the corresponding FFIMG_... constants). 
#define DFIMG_NONE  0  // No dark field (df) image is currently loaded
#define DFIMG_RGB   1  // A colour image is loaded for df correction
#define DFIMG_Y     2  // A monochrome image is loaded for df correction


// Flat field correction
int fffile_loaded;          // Lets functions know if a flat field
                            // correction reference file is loaded and
                            // what type it is. 
int FFht,FFwd;              // The dimensions of the currently loaded
                            // flat field image - to check if they are
                            // identical to the current main image. 
double *FF_Image;           // The raw doubles version of flat field.
int ffcorr_status;          // Lets the program know whether the user
                            // has ticked the box for FF correction to
                            // be applied. Other conditions need to be
                            // met before this choice can be actioned.
int ff_pending=0;           // Lets settings control know if the user
                            // selected a flat field image so it can
                            // attempt to load/process it upon 'Apply'
int do_ff_correction;       // Whether to actually flat field correct an
                            // image or not in any given instance of
                            // image capture.
                            
// Values for do_ff_correction
#define DOFF_NO     0  // Don't do it (usually because the currently
                       // selected 'save as' format is incompatible with
                       // the colour format of the currently loaded
                       // flat field image.
#define DOFF_RGB    1  // Yes, correct a colour image with the currently
                       // loaded colour flat field image.
#define DOFF_Y      2  // Yes, correct a greyscale image with the
                       // currently loaded greyscale flat field image.
// Values for ffcorr_status are
#define FFCORR_OFF  0 // Don't (or can't) do flat field correction 
#define FFCORR_ON   1 // Do flat field correction.

// Values for fffile_loaded 
#define FFIMG_NONE  0  // No flat field (ff) image is currently loaded
#define FFIMG_RGB   1  // A colour image is loaded for ff correction
#define FFIMG_Y     2  // A monochrome image is loaded for ff correction
#define FFIMG_NORM  3  // It loaded fine but is awaiting normalisation.


// Corrections mask
int mskfile_loaded;         // Lets functions know if a corrections mask
                            // file is loaded. 
int MKht,MKwd;              // The dimensions of the currently loaded
                            // corrections mask - to check if they are
                            // identical to the current main image.
unsigned char *MaskIm;      // To hold the corrections mask image. 
int msk_pending=0;          // Lets settings control know if the user
                            // selected a corrections mask image so it 
                            // can attempt to load it upon 'Apply'
int mask_status;            // Let the program know whether to use
                            // a user-supplied correction mask when
                            // doing dark or flat field correction.
int mask_alloced;           // Whether a useable corrections mask is
                            // present. Even if a user-supplied mask is
                            // not present an internal all-premissive
                            // mask is generated and used. If even this
                            // internally generated mask cannot be made
                            // (due to failure of memory allocation)
                            // then no dark field of flat field
                            // correction can be applied even if those
                            // other correction images are loaded and
                            // good to go.
double Mask_supp_size;      // The number of pixels >0 (used for
                            // calculating mean pixel values per frame
                            
// Values for mask_alloced
#define MASK_NO     0  // Don't use a custom corrections mask.
#define MASK_YES    1  // Yes, use a custom corrections mask.

// Values for mskfile_loaded 
#define MASK_NONE  0  // This is set when mask_alloced fails.
#define MASK_YRGB  1  // A custom corrections mask is currently loaded
#define MASK_FULL  2  // A non-custom full support mask is present


// Camera Settings
int csetfile_loaded;        // Lets functions know if a settings file is
                            // loaded. 
// Values for csetfile_loaded 
#define CSET_NONE  0  // No custom settings file is currently loaded
#define CSET_CUST  1  // A custom settings file is currently loaded


// Values for colour channel for raw doubles IO
#define CCHAN_Y 1
#define CCHAN_R 2
#define CCHAN_G 3
#define CCHAN_B 4

// Camera and v4l2-related

CamStatus camera_status; // Lets any function know what state the
                         // camera is in at any given time so
                         // appropriate actions can be taken
int camera_remote=0;     // Flag set to 1 if user selects remote control
                         // camera operation from camera settings
int grab_report;         // The global version of 'grabber_says' allows
                         // functions that call the main camera button
                         // save routine to know the outcome.
struct v4l2_queryctrl vd_queryctrl; // To list capture device controls
struct v4l2_querymenu vd_querymenu; // To list capture device control
                                    // menu items.
struct v4l2_control vd_control;     // To get (VIDIOC_G_CTRL) or set
                                    // (VIDIOC_S_CTRL) a capture device
                                    // control value.
enum io_method {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};
struct buffer {
        void   *start;
        size_t  length;
};
static char            *dev_name;
static enum io_method   io = IO_METHOD_USERPTR;
static int              fd = -1;
struct buffer          *buffers = NULL;
static unsigned int     n_buffers;
static int              frame_number = 0;
// Frames to skip between each capture (to clear the buffers and refresh
// the image):
static int skipframe,skiplim=0; 
static int frame_timeout_sec;  // Time to wait before giving
static int frame_timeout_usec; // up on getting a frame.

// General
#define FOREVER for(;;)
#define MAX_CMDLEN 512  // Maximum length for a command string.
int ImHeight,ImWidth,ImSize,ImWidth_stride,Need_to_save;
int CurrDims_idx,VGA_idx;
int Delayed_start_on,Delayed_start_in_progress;
double Delayed_start_seconds;
char *ImRoot,*FFFile,*DFFile,*CSFile,*MaskFile;
char Selected_FF_filename[FILENAME_MAX];  // Flat field
char Selected_DF_filename[FILENAME_MAX];  // Dark field
char Selected_CS_filename[FILENAME_MAX];  // Camera settings
char Selected_Mask_filename[FILENAME_MAX];// Mask file
// YUYV to RGB conversion LUTs:
double *lut_yR,*lut_yG,*lut_yB,*lut_crR,*lut_crG,*lut_cbG,*lut_cbB;
// Gain and Bias factors for YUYV to RGB conversion - if these values
// change, the LUTs must be re-calculated: 
double Gain_conv,Bias_conv; 
// Whether the YUYV to RGB conversion luts have been calloced or not:
int luts_alloced = 0; 
// To store the full-size colour converted camera image (also used for
// Y-only image):
unsigned char *RGBimg; 
int           RGBsize;  // The size, in bytes, of the RGBimg RAM block.
int           col_conv_type;// Flag that determines what level of
                            // conversion is required - see colour
                            // conversion #defs YUYV_TO_... MJPEG_TO_...
int JPG_Quality=100; // JPEG save as quality (only applies to images 
                     // from a non-MJPEG stream (i.e. YUYV format) or to
                     // multi-frame averages from any camera stream.


////////////////////////////////////////////////////////////////////////
//////////                                                    //////////
///////               PREVIEW-RELATED GLOBALS                  /////////
//                                                                    //

// I anticipate that no camera will have more than 20 different size
// resolutions. If one does then this will need to be expanded:
#define MAX_RESOLUTIONS 20 
// Actual number of resolutions in drop-down list
int Nresolutions;

// Maximum frame rate for preview for a given resolution: 
unsigned int maxframerate[MAX_RESOLUTIONS];  
// Flag to let the grab_image function know if it is being called
// already - to prevent multiple siumultaneous usage (e.g. by preview
// timeout function and main GUI functions)
int image_being_grabbed = 0; 
int from_preview_timeout = 0; // This flag lets grabber know if it is
                              // being called by the live preview
                              // timeout function.

// This flag (set to 1) allows the preview function to perform a task
// only once when previewing is activated by the user (the preview
// function gets called for each frame capture of a live stream):                            
int preview_only_once = 1; 

int preview_fps;// Frames per second for the preview update. This is
                // not the same as the camera's inherent fps capability.
                
int change_preview_fps = 0; // flag which lets the timeout know when to
                            // update its interval (and so alter the 
                            // preview FPS).
                            
int preview_stored; // Flag that lets us know if a preview image was
                    // generated for any given fame capture so we don't
                    // try to generate it multiple times and so waste
                    // processing time.

int Preview_changed; // Lets us know we need to update preview
                     // parameters after an 'Apply' settings click.

#define PREVINTMAX 20 // Maximum No. of frames for integration.
                    
unsigned char *PreviewImg,*PreviewRow;
int           *PreviewBuff[PREVINTMAX];
int            PreviewImg_size,PreviewImg_rgb_size;
int            PreviewHt,PreviewWd,PreviewWd_stride;
int            Preview_impossible,Need_to_preview;
int            Preview_integral,Preview_bias,PreviewIDX;
double        *Preview_dark; // Master dark for live preview
double        *Preview_flat; // Master flat for live preview
int            PrevCorr_BtnStatus = 0; // Preview correction btn state
int            PrevDark_Loaded = 0; // Whether a preview dark is loaded
int            PrevFlat_Loaded = 0; // Whether a preview flat is loaded
// PrevCorr_BtnStatus values (determines the action of the preview dark
// button)
#define PD_LOADD 0 // Button will load a master dark
#define PD_LOADF 1 // Button will load a master flat
#define PD_EJECT 2 // Button will eject a loaded master dark +/ flat
void nullify_preview_darkfield(void);
void nullify_preview_flatfield(void);

// Values for preview frame rate
const char *fps_options[] = {"1","2","3","4","5","7","10","15","25","30"};
int  fpx_max=10;

// Values for preview_stored
#define PREVIEW_STORED_NONE 0
#define PREVIEW_STORED_MONO 1
#define PREVIEW_STORED_RGB  2

// Values for Need_to_preview
#define PREVIEW_OFF  0 // No preview required
#define PREVIEW_ON   1 // Preview image required

// Variables for subsampling down monochrome images to preview size:
int Prev_startrow,Prev_startcol; // For centering the preview image
int Prev_startrow1; // To calculate tile offset
int Img_startrow,Img_startcol; // For selecting tile of full scale image
int *SSrow,*SScol; // Subsampling index arrays
double Prev_scaledim; // Calculate preview sampling and tile centering.

// These enable the preview of a tile crop window from the full-sized
// captured image (as opposed to a scaled down subsample):             
int Preview_fullsize,selected_Preview_fullsize;
int Preview_tile_selection_made; // Has the user slected the tile yet?
int Prevclick_X,Prevclick_Y; // The co-ords selected by the user.

// Preview frame integration and bias constants             
#define PADJUST_INTEGRAL 1
#define PADJUST_BIAS     2

// Some function defs and GUI widget-related globals for the preview are
// given below.

//                                                                    //
///////       (end of)  PREVIEW-RELATED GLOBALS                /////////
//////////                                                    //////////
////////////////////////////////////////////////////////////////////////




// Widget indices - used to keep track of which  CamsetWidget[] we are
// dealing with when working with the camera settings window interface 
int windex,rowdex,windex_gn,windex_bs,windex_sz,windex_fps;
int windex_camfmt,windex_safmt; // Camera stream format and save-as fmt.
int windex_uf,windex_uf2;  // Use FF label and check box
int windex_ud,windex_ud2;  // Use DF label and check box
int windex_um,windex_um2;  // Use Mask label and check box
int windex_imroot,windex_fno,windex_pc,windex_avd,windex_yo;
int windex_rffi,windex_rdfi; // Flat field and dark field labels
int windex_ldcs,windex_sacs; // Load/Save camera settings labels
int windex_rmski;            // Corrections mask label
int windex_to,windex_rt;   // Frame grabber timeout and no. of retries. 
int windex_srn,windex_srd; // Image series controls.
int windex_sad;            // Save as raw doubles 
int windex_fit;            // Save as FITS (with pixels as doubles) 
int windex_smf;            // Scale mean of each frame to first
int windex_del;            // Delayed start to capture
int windex_jpg;            // JPEG save as quality for averaged images
                           // (does not apply to single frames directly
                           // saved from the camera's MJPEG stream).


int Save_raw_doubles=0;    // Save the image in raw doubles format (in
                           // addition to the primary format).
int Save_as_FITS=0;        // Save the image in FITS format (in
                           // addition to the primary format). This
                           // saves a double precision floating point
                           // FITS file so saves the pixel data to the
                           // same precision as raw doubles. The FITS
                           // header contains more information about
                           // the image capture and processing settings
                           // than the external header saved with the
                           // raw doubles option.

int Selected_Ht,Selected_Wd; // The image dimensions selected from the
                             // combo list, prior to them being applied.
                             
// These enable GUI edit boxes with the camera setting they represent.
unsigned int ctrl_id[MAX_CAM_SETTINGS],cswt_id[MAX_CAM_SETTINGS]; 
// Values for cswt_id
#define CS_WTYPE_UNDEF 0 // Camera setting widget type not yet defined.
#define CS_WTYPE_ENTRY 1 // Camera setting widget type is GtkEntry
#define CS_WTYPE_LABEL 2 // Camera setting widget type is GtkLabel
#define CS_WTYPE_COMBO 3 // Camera setting widget type is GtkCombo

// Frame stores to hold the captured and decoded image prior to any
// further processing such as dark field and flat field corrections.
double *Frmr,*Frmg,*Frmb;
int Frame_status=0; // Let us know if frame stores are alloced.
#define FRM_ALLOCED 1
#define FRM_FREED   0

// Accumulators to hold multiframe average images.
double *Avr,*Avg,*Avb; 

// The mean to shift each frame to before accumulating them into the
// multi-frame average accumulators
double Av_meanr,Av_meang,Av_meanb; 

// The multiframe average denominator (= No. of frames to average) and
// index (= the current frame we are processing in a multiframe average)
int Av_denom = 1,Av_denom_idx,Av_limit=0,Av_scalemean=0; 
int From_av_cancel=0; // To enable graceful interruption of averging.
int Accumulator_status=0; // Let us know if accumulators are alloced.
#define ACC_ALLOCED 1
#define ACC_FREED   0

// Number of seconds to wait to a frame from the frame grabber while
// capturing (not preview) and the number of times to retry
int Gb_Timeout=4, Gb_Retry=100;

// Settings for image series (including time-lapse).
// Number of images to capture in a series and time delay between each
// capture. For the purposes of a series, a single 'image to capture'
// may be a multi-frame average with dark field and flat field
// correction if those options are requrested and properly set up.
// A delay value is actually only a 'minimum' delay because if it takes
// longer to capture an image than Ser_Delay then that longer time will
// be the actual delay for the next image. Actual time intervals between
// the start of one image capture and the start of the next will be
// written to a text file for reference (via FPseries). FPseries will be
// open for writing when Ser_active is 1 and closed  when it is 0.
// The text file will called Series_<root>.txt where <root> is the root
// file name for image capture supplied by the user (default = 'frame'). 
int Ser_Number=1, Ser_Delay=0, Ser_idx=0, Ser_active=0,Ser_cancel=0;
int Ser_lastidx; // Used to calculate remaining captures after cancel
FILE *FPseries;
time_t Ser_ts,Ser_tc; // Timing variables for series and start delay.
char Ser_name[FILENAME_MAX],Ser_logname[FILENAME_MAX+64];

// Function to re-size memory storage for pointers
int resize_memblk(void **,size_t, size_t,char *);

// The format to use when saving a captured full frame image to disk:
int saveas_fmt; 
// Values for saveas_fmt
const char *safmt_options[] = {"Raw YUYV","Y PGM","Y BMP","Intensity","PPM","BMP 24bpp","PNG","JPEG"};
int Nsafs=8;
#define SAF_YUYV 0 // Raw YUYV 16 bpp array
#define SAF_YP5  1 // Y-only (greyscale) 8bpp pgm p5 format
#define SAF_BM8  2 // Y-only (greyscale) 8bpp bmp format
#define SAF_INT  3 // Insensity from RGB/YUYV raw doubles monochrome
#define SAF_RGB  4 // Full colour RGB array saved in PPM p6 format
#define SAF_BMP  5 // Full colour RGB array saved in 24bpp BMP format
#define SAF_PNG  6 // Full colour PNG file
#define SAF_JPG  7 // Full colour JPEG file

// The format used by the data stream from the camera:
unsigned int CamFormat;
int FormatForbidden;

// Values for camfmt_options
const char *camfmt_options[] = {"Raw YUYV","MJPEG"};
int Ncamfs=2;
#define CAF_YUYV   0 // Raw YUYV 16 bpp array
#define CAF_MJPEG  1 // Y-only (greyscale) 8bpp pgm p5 format
#define CAF_ALLOK  3 // For the FormatForbidden flag
#define CAF_ALLBAD 4 // For the FormatForbidden flag


// Values for col_conv_type
#define CCOL_TO_Y    1 // Extract the Y component  
#define CCOL_TO_RGB  2 // Convert to full RGB
#define CCOL_TO_BGR  3 // Convert to full BGR for saving as BMP file


// Other GUI-related

// This flag lets functions know if the GUI is up or destroyed (at shut
// down) so they don't inadvertently try to modify widgets after the GUI
// goes down:
int gui_up;

// Gtk controls
GtkWidget *Win_main;
GtkWidget *dlg_choice,*dlg_info;
GtkWidget *chk_preview_central,*chk_cam_yonly,*chk_useffcor;
GtkWidget *chk_scale_means;
GtkWidget *chk_sa_rawdoubles,*chk_sa_fits;
GtkWidget *chk_usedfcor,*chk_usemskcor;
GtkWidget *lab_cam_status,*btn_cam_stream,*chk_cam_preview;
GtkWidget *chk_audio;
GtkWidget *Img_preview,*Ebox_preview,*Ebox_lab_preview;
GtkWidget *win_cam_settings,*grid_camset,*btn_cs_apply;
GtkWidget *btn_cs_load_ffri,*btn_cs_load_dfri,*btn_cs_load_mskri;
GtkWidget *btn_cs_load_cset,*btn_cs_save_cset;
GtkWidget *btn_av_interrupt; // To cancel an averaging sequence.
GtkWidget *CamsetWidget[MAX_CAM_SETTINGS];
GtkWidget *combo_sz,*combo_fps,*combo_safmt,*combo_camfmt;
GtkWidget *btn_cam_save, *btn_cam_settings;
GtkWidget *load_file_dialog,*save_file_dialog;
GtkWidget *About_dialog;
GtkWidget *preview_integration_sbutton;
GtkWidget *preview_bias_sbutton;
GtkWidget *preview_corr_button;
GtkWidget *prev_int_label,*prev_bias_label;


// These are for the interactive slider control for real-time setting of
// a camera control value:
GtkWidget *ISlider,*ISLabel;
GtkAdjustment *ISlider_params;
int IScompatible; // To know if a control can be used with the slider.
int IScidx,ISwindex;


//  Allows text labels to be overlayed onto the preview image:
GtkWidget *Overlay_preview; 
//  Text label to be overlayed onto the preview image:
GtkWidget *Label_preview;
// Default width of buttons in the GUI - may be varied during GUI set up
// by the coder as required:  
gint bt_def_wd; 


#define UPDATE_GUI if(gui_up)while(gtk_events_pending())gtk_main_iteration();

GtkEntryBuffer *camgeb[MAX_CAM_SETTINGS];
GdkPixbuf      *gdkpb_preview;

// For the program's icon
 int PardIcon_ready = 0;
 GdkPixbuf *PardIcon_pixbuf = NULL;
 unsigned char *PardIcon_data =NULL;
// End of icon-related variables.

int fmtchoice(char *);
int fps_index(int);
int camfmt_from_string(char *);
int saveas_from_string(char *);
int cs_int_range_check(int, int,const char *, int, int);

int change_image_dimensions(void);
// Return values for change_image_dimensions
#define CID_OK          0 // Success
#define CID_NOCLOSE     1 // Failed - couldn't close the imaging device
#define CID_NOREVERT    2 // Failed - and couldn't revert to previous
                          // either - big trouble.
#define CID_REVERTED    3 // Failed - but managed to safely revert to
                          // previous dimensions.
#define CID_NOSTREAM    4 // Success in changing dimensions but failed
                          // to re-start the camera stream
#define CID_NOPREVIEW   5 // Success in changing dimensions but failed
                          // to set the preview choice


int raw_to_pgm(char *fname, int ht, int wd,void *data);
int raw_to_ppm(char *fname, int ht, int wd,void *data);

// I channel all messages to user via this one function. It allows me to
// control whether a GUI popup box should be used and/or whether the
// message should go to the command line (stdout) and/or to stderr
// and/or to the log file and/or be sent over the network - depending on
// whether there is a GUI operational and other global switches.
// When modifying the source code you should use this function instead
// of printf / fprintf or showing a dialog directly.
void show_message(char *,char *,int,int);
// Message types used with function show_message
#define MT_INFO 1 // Equivalent of printf - usually for info only
#define MT_ERR  2 // Equivalent of fprintf(sderr, - to indicate an error
#define MT_QUIT 3 // Message given upon program quit request.
char LogFilename[PATH_MAX];   // Name of file used to store log
                              // information of a user session.
FILE* fplog = NULL;           // For the log file - always set to NULL
                              // when not in use so it can also be used
                              // as a flag to let us know if the file is
                              // open.
int Log_wanted = 0; // A global to let us know if log entries to a file
                    // are to be attempted.

static int grab_image(void);

static void btn_cam_save_click(GtkWidget *,gpointer);
static int set_dims_as_per_selected(void);
void nullify_flatfield(void);
static int init_flatfield_image(int);
void nullify_darkfield(void);
void nullify_mask(void);
int test_selected_ff_filename(char *);
int test_selected_df_filename(char *);
int test_selected_msk_filename(char *);

static int open_device(void);
static int init_device(void);
static int uninit_device(void);
static int close_device(void);
static int start_streaming(void);
static int stop_streaming(void);
static int calculate_preview_params(void);
static void change_cam_status(int, char);
static void toggled_cam_preview(GtkWidget *,gpointer);

void fmtstr(int fmt,char *fstr);
int update_preview_settings(int);
int set_camera_control(unsigned int, int, char *);
int get_camera_control(unsigned int, int *);
static void calculate_yuyv_luts(void);
int raw_to_bmp(unsigned int,unsigned int, unsigned char **, char *,int); 
int try_running_camera(void);


// License notification
const char *License_note = "GNU GPL v3\n\nThis program is free software: you can redistribute it and/or modify\nit under the terms of the GNU General Public License as published by the\nFree Software Foundation, either version 3 of the License, or (at your\noption) any later version.\n\nThis program is distributed in the hope that it will be useful, but\nWITHOUT ANY WARRANTY; without even the implied warranty of\nMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\nGNU General Public License for more details.\n\nYou should have received a copy of the GNU General Public License along\nwith this program. If not, see <https://www.gnu.org/licenses/>.";


///// ----- /////


    ////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    .
   //                                                              \\   .
  //           CAMERA SETTINGS ENUMERATION FUNCTIONALITY            \\  .
 //                                                                  \\ .
////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\.

#define MAX_CAM_MENU_ITEMS 32 // The maximum number if menu items a
                              // camera menu setting structure can hold.

// Structure for a camera settings node
typedef struct {
    char         *name;
    unsigned int ctrl_id;
    int          minimum,maximum,step,currval;
    int          num_menuitems;
    char         *miname[MAX_CAM_MENU_ITEMS];
 } CamSetting;

CamSetting    *CSlist=NULL;   // To store a list of all camera settings
unsigned int        NCSs=0;   // Number of camera settings in the list
size_t           SizeCSs=0;   // Current size of camera settings list
char          *CSBlob=NULL;   // To hold a string of all settings values
unsigned int     SizeCSB=0;   // The CSBlob size
unsigned int    NCSlines=0;   // The number of lines allowed in CSBlob

int enumerate_cs_menu(__u32,unsigned int);

// Return values for loop_new()
#define CSE_SUCCESS 0 // No error. Camera setting created OK
#define CSE_MEMFAIL 1 // Failed. Couldn't get memory for the new camera setting.
#define CSE_MEMNAME 2 // Failed. Couldn't get memory for the name of the new camera setting.
#define CSE_MEMMENU 3 // Failed. Couldn't get memory for the name of the new camera setting menu item.
#define CSE_MAXMENU 4 // Too many menu items to store.  This is not fatal - just issue a warning.

int cs_new(unsigned int id, char *name, int minimum, int maximum,int step,int currval, int mtype)
// Adds a new camera setting node to the CSlist.
// Returns one of the CSE_ constants (which see)
{
 CamSetting *newmem;
 size_t newsize,nomlen;
 unsigned int idx;
 int fnresult;
 

 // Try to get memory for the new node
 newsize=SizeCSs+sizeof(CamSetting);
 newmem=realloc(CSlist,newsize);
 if(newmem==NULL){
    show_message("Failed to get memory for camera settings structure.\n","cs_new: ",MT_ERR,1);
    return CSE_MEMFAIL;
  }
 CSlist=newmem;
 SizeCSs=newsize;
 idx=NCSs;
 NCSs++;
 
 // Now set the values assigned to the camera settings node
 nomlen=strlen(name); nomlen++;
 if((CSlist[idx].name=(char *)malloc(nomlen))==NULL){
    show_message("Failed to get memory for camera setting name.\n","cs_new: ",MT_ERR,1);
    return CSE_MEMNAME;
   }
 memcpy(CSlist[idx].name,name,nomlen);
 CSlist[idx].ctrl_id=id;
 CSlist[idx].minimum=minimum;
 CSlist[idx].maximum=maximum;
 CSlist[idx].step=step;
 CSlist[idx].currval=currval;
 CSlist[idx].num_menuitems=0;
 
 fnresult=CSE_SUCCESS;
 
 if(mtype == V4L2_CTRL_TYPE_MENU) fnresult= enumerate_cs_menu(id,idx);

 return fnresult;
}

void cs_listfree(void)
// Frees the list of camera settings
{
 unsigned int idx,mdx;
 if(NCSs==0) return;
 for(idx=0;idx<NCSs;idx++){
   free(CSlist[idx].name);
   if(CSlist[idx].num_menuitems)
    for(mdx=0;mdx<CSlist[idx].num_menuitems;mdx++)free(CSlist[idx].miname[mdx]);
  }
 free(CSlist);
 CSlist=NULL;
 SizeCSs=0;
 NCSs=0;
 return;
}

int csblob_new(char *line)
// Adds a new line to the CSBlob list.
// Returns 0 on success and 1 on failure
{
 char *cp;
 size_t newsize,nomlen,oldsz;
 int idx;
 
 // Try to get memory for the new line
 nomlen=strlen(line); 

 newsize=SizeCSB+nomlen;
 cp=realloc(CSBlob,newsize*sizeof(char));
 if(cp==NULL){
    show_message("Failed to get memory for camera setting blob line.\n","csblob_new: ",MT_ERR,1);
    return 1;
  }
 oldsz=SizeCSB;
 CSBlob=cp;
 SizeCSB=newsize;
 NCSlines++;

 // Add the line to the newly extended CSBlob
 for(idx=0;idx<nomlen;idx++) CSBlob[idx+oldsz]=line[idx];

 return 0;
}

void csblob_free(void)
// Frees the list of camera settings blob lines
{
 if(NCSlines==0) return;
 free(CSBlob);
 CSBlob=NULL;
 NCSlines=0;
 SizeCSB=0;
 return;
}

int enumerate_camera_settings(void)
// Generates and populates a list of current camera settings
{
 char ctrl_name[64],msgtxt[320];
 int currval,returnval;
 
 // Free any pre-existing list
 cs_listfree();
 
 // Query the camera to get a list of all its settings & current values
 memset(&vd_queryctrl, 0, sizeof(vd_queryctrl));
 memset(&vd_control, 0, sizeof (vd_control));
 returnval=CSE_SUCCESS;
   
 vd_queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
 while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &vd_queryctrl)) {
      if (!(vd_queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
         // Query the vd_control and get current value, name and ranges
         vd_control.id = vd_queryctrl.id;
         if (0 == ioctl(fd, VIDIOC_G_CTRL, &vd_control)) currval=vd_control.value;
         else {
               // There will be an error for title nodes - you can't get
               // or set their values - so don't include these in true
               // error detection (a title is recognised by having a
               // zero range).
               if(vd_queryctrl.minimum == vd_queryctrl.maximum) goto add_node;
               // Otherwise print an error message:
               currval=0;
               sprintf(msgtxt,"%s (%s)",(char *)vd_queryctrl.name, strerror(errno));
               show_message(msgtxt,"VIDIOC_G_CTRL: ",MT_ERR,0);
              }
                  
         // Add a node to the camera settings list with those values
         // (it will also retrieve any menu items)
         add_node:
         sprintf(ctrl_name,"%s",vd_queryctrl.name); 
         returnval=cs_new(vd_queryctrl.id,ctrl_name,vd_queryctrl.minimum,vd_queryctrl.maximum,vd_queryctrl.step,currval,vd_queryctrl.type);
         switch(returnval){
           case CSE_SUCCESS: break;
           case CSE_MEMMENU: break;
           case CSE_MAXMENU: break;
           default: goto endof;
          }
        }
     vd_queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  }
 endof:
 return returnval;
}

int enumerate_cs_menu(unsigned int id,unsigned int sdx)
// Populates a camera settings node menu items list
{
 char ctrl_name[64];
 size_t nomlen;

    memset(&vd_querymenu, 0, sizeof(vd_querymenu));
    vd_querymenu.id = id;

    for (vd_querymenu.index = vd_queryctrl.minimum; vd_querymenu.index <= vd_queryctrl.maximum; vd_querymenu.index++){
       if (0 == ioctl(fd, VIDIOC_QUERYMENU, &vd_querymenu)){
            sprintf(ctrl_name,"%s", vd_querymenu.name);
            nomlen=strlen(ctrl_name); nomlen++;
            if((CSlist[sdx].miname[CSlist[sdx].num_menuitems]=(char *)malloc(nomlen))==NULL){
              show_message("Failed to get memory for camera setting menu item name.\n","enumerate_cs_menu: ",MT_ERR,1);
              return CSE_MEMMENU;
             }
           memcpy(CSlist[sdx].miname[CSlist[sdx].num_menuitems],ctrl_name,nomlen);
           CSlist[sdx].num_menuitems++; 
           if(CSlist[sdx].num_menuitems==MAX_CAM_MENU_ITEMS){
              show_message("Maximum number of menu items has been reached.\n","enumerate_cs_menu: ",MT_ERR,1);
              return CSE_MAXMENU;
           }        
        }
      } 
 return CSE_SUCCESS; 
}

int print_cs_file(const char *fname)
// Prints the current list of camera settings structures to disk as a
// plain text file using fname as the name of the file.
// Returns 0 on success and 1 on error
{
 FILE *fp;
 unsigned int sdx,mdx; // sdx for the settings, mdx for menu items
 
 if(NCSs==0){
     show_message("No point saving settings file as there are no camera settings to save.","File Save FAILED: ",MT_ERR,1);
     return 1;
 }
 
 fp=fopen(fname,"wb");
 if(fp==NULL){ show_message("Failed to open file for writing camera settings.","File Save FAILED: ",MT_ERR,1); return 1;}
 // Write a header to identify this file format
 fprintf(fp,"PCamSet 1 %u %d\n\n",NCSs,windex);
 // Now loop through the camera settings with their current values
 for(sdx=0;sdx<NCSs;sdx++){
   fprintf(fp,"\nidx:  %u\n",sdx);
   fprintf(fp,"name: %s\n",CSlist[sdx].name);
   fprintf(fp,"ctrl: %u\n",CSlist[sdx].ctrl_id);
   fprintf(fp,"min:  %d\n",CSlist[sdx].minimum);
   fprintf(fp,"max:  %d\n",CSlist[sdx].maximum);
   fprintf(fp,"step: %d\n",CSlist[sdx].step);
   fprintf(fp,"curr: %d\n",CSlist[sdx].currval);
   fprintf(fp,"mdx:  %d\n",CSlist[sdx].num_menuitems);
   if(CSlist[sdx].num_menuitems)
    for(mdx=0;mdx<CSlist[sdx].num_menuitems;mdx++)
       fprintf(fp,"%s\n",CSlist[sdx].miname[mdx]);
 }
 fflush(fp); fclose(fp); 
 return 0;
}

int list_camera_settings(void)
// Enumerates the available camera settings and put them into the CSBlob
// string. Memory will be allocated as required.
// Returns 0 on success.
// Returns 1 on error.
{
 int returnval;
 char line[64];
 unsigned int sdx,mdx; // sdx for the settings, mdx for menu items

  if(camera_status.cs_opened == 0) if(open_device()) return 1;
   
 // List the device's settings into the CSlist
 returnval=enumerate_camera_settings();
 switch(returnval){
    case CSE_SUCCESS: break;
    case CSE_MEMMENU: break; // Non-fatal
    case CSE_MAXMENU: break; // Non-fatal
    default: return 1;
   }
 if(NCSs==0){
     show_message("No settings to save.","list_camera_settings: ",MT_ERR,1);
     return 1;
 } 
 // Free any previous blob info
 csblob_free();
 
 // Add the total number of settings entries at the start of the blob
 sprintf(line,"%u\n",NCSs);
 if(csblob_new(line)) return 1;
 
 // Add each line of settings to the blob list.
  for(sdx=0;sdx<NCSs;sdx++){
   sprintf(line,"%u\n",sdx);      
   if(csblob_new(line)) return 1;
   sprintf(line,"%s\n",CSlist[sdx].name);
   if(csblob_new(line)) return 1;     
   sprintf(line,"%u\n",CSlist[sdx].ctrl_id);      
   if(csblob_new(line)) return 1;
   sprintf(line,"%d\n",CSlist[sdx].minimum);
   if(csblob_new(line)) return 1;     
   sprintf(line,"%d\n",CSlist[sdx].maximum);
   if(csblob_new(line)) return 1;     
   sprintf(line,"%d\n",CSlist[sdx].step);
   if(csblob_new(line)) return 1;     
   sprintf(line,"%d\n",CSlist[sdx].currval);
   if(csblob_new(line)) return 1;     
   sprintf(line,"%d\n",CSlist[sdx].num_menuitems);
   if(csblob_new(line)) return 1;     
   if(CSlist[sdx].num_menuitems)
    for(mdx=0;mdx<CSlist[sdx].num_menuitems;mdx++){
       sprintf(line,"%s\n",CSlist[sdx].miname[mdx]);
       if(csblob_new(line)) return 1;     
      }
 }

 return 0;  
}

// Return values for check_camera_setting()
#define CSC_OK      0 // All good
#define CSC_NOCS    1 // No settings are loaded
#define CSC_NOID    2 // The proferred control ID is not in the list of
                      // controls for the camera currently being used.
#define CSC_RANGE   3 // The proferred control value is outside the
                      // range supported by that control for this camera

int check_camera_setting(unsigned int idx, int ival, int *csdx)
// Checks to see if the proferred camera control change is possible by
// comparing idx and ival to the database of current camera controls
// available.
// idx - the control ID
// ival - the value the user wants to set the control to.
// Returns one of the CSC_ #define constants (which see).
// If all went well, it sets the value of *csdx to the index of the
// control
{
    int cdx;

    if (NCSs == 0) return CSC_NOCS;

    for (cdx = 0; cdx < (int)NCSs; cdx++)
        if (CSlist[cdx].ctrl_id == idx){
            if (ival<CSlist[cdx].minimum || ival>CSlist[cdx].maximum)
             return CSC_RANGE; else { *csdx = cdx; return CSC_OK; }
          }
    return CSC_NOID;
}

int ncsidx_from_ctrl_id(int ctrlidx)
// Return the CSlist index of a control from the control ID number given
// a proposed control ID number (ctrlidx).
// Return "-1" if there are no entries in the CSlist or if the supplied
// ctrlidx value is not found in all the CSlist entries.
{
    int cdx;

    if (NCSs == 0) return -1;

    for (cdx = 0; cdx < (int)NCSs; cdx++)
        if (CSlist[cdx].ctrl_id == ctrlidx){
             return cdx;
          }

    return -1;
}

int windex_from_widget(GtkWidget *widget)
// Return the widget index of a camera setting widget that matches the
// supplied widget address.
// Return -1 if the supplied widget address does not match any of the
// widgets in the CamsetWidget[] array.
{
 int idx;

 for(idx=0;idx<windex;idx++){
    if(CamsetWidget[idx]==widget) return idx;
   }
 return -1;
}

int test_framerate_resolutions(unsigned int lwd, unsigned int lht, unsigned int cfmt)
// Test to see if the camera admits an image dimension set of lwdxlht
// for a format stream of format cfmt. This is used when checking a
// settings file settings - it does not set any globals or GUI controls.
// Return 0 if it is supported and 1 if not
{
 int fdx,selectedfdx;
 struct v4l2_frmsizeenum frmsizeenum;
 
  if(!camera_status.cs_opened) if(open_device()) return 1;

 // Query the camera to list the device's frame sizes for the YUYV 
 // format into the frame size drop-down list

   memset(&frmsizeenum, 0, sizeof(frmsizeenum));
   frmsizeenum.pixel_format=cfmt; 
   fdx=0; frmsizeenum.index=fdx;
   
   // Test the supplied lwd andlht combination against those supported
   // by the camera for image stream of cfmt
   selectedfdx = 0;
   while(0 == ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum)){
         // PARD Capture does not yet support these frame heights:
         if(frmsizeenum.discrete.height==288) goto Next_resolution;
         if(frmsizeenum.discrete.height==144) goto Next_resolution;

         if(lwd==frmsizeenum.discrete.width && lht==frmsizeenum.discrete.height) selectedfdx++;
         Next_resolution:
         fdx++; // index of next frame size to query  
         frmsizeenum.index=fdx;
        }
   // See if we found a match:
   if(selectedfdx) return 0;
   
 return 1;
}

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////
 //                                                                  // 
  //          CAMERA SETTINGS ENUMERATION FUNCTIONALITY             //
   //                                                              //
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////



    ////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\    .
   //                                                              \\   .
  //                AUDIO GUI FEEDBACK FUNCTIONALITY                \\  .
 //                                                                  \\ .
////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\.



int AudioInit()
{
 char msgtxt[256];
 int retval=0;
 int error;
 // The Sample format to use
 static const pa_sample_spec ss = {
     .format = PA_SAMPLE_S16LE,
     .rate = 44100,
     .channels = 2
 };

 if(Audio_status!=AS_NULL) return 1; 
 
 // Create a new playback stream 
 if (!(PA_s=pa_simple_new(NULL,"a_beep",PA_STREAM_PLAYBACK,NULL,"playback",&ss,NULL,NULL,&error))){
    sprintf(msgtxt,"pa_simple_new() failed: %s\n", pa_strerror(error));
    show_message(msgtxt,"AudioInit: ",MT_ERR,0);
    retval=1;
  }

  if(retval==0) Audio_status=AS_INIT;
  return retval; 
}

int AudioUnInit(void)
{
  if(Audio_status!=AS_INIT) return 1; 
  if (PA_s) pa_simple_free(PA_s);
  Audio_status=AS_NULL; 
  return 0;
}

int a_beep(int duration, int pitch)
// Prinitive beeper function using PulseAudio-simple lib APIs.
// duration = duration of beep
// pitch = the octave: supply 0, 1, 2 or 3.
{
 uint8_t *audiobuffer;
 int idx,sdx;
 int error;

if(Use_audio==AU_NO) return 0;
// One beep at a time. This stops multiple parallel threads
// executing the beep function. A further safeguard is below.
if(Audio_sounding) return 0; 
Audio_sounding=1;
// Initialise sound generating capability
 if(AudioInit()){Audio_sounding=0; return 1;}

 audiobuffer=(uint8_t *)calloc((size_t)(PABUFSIZE),sizeof(uint8_t));
 if(audiobuffer==NULL){
   show_message("Failed to get memory for audio buffer.","a_beep: ",MT_ERR,0);  
   AudioUnInit();
   Audio_sounding=0;
   return 1;
  }

 for( idx=sdx=0; idx<PABUFSIZE; idx++ ){
      audiobuffer[idx]=sine_buffer[sdx];
      sdx+=pitch;
      if(sdx>=PABUFSIZE) sdx=0;
     }

 if(duration<20) duration =20;

 for(idx=0;idx<duration;idx++){
 if(pa_simple_write(PA_s,audiobuffer,(size_t)PABUFSIZE,&error)<0){
    fprintf(stderr,"pa_beep: pa_simple_write() failed: %s\n", pa_strerror(error));
    goto uninit_audio;
   }
   usleep(200);
  }
usleep(100000);
  if (pa_simple_drain(PA_s,&error)<0){
        fprintf(stderr, "pa_beep: pa_simple_drain() failed: %s\n", pa_strerror(error));
        goto uninit_audio;
    }
uninit_audio:
        AudioUnInit();
free(audiobuffer);
Audio_sounding=0;
 return 0;
}

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////
 //                                                                  // 
  //                AUDIO GUI FEEDBACK FUNCTIONALITY                //
   //                                                              //
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////



////////////////////////////////////////////////////////////////////////
//                                                                    //
//      PROGRAM UTILITY FUNCTIONS                                     //
//                                                                    //
////////////////////////////////////////////////////////////////////////



int resize_memblk(void **ptr,size_t blocksz, size_t unitsize, char *pname)
// Change the size of a general pointer to a memory block.
// It is assumed that the pointer pointed to by *ptr already has some
// memory alloced to it (e.g. via malloc or calloc) prior to calling
// this function. If it does not, this will cause a memory error.
// pname is the name you want associated with the pointer for error
// message purposes. Keep this to under 64 chars.
{
 char emsg[256];
 
 if(*ptr==NULL) return 1; // Avoid 'double free' errors if multiple
                          // threads start calling out of sync. This
                          // ought not to happen so return with an error
                          // and let the debug process begin!
 free(*ptr); *ptr=NULL; // See above.
 *ptr=calloc(blocksz,unitsize);
 if(*ptr==NULL){
    sprintf(emsg,"Failed to get %zu memory units for %s.",blocksz,pname);
    show_message(emsg,"Memory Error: ",MT_ERR,1);
    *ptr=calloc(1,unitsize);
    if(*ptr==NULL){
      sprintf("Could not get even re-initialise %s. This is a serious memory error.\nPlease save your work and exit ASAP because the program may crash at any time without further notice.",pname);
      show_message(emsg,"Memory Error: ",MT_ERR,1);
      return 1;
    } 
    return 1;
  }
 return 0;
}

int arg_count(char *line)
{
    int numtok, len, idx;
    char wspc[] = " \t", * tok, linecp[MAX_CMDLEN];

    len = strlen(line);
    if (len == 1) return 0; // Blank non-coding line  

    numtok = 0;

    // Check for lines that are entirely white space (i.e. blank non-
    // coding lines)
    for (idx = 0; idx < len; idx++)  if (isspace(line[idx])) continue; else numtok++; 
    if (numtok == 0) return 0; // Blank non-coding line  

    memcpy(linecp, line, MAX_CMDLEN);
    numtok = 0;
    tok = linecp;
    while ((tok = strtok(tok, wspc)) != NULL) {
        numtok++;
        tok = NULL;
    }

    return numtok;
}

void put_entry_txt(char *vstr,GtkWidget *gentry)
// Puts an string into a GtkEntry box ensuring there is no trailing
// space.
{
 size_t slen,idx;

 slen=strlen(vstr);
 for(idx=0;idx<slen;idx++) if(isspace((int)vstr[idx])){vstr[idx]='\0'; break;}
 gtk_entry_set_text(GTK_ENTRY(gentry), vstr);
 return;
}

int pcs_argc_check(int argcount, int low, int high, int wrong, char *cmd, char *errmsg)
// Check the validity of the number of arguments for a command in a PCS
// script.
// argcount is the actual number of arguments on the line (including the
// prime command)
// low is the minimum number allowed for this command
// high is the maximum number allowed for this command
// wrong is a forbidden number of commands for this argument (submit 0
// if there is no forbidden number)
// cmd is the name of the script command
// errmsg is the error string to be printed if there is a problem
// Returns 0 on success and 1 on error
{
    if (argcount < low) { sprintf(errmsg, "Too few arguments for %s.", cmd); return 1; }
    if (argcount > high) { sprintf(errmsg, "Too many arguments for %s.", cmd); return 1; }
    if (wrong) if (argcount == wrong) { sprintf(errmsg, "Wrong number of arguments for %s.", cmd); return 1; }
    return 0;
}

int is_not_integer(const char *instr)
// Checks if a string instr does not represent an integer (with an
// optional + or - in front)
// Return 1 if it is NOT an integer, 0 otherwise.
{
    size_t len, idx;

    len = strlen(instr);
    if (len < 1) return 1; // An empty string is not an integer

    //Ignore leading spaces
    for(idx=0;idx<len;idx++) if(!isspace(instr[idx])) break;
    if(idx==len) return 1; // A string composed entirely of white space
    
    // Check for an optional sign
         if (instr[idx] == '-') idx++;
    else if (instr[idx] == '+') idx++;

    for (; idx < len; idx++)  if (isdigit(instr[idx])) continue; else break;

    //Ignore trailing spaces
    for (; idx < len; idx++) if(isspace(instr[idx])) continue; else return 1;

    return 0;
}

int is_not_float(const char *instr)
// Checks if a string instr does not represent a float (with an optional
// + or - in front and optionally containing a decimal point and/or
// exponential notation with up to 3 digits after the exponent mark).
// Returns 0 if it is a properly formatted float.
// Returns non-zero if it is NOT a properly formatted float and this
// value may be:
// 1 - not recognised as any type of float
// 2 - the substring #IND occurs in it
// 3 - the substring #INF occurs in it

{
    int len, idx;
    int pre,post,expo,prepoint,preexp,ndigits;

    len = strlen(instr);
    if (len < 1) return 1; // An empty string is not a float
    
    //Ignore leading spaces
    for(idx=0;idx<len;idx++) if(!isspace(instr[idx])) break;
    if(idx==len) return -1; // A string composed entirely of white space
    
    // Check for an optional sign
         if (instr[idx] == '-') idx++;
    else if (instr[idx] == '+') idx++;

    pre = 0; // The number of digits before the decimal point
    post= 0; // The number of digits after the decimal point
    expo= 0; // the number of digits after e+, e- E+ or E-
             // (i.e. an exponent mark)
    prepoint=1; // Set to zero when a decimal point is found and to a
                // negative number if there is >1 point (an error)
    preexp=1;   // Set to zero when 1 exponent mark is found and to a
                // negative number if there is >1 mark (an error)
    for (; idx < len; idx++){
      if(prepoint<0) return 1; // Too many decimal points
      if(preexp<0) return 1;   // Too many exponent marks
      // We are only permitted to find a digit, a decimal point, a # or
      // an e (pr E):
      if(isdigit(instr[idx])){
          // If there are too many exponent digits, return an error
          if(preexp==0){ expo++; if(expo>3) return 1; }
          else if(prepoint) pre++;
          else post++;        
          continue;
        }
      else if(instr[idx] == '.'){prepoint--; continue;}
      else if(instr[idx] == 'e' || instr[idx] == 'E'){
          // This is the beginning of an exponent mark or an error -
          // so check which it is:
          idx++;
          if(instr[idx] == '+' || instr[idx] == '-'){ // It's an expo
             preexp--;
             if(prepoint==1) prepoint=0; // Any decimal point after this
                                         // will be invalid.
             continue;
            }  else return 1; // It's an error
       }
      else if(instr[idx] == '#'){ 
          // This will be an error, but we check to see if we can
          // distinguishe #IND from #INF
          idx++;
          if(instr[idx] == 'I' && instr[idx+1] == 'N' && instr[idx+2] == 'D') return 2;
          if(instr[idx] == 'I' && instr[idx+1] == 'N' && instr[idx+2] == 'F') return 3;
          return 1;
          
      }

     }

   // There must be at least one digit prior to any exponent,
   // so check for this:
   ndigits=pre+post;
   if(ndigits==0) return 1;

   // If we got here then the string represents a properly formatted
   // float number.
    return 0;
}

int is_not_yesno(const char *instr)
// Checks if a string instr is either "Yes" or "No" (case sensitive)
// Returns 1 if it is neither Yes or No, 0 otherwise.

{
 if(strlen(instr)>3) return 1;
 if(strlen(instr)<2) return 1;
 if(strcmp(instr,"Yes")) return 0;
 if(strcmp(instr,"No")) return 0;
 return 1;
}

int set_combo_sz_index(int wd,int ht)
// Set the currently selected image dimensions combo box entry to the
// one that has dimensions wd x ht. Set it to 0 if wdxht is not found.
// Return 0 on success, 1 if the chosen dimensions were not found in the
// drop down list.
{
 int idx,lw,lh;
 gchar *numstr;
 
 for(idx=0;idx<Nresolutions;idx++){
  gtk_combo_box_set_active(GTK_COMBO_BOX (combo_sz), idx);
  numstr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_sz));
  sscanf(numstr,"%d %*s %d",&lw,&lh);
  g_free(numstr);
  if(lw==wd && lh==ht) return 0;
 }
 return 1;
}


#define PCS_NULL    1 // fgets returned NULL (end of file was reached)
#define PCS_SKIP    2 // a blank line or a comment line
#define PCS_OK      3 // line read OK, argcount and argstr1 contain
                      // relevant information

int read_pcs_line(FILE *fp, char *line, unsigned int *linenum, int *argcount, char *argstr1)
// Read the next line of a PARDUS Command Script (PCS) or similar text
// file and process it to remove preceeding and trailing white space and
// count the number of arguments (words) on the line.
// Blank lines and comment lines (that begin with #) are ignored.
// Put the processed line into line, the argument count into argcount
// and the first argument on the line into argstr1.
// Return a value that os one of the 'PCS_' #defined constants according
// to how the process has gone.
{
    int idx, c, prec, odx;
    char cleanline[MAX_CMDLEN];

    // Read the next line of the file
    if (NULL == fgets(line, MAX_CMDLEN, fp)) return PCS_NULL; 
    // Increment the source line number marker 
    (*linenum)++;         
    // Count the number of words on the line                                     
    *argcount = arg_count(line);
    // Skip blank lines
    if (*argcount < 1) return PCS_SKIP;
    // Read the first word on the line              
    sscanf(line, "%s", argstr1);  
    // Skip comment lines                             
    if (!strncmp(argstr1, "#", 1)) return PCS_SKIP;           

    // Strip preceeding whitespace and terminal newline 
    // (or post-command comment)
    for (idx = odx = prec = 0; idx < MAX_CMDLEN; idx++) {
        c = (int)line[idx];
        if (c == '\n') break;
        if (c == '#')  break;
        if (isspace(c)) { if (!prec) continue; }
        else prec = 1;
        cleanline[odx++] = line[idx];
    }
    // Strip trailing whitespace  
    idx--;
    for (; idx > 0; idx--) {
        c = (int)line[idx];
        if (isspace(c)) odx--; else break;
    }

    cleanline[odx] = '\0';
    memcpy(line, cleanline, MAX_CMDLEN);

    // Update the count now all trailing whitespace chars are removed.
    *argcount = arg_count(line);  // Count the No. of words on the line
    if (*argcount < 1) return PCS_SKIP; // Skip blank lines

    return PCS_OK;
}

int csetfile_check(FILE *fp, unsigned int *linenum, char *errmsg)
// Checks validity and syntax of a supposed PARD Capture settings file
// *fp is a pointer to the file and must be open and at the beginning.
// *linenum will contain the line number of the first error encountered.
// *errmsg will contain a textual explanantion of any error encountered.
// Returns PCHK_ALL_GOOD if there are no errors or one of the #defined
// PCHK_ error codes.
{
    int   returnvalue,argcount,inum1,mdx,line_status,idx,cidx;
    char  line[MAX_CMDLEN], argstr1[64], argstr2[64];
    char  argstr3[64], argstr4[64], argstr5[256];
    char  imsg[MAX_CMDLEN+32];
    int   lht, lwd;      // The proposed frame resolution
    unsigned int   cfmt; // The proposed camera stream format
    int   cfmt_selected, dims_selected,custset_found,header_wrong;
    int   file_exists = 0;

    header_wrong=1;  // For checking the settings file header
    custset_found=0; // So we know that the custom settings version has
                     // been checked.
    cfmt_selected=0; // Whether a new camera stream format was selected.
    dims_selected=0; // Whether the image frame size was selected.
    mdx=0; // Menu items = number of lines to skip in this check
    *linenum = 0;
    returnvalue = PCHK_TERMINUS; // Ensures improper termination flag is
                                 // returned if the file ends before the
                                 // proper exit signal is picked up.

    show_message("Checking settings file ...\n", "FYI: ", MT_INFO, 0);

    while (!feof(fp)) {
         
        // Read next source code line
        line_status = read_pcs_line(fp, line, linenum, &argcount, argstr1); 
        // If end of file, break out of while loop
        if (line_status == PCS_NULL) break; 
        // If get here then at least the file has something to read
        file_exists=1;
        // If this is a non-coding line, skip it.   
        if (line_status == PCS_SKIP) continue; 
        // If this is a menu item line, skip it.   
        if (mdx > 0){mdx--; continue;} 
        // The first line should state the magic word, version and
        // number of camera settings (4 args), so check for that:
        if (*linenum == 1) {
            // Are there 4 arguments (only)?
            if (argcount != 4) {
              sprintf(errmsg, "Invalid PARDUS settings file header.");
              returnvalue = PCHK_E_FORMAT;
              break;
             }
            // Read all 4 arguments in the line:
            sscanf(line, "%s %s %s %s", argstr1, argstr2, argstr3, argstr4);
            // Magic word must be PCamSet
            if (strcmp(argstr1, "PCamSet")) {
               sprintf(errmsg, "Not a valid PARDUS settings file. It does not begin with PCamSet.");
               returnvalue = PCHK_E_FORMAT; break;
              }
            // The current version is 1
            if (strcmp(argstr2, "1")) {
               sprintf(errmsg, "%s: The chosen settings file version ('%s') is incompatible with the version used by this program (1).", argstr1, argstr2);
               returnvalue = PCHK_E_FORMAT; break;
              }
            // The number after this is the number of used camera
            // settings. This must equal the NCSs number of the current
            // camera else the settings in the file are not compatible:
            inum1 = atoi(argstr3);
            if((unsigned int)inum1!=NCSs){
               sprintf(errmsg, "%s: '%s' is not equal to the current number of camera settings (%u).", argstr1, argstr3,NCSs);
               returnvalue = PCHK_E_FORMAT; break;
              }
            // The number after this is the number of widgets used to
            // store the settings in the GUI.
            // This must equal the global windex value of the current
            // camera else the settings in the file are not compatible:
            inum1 = atoi(argstr4);
            if(inum1!=windex){
               sprintf(errmsg, "%s: '%s' is not equal to the current number of camera control entry boxes (%d).", argstr1, argstr4, windex);
               returnvalue = PCHK_E_FORMAT; break;
              }
            header_wrong=0;
            continue;
        } else if(header_wrong){
               sprintf(errmsg, "%s: The header was not found on the first line.", argstr1);
               returnvalue = PCHK_E_FORMAT; break;
              }
        // If we have got to here we know that this is not a comment,
        // not a blank line and not the first line. We also know that
        // the first line in the file is of the correct format and the
        // first word of the current line is in argstr1.
        // The total number of words on the line is in argcount.
        // Proceed according to the first word (i.e. the prime command).
        // The next check in each case is whether the argument count
        // is appropriate to the prime command:
        sprintf(imsg, "\t[%3d]: %s\n", *linenum, line); show_message(imsg, "", MT_INFO, 0);
        if (!strcmp(argstr1, "idx:")) {
            // idx: <U.INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an unsigned int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the index into inum1
            if(inum1<0){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: '%s' must be >= 0.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "name:")) {
            // name: <string1> [<string2> [<string3>] ...]
            // For the OptArc cameras the control name will not be more
            // than 3 strings long so argcount won't be >4.
            if(pcs_argc_check(argcount, 2, 4, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Construct the full name into argstr5 according to how
            // many strings make it up:
            switch(argcount) {
              case 2:
                sscanf(line, "%s %s", argstr1, argstr5);
              break;
              case 3:
                sscanf(line, "%s %s %s", argstr1, argstr2, argstr3);
                sprintf(argstr5,"%s %s",argstr2, argstr3);
              break;
              case 4:
                sscanf(line, "%s %s %s %s", argstr1, argstr2, argstr3,argstr4);
                sprintf(argstr5,"%s %s %s",argstr2,argstr3,argstr4);
              break;
              default:
              break; // pcs_argc_check ensures we not get here.
            }
            // Now check that corresponds to the current control index
            // read previously:
            if(strcmp(argstr5,CSlist[inum1].name)){
               sprintf(errmsg, "%s: '%s' is not the name of control %d.", argstr1, argstr5,inum1);
               returnvalue = PCHK_E_SYNTAX; break;
              }
          }
        else if (!strcmp(argstr1, "ctrl:")) {
            // ctrl: <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the control ID into inum1
            // Now ensure this control ID is in the list of control IDs
            // associated with an entry box (or we can't set its value):
            for(idx=0,cidx=1;idx<windex;idx++){
              if (ctrl_id[idx]==inum1){cidx=0; break;}             
             } 
            if(cidx){
             sprintf(errmsg, "%s: Control index '%s' is not associated with an entry box.", argstr1, argstr2);
             returnvalue = PCHK_E_SYNTAX; break;
            }
          }
        else if (!strcmp(argstr1, "min:")) {
            // min: <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "max:")) {
            // max: <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "step:")) {
            // step: <U.INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an unsigned int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the step size number into inum1
            if(inum1<0){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: '%s' must be >= 0.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "curr:")) {
            // curr: <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "mdx:")) {
            // mdx: <U.INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg))
             { returnvalue = PCHK_E_SYNTAX; break; }
            // We expect argstr2 to be an unsigned int - so check this
            // Read all arguments in the line
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            mdx=atoi(argstr2); // Get the number of menu items
            if(mdx<0){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: '%s' must be >= 0.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "PCustSet")) {
            // PCustSet <Version_integer>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Currently only version 1 is supported, so check for it.
            sscanf(line, "%s %s", argstr1, argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the version number
            if(inum1!=1){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: Version '%s' is not supported.", argstr1, argstr2);
               break;
              }
            custset_found=1;
          }
        else if (!strcmp(argstr1, "windex_sz")) {
            // windex_sz <width> <x> <height> <at> <framerate> <fps>
            if(pcs_argc_check(argcount, 7, 7, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // We only use <width> and <height>. Check they are each >0
            // and that the combination is represented in the camera's
            // current list of resolutions (or they can't be loaded):
            sscanf(line, "%s %s %*s %s", argstr1, argstr2,argstr3);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: Width value '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            lwd=atoi(argstr2); // Get the width
            if (is_not_integer(argstr3)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: Height value '%s' is not an integer.", argstr1, argstr3);
                break;
               }
            lht=atoi(argstr3); // Get the height
            if(lwd<1){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: Width '%s' is not supported.", argstr1, argstr2);
               break;
              }
            if(lht<1){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: Height '%s' is not supported.", argstr1, argstr3);
               break;
              }
            dims_selected=1; // So we know to check at the end if these
                             // are supported by the camera. We wait
                             // till the end because we need to see what
                             // camera stream format the settings file
                             // specifies because thay may determine
                             // what frame sizes are available. 
          }
        else if (!strcmp(argstr1, "windex_fps")) {
            // windex_fps <fps>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // <fps> must be a positive integer and be present in the 
            // available range:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: FPS value '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the fps
            if(inum1<1){ // Check it is >=1
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An FPS of '%s' is not supported.", argstr1, argstr2);
               break;
              }
            if(!fps_index(inum1)){ // Check it is in the list of FPS
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An FPS of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_camfmt")) {
            // windex_camfmt <string1> [<string2>]
            // For the OptArc cameras the format name will not be more
            // than 2 strings long so argcount won't be >3.
            if(pcs_argc_check(argcount, 2, 3, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Construct the full name into argstr5 according to how
            // many strings make it up:
            switch(argcount) {
              case 2:
                sscanf(line, "%s %s", argstr1, argstr5);
              break;
              case 3:
                sscanf(line, "%s %s %s", argstr1, argstr2, argstr3);
                sprintf(argstr5,"%s %s",argstr2, argstr3);
              break;
              default:
              break; // pcs_argc_check ensures we not get here.
            }
            // Now check that corresponds to an available format:
            switch(camfmt_from_string(argstr5)){
              case CAF_YUYV: cfmt=V4L2_PIX_FMT_YUYV; break;
              case CAF_MJPEG: cfmt=V4L2_PIX_FMT_MJPEG; break;
              default:
               sprintf(errmsg, "%s: Stream format '%s' is not available.", argstr1, argstr5);
               returnvalue = PCHK_E_SYNTAX;
               break;
              }
            if(returnvalue == PCHK_E_SYNTAX) break;
            cfmt_selected=1; // So we know a camera format is requested.
          }
        else if (!strcmp(argstr1, "windex_safmt")) {
            // windex_safmt <string1> [<string2>]
            // The 'save as' format name will not be more
            // than 2 strings long so argcount won't be >3.
            if(pcs_argc_check(argcount, 2, 3, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Construct the full name into argstr5 according to how
            // many strings make it up:
            switch(argcount) {
              case 2:
                sscanf(line, "%s %s", argstr1, argstr5);
              break;
              case 3:
                sscanf(line, "%s %s %s", argstr1, argstr2, argstr3);
                sprintf(argstr5,"%s %s",argstr2, argstr3);
              break;
              default:
              break; // pcs_argc_check ensures we not get here.
            }
            // Now check that corresponds to an available format:
            if(saveas_from_string(argstr5)<0){
               sprintf(errmsg, "%s: Save-as format '%s' is not available.", argstr1, argstr5);
               returnvalue = PCHK_E_SYNTAX;
               break;
              }
            cfmt_selected=1; // So we know a camera format is requested.
          }
        else if (!strcmp(argstr1, "windex_imroot")) {
            // windex_imroot <fname>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // <fname> must not be an empty string:
            sscanf(line, "%s %s", argstr1,argstr2);
            if(strlen(argstr2)<1){ // Check it is >=1
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An empty root name is not supported.", argstr1);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_fno")) {
            // windex_fno <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_avd")) {
            // windex_avd <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(0, 4097,"Frame averaging (number of frames)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_to")) {
            // windex_to <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(3, 361,"Grabber timeout (seconds)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_rt")) {
            // windex_rt <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(-1, 4097,"Frame capture (number of retries)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_srn")) {
            // windex_srn <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(0, 604801,"Series (number of images)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_srd")) {
            // windex_srd <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(-1, 86401,"Min. interval for series (s)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_jpg")) {
            // windex_jpg <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(0, 101,"JPEG save quality", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_gn")) {
            // windex_gn <FLOAT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be a valid float:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_float(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an valid floating point number.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_bs")) {
            // windex_bs <FLOAT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be a valid float:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_float(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an valid floating point number.", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_del")) {
            // windex_del <INT>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be an integer:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_integer(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not an integer.", argstr1, argstr2);
                break;
               }
            inum1=atoi(argstr2); // Get the value and check its range:
            if(cs_int_range_check(-1, 172801,"Delay first capture (s)", inum1,0)){ 
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: A value of '%s' is not supported.", argstr1, argstr2);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_pc")) {
            // windex_pc <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_yo")) {
            // windex_yo <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_sad")) {
            // windex_sad <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_fit")) {
            // windex_fit <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_smf")) {
            // windex_smf <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_ud")) {
            // windex_ud <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_uf")) {
            // windex_uf <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_um")) {
            // windex_um <Yes/No>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // Must be Yes or No:
            sscanf(line, "%s %s", argstr1,argstr2);
            if (is_not_yesno(argstr2)) {
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: '%s' is not 'Yes' or 'No' (case sensitive).", argstr1, argstr2);
                break;
               }
          }
        else if (!strcmp(argstr1, "windex_rdfi")) {
            // windex_rdfi <fname>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // <fname> must not be an empty string:
            sscanf(line, "%s %s", argstr1,argstr2);
            if(strlen(argstr2)<1){ // Check it is >=1
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An empty file name is not supported.", argstr1);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_rffi")) {
            // windex_rffi <fname>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // <fname> must not be an empty string:
            sscanf(line, "%s %s", argstr1,argstr2);
            if(strlen(argstr2)<1){ // Check it is >=1
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An empty file name is not supported.", argstr1);
               break;
              }
          }
        else if (!strcmp(argstr1, "windex_rmski")) {
            // windex_rmski <fname>
            if(pcs_argc_check(argcount, 2, 2, 0, argstr1, errmsg)){
               returnvalue = PCHK_E_SYNTAX;
               break; 
              }
            // <fname> must not be an empty string:
            sscanf(line, "%s %s", argstr1,argstr2);
            if(strlen(argstr2)<1){ // Check it is >=1
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "%s: An empty file name is not supported.", argstr1);
               break;
              }
          }              
        else if (!strcmp(argstr1, "exit")) {
            // There are no more settings to read. Before we go let us
            // see if new image dimensions were selected +/- a new image
            // stream format and check if these dimensions are supported
            // by the current camera in the chosen stream format.
            if(dims_selected){
             if(!cfmt_selected) cfmt=CamFormat;
             if(test_framerate_resolutions((unsigned int)lwd,(unsigned int)lht,cfmt)){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "The WxH (%dx%d) is not supported at the chosed stream format (%d).", lwd,lht,cfmt);
               break;              
             }
            }
            if(!custset_found){
               returnvalue = PCHK_E_SYNTAX; 
               sprintf(errmsg, "The custom settings header could not be found.");
               break;              
             }
            returnvalue = PCHK_ALL_GOOD; break;
        } else {// An unrecognised command:
                sprintf(errmsg,"%s", argstr1);
                returnvalue = PCHK_E_COMMND; break;
        }


    } // End of master 'While' loop.

  if(!file_exists){
    sprintf(errmsg, "Could not read data from the selected file (it may be empty).");
    returnvalue = PCHK_E_FORMAT;
   }


    return returnvalue;
}

int csetfile_load(FILE *fp, unsigned int *linenum, char *errmsg)
// Loads a PARD Capture settings file and attempts to set all controls
// to the values provided in that file. You should ensure such a file is
// first checked for validity using csetfile_check before calling this
// function because it does not make all those safety checks again here.
// *fp is a pointer to the file and must be open and at the beginning.
// *linenum will contain the line number of the first error encountered.
// *errmsg will contain a textual explanantion of any error encountered.
// Returns PCHK_ALL_GOOD if there are no errors or one of the #defined
// PCHK_ error codes.
{
    int   returnvalue, argcount, inum1, mdx, line_status,idx;
    char  line[MAX_CMDLEN], argstr1[64], argstr2[64];
    char  argstr3[64], argstr5[256];
    char  imsg[MAX_CMDLEN+32];
    int   esdx,dfoff,ffoff,mskoff;

    mdx=0;   // Menu items = number of lines to skip in this check
    esdx=0;  // To detect non-fatal errors.
    dfoff=0; // 'Use dark field?' must be switched off flag
    ffoff=0; // 'Use flat field?' must be switched off flag
    mskoff=0; // 'Use mask?' must be switched off flag
 
    *linenum = 0;
    returnvalue = PCHK_TERMINUS; // Ensures improper termination flag is
                                 // returned if the file ends before the
                                 // proper exit signal is picked up.

    show_message("Loading settings file ...\n", "FYI: ", MT_INFO, 0);

    while (!feof(fp)) {
    
        // Read next source code line
        line_status = read_pcs_line(fp, line, linenum, &argcount, argstr1); 
        // If end of file, break out of while loop
        if (line_status == PCS_NULL) break; 
        // If this is a non-coding line, skip it.   
        if (line_status == PCS_SKIP) continue; 
        // If this is a menu item line, skip it.   
        if (mdx > 0){mdx--; continue;} 
        // The first line has the magic word, etc. and can be skipped:
        if (*linenum == 1) continue;

        // If we have got to here we know that this is not a comment,
        // not a blank line and not the first line. We also know that
        // the first line in the file is of the correct format and the
        // first word of the current line is in argstr1.
        // The total number of words on the line is in argcount.
        // Proceed according to the first word (i.e. the prime command).
        // The next check in each case is whether the argument count
        // is appropriate to the prime command:
        sprintf(imsg, "\t[%3d]: %s\n", *linenum, line);
        show_message(imsg, "", MT_INFO, 0);
        if (!strcmp(argstr1, "idx:")) {       // Not used
          }
        else if (!strcmp(argstr1, "name:")) { // Not used
          }
        else if (!strcmp(argstr1, "ctrl:")) {
            // curr: <INT>
            // Get the control ID number into inum1
            sscanf(line, "%s %s", argstr1, argstr2);
            inum1=atoi(argstr2);
          }
        else if (!strcmp(argstr1, "min:")) {  // Not used
          }
        else if (!strcmp(argstr1, "max:")) {  // Not used
          }
        else if (!strcmp(argstr1, "step:")) { // Not used
          }
        else if (!strcmp(argstr1, "curr:")) {
            // curr: <INT>
            // Get the widget index for the entry box of this control
            for(idx=0;idx<windex;idx++){
             if (ctrl_id[idx]==inum1)  break;
            }
            // Skip non-entry box controls
            if(cswt_id[idx]!=CS_WTYPE_ENTRY) continue;
            // Set the control entry box to have the desired value:
            sscanf(line, "%s %s", argstr1, argstr2);
            put_entry_txt(argstr2,CamsetWidget[idx]);
          }
        else if (!strcmp(argstr1, "mdx:")) { // Need this to skip menus
            sscanf(line, "%s %s", argstr1, argstr2);
            mdx=atoi(argstr2); // Get the number of menu items
          }
        else if (!strcmp(argstr1, "PCustSet")) {  // Not used
          }
        else if (!strcmp(argstr1, "windex_sz")) {
            // windex_sz <width> <x> <height> <at> <framerate> <fps>
            sscanf(line, "%s %d %*s %d", argstr1,&Selected_Wd,&Selected_Ht);
            if(set_combo_sz_index(Selected_Wd,Selected_Ht)){
                returnvalue = PCHK_E_SYNTAX;
                sprintf(errmsg, "%s: Image dimensions %d x %d (WxH) not supported.", argstr1,Selected_Wd,Selected_Ht);
                break;
            }
          }
        else if (!strcmp(argstr1, "windex_fps")) {
            // windex_fps <fps>
            sscanf(line, "%s %s", argstr1,argstr2);
            inum1=atoi(argstr2); // Get the fps and set the combo GUI
            gtk_combo_box_set_active(GTK_COMBO_BOX(combo_fps), fps_index(inum1));
          }
        else if (!strcmp(argstr1, "windex_camfmt")) {
            // windex_camfmt <string1> [<string2>]
            // For the OptArc cameras the format name will not be more
            // than 2 strings long so argcount won't be >3.
            // Construct the full name into argstr5 according to how
            // many strings make it up:
            switch(argcount) {
              case 2:
                sscanf(line, "%s %s", argstr1, argstr5);
              break;
              case 3:
                sscanf(line, "%s %s %s", argstr1, argstr2, argstr3);
                sprintf(argstr5,"%s %s",argstr2, argstr3);
              break;
              default:
              break;
            }
            // Set the selection combo accordingly
            gtk_combo_box_set_active(GTK_COMBO_BOX(combo_camfmt), camfmt_from_string(argstr5));
          }
        else if (!strcmp(argstr1, "windex_safmt")) {
            // windex_safmt <string1> [<string2>]
            // The 'save as' format name will not be more
            // than 2 strings long so argcount won't be >3.
            // Construct the full name into argstr5 according to how
            // many strings make it up:
            switch(argcount) {
              case 2:
                sscanf(line, "%s %s", argstr1, argstr5);
              break;
              case 3:
                sscanf(line, "%s %s %s", argstr1, argstr2, argstr3);
                sprintf(argstr5,"%s %s",argstr2, argstr3);
              break;
              default:
              break; // pcs_argc_check ensures we not get here.
            }
            // Set the selection combo accordingly
            gtk_combo_box_set_active(GTK_COMBO_BOX(combo_safmt), saveas_from_string(argstr5));
          }
        else if (!strcmp(argstr1, "windex_imroot")) {
            // windex_imroot <fname>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_imroot]);
          }
        else if (!strcmp(argstr1, "windex_fno")) {
            // windex_fno <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_fno]);
          }
        else if (!strcmp(argstr1, "windex_avd")) {
            // windex_avd <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_avd]);
          }
        else if (!strcmp(argstr1, "windex_to")) {
            // windex_to <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_to]);
          }
        else if (!strcmp(argstr1, "windex_rt")) {
            // windex_rt <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_rt]);
          }
        else if (!strcmp(argstr1, "windex_srn")) {
            // windex_srn <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_srn]);
          }
        else if (!strcmp(argstr1, "windex_srd")) {
            // windex_srd <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_srd]);
          }
        else if (!strcmp(argstr1, "windex_jpg")) {
            // windex_jpg <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_jpg]);
          }
        else if (!strcmp(argstr1, "windex_gn")) {
            // windex_gn <FLOAT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_gn]);
          }
        else if (!strcmp(argstr1, "windex_bs")) {
            // windex_bs <FLOAT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_bs]);
          }
        else if (!strcmp(argstr1, "windex_del")) {
            // windex_del <INT>
            sscanf(line, "%s %s", argstr1,argstr2);
            put_entry_txt(argstr2,CamsetWidget[windex_del]);
          }
        else if (!strcmp(argstr1, "windex_pc")) {
            // windex_pc <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_preview_central), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_preview_central), FALSE);
          }
        else if (!strcmp(argstr1, "windex_yo")) {
            // windex_yo <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_cam_yonly), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_cam_yonly), FALSE);
          }
        else if (!strcmp(argstr1, "windex_sad")) {
            // windex_sad <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_rawdoubles), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_rawdoubles), FALSE);
          }
        else if (!strcmp(argstr1, "windex_fit")) {
            // windex_fit <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_fits), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_fits), FALSE);
          }
        else if (!strcmp(argstr1, "windex_smf")) {
            // windex_smf <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_scale_means), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_scale_means), FALSE);
          }
        else if (!strcmp(argstr1, "windex_ud")) {
            // windex_ud <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usedfcor), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usedfcor), FALSE);
          }
        else if (!strcmp(argstr1, "windex_uf")) {
            // windex_uf <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_useffcor), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_useffcor), FALSE);
          }
        else if (!strcmp(argstr1, "windex_um")) {
            // windex_um <Yes/No>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"Yes"))
             gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usemskcor), TRUE);
             else gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usemskcor), FALSE);
          }
        else if (!strcmp(argstr1, "windex_rdfi")) {
            // windex_rdfi <fname>
            sscanf(line, "%s %s", argstr1,argstr2);
           if(!strcmp(argstr2,"[None]")){
              if(dffile_loaded != DFIMG_NONE) nullify_darkfield();
             } else if(test_selected_df_filename(argstr2)){
             esdx++;
             dfoff++;
            }
          }
        else if (!strcmp(argstr1, "windex_rffi")) {
            // windex_rffi <fname>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(!strcmp(argstr2,"[None]")) {
                if(fffile_loaded != FFIMG_NONE) nullify_flatfield();
              } else if(test_selected_ff_filename(argstr2)){
             esdx++;
             ffoff++;
            }
          }
        else if (!strcmp(argstr1, "windex_rmski")) {
            // windex_rmski <fname>
            sscanf(line, "%s %s", argstr1,argstr2);
            if(test_selected_msk_filename(argstr2)){
             esdx++;
             mskoff++;
            }
          }              
        else if (!strcmp(argstr1, "exit")) {
            switch(esdx){
             case 0:
                sprintf(errmsg, "No problems setting file names.");
             break;
             case 1:
                sprintf(errmsg, "One file name could not be set.");
             break;
             case 2:
                sprintf(errmsg, "Two file names could not be set.");
             break;
             case 3:
                sprintf(errmsg, "Three file names could not be set.");
             break;
             default:
                sprintf(errmsg, "A programming error occured.");
             break;
            }
            returnvalue = PCHK_ALL_GOOD; break;
        } 


    } // End of master 'While' loop.

  // Switch off corrective file usage flags if there was a problem
  // verifying the chosen corrective file
  if(dfoff) gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usedfcor), FALSE);
  if(ffoff) gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_useffcor), FALSE);
  if(mskoff) gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usemskcor), FALSE);


    return returnvalue;
}

int append_cs_file(const char *fname)
// Appends the current list of settings values to disk as a plain text
// file using fname as the name of the file. This retrieves and stores
// those settings values that are currently applied and in operation.
// It does NOT save the settings that a user may have just altered prior
// to them being implemented (by clicking the 'Apply' button on the
// settings window). Also it does not save the 'Settings Files' file
// names.
// Returns 0 on success and 1 on error
{
 FILE *fp;
 time_t ct;
 
 // Attempt to open the file in append mode
 fp=fopen(fname,"ab");
 if(fp==NULL){ show_message("Failed to open file for writing camera settings.","File Save FAILED: ",MT_ERR,1); return 1;}
 
 // Write a header to identify this file format
 fprintf(fp,"\n\n\nPCustSet 1\n\n");
 
 // Now print the custom settings with their current values
 fprintf(fp,"# Image size and FPS for full frame capture\n");
 fprintf(fp,"windex_sz %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_sz])));

 fprintf(fp,"# Frames per second for live preview\n");
 fprintf(fp,"windex_fps %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_fps])));

 fprintf(fp,"# Format stream from the camera\n");
 fprintf(fp,"windex_camfmt %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_camfmt])));

 fprintf(fp,"# Format to save image files to disc as\n");
 fprintf(fp,"windex_safmt %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_safmt])));

 fprintf(fp,"# File name root for saved images\n");
 fprintf(fp,"windex_imroot %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_imroot+1])));

 fprintf(fp,"# File name frame number to start from\n");
 fprintf(fp,"windex_fno %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_fno+1])));

 fprintf(fp,"# Frame averaging (number of frames)\n");
 fprintf(fp,"windex_avd %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_avd+1])));

 fprintf(fp,"# Grabber timeout (number of seconds)\n");
 fprintf(fp,"windex_to %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_to+1])));

 fprintf(fp,"# Frame catpure (number of retries)\n");
 fprintf(fp,"windex_rt %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_rt+1])));

 fprintf(fp,"# Series (number of images)\n");
 fprintf(fp,"windex_srn %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_srn+1])));

 fprintf(fp,"# Min. interval for series (s)\n");
 fprintf(fp,"windex_srd %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_srd+1])));

 fprintf(fp,"# JPEG save quality\n");
 fprintf(fp,"windex_jpg %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_jpg+1])));

 fprintf(fp,"# YUYV conversion gain\n");
 fprintf(fp,"windex_gn %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_gn+1])));

 fprintf(fp,"# YUYV conversion bias\n");
 fprintf(fp,"windex_bs %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_bs+1])));

 fprintf(fp,"# Delay first capture by (s)\n");
 fprintf(fp,"windex_del %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_del+1])));

 fprintf(fp,"# Use crop from full-size image as preview?\n");
 fprintf(fp,"windex_pc %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_pc])));

 fprintf(fp,"# Preview in monochrome?\n");
 fprintf(fp,"windex_yo %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_yo])));

 fprintf(fp,"# Save as raw doubles?\n");
 fprintf(fp,"windex_sad %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_sad])));

 fprintf(fp,"# Save as FITS?\n");
 fprintf(fp,"windex_fit %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_fit])));

 fprintf(fp,"# Scale mean of each frame to first?\n");
 fprintf(fp,"windex_smf %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_smf])));

 fprintf(fp,"# Dark field subtraction image\n");
 fprintf(fp,"windex_rdfi %s\n\n",(dffile_loaded==DFIMG_NONE)?"[None]":DFFile);
         
 fprintf(fp,"# Apply dark field subtraction?\n");
 fprintf(fp,"windex_ud %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_ud])));

 fprintf(fp,"# Flat field subtraction image\n");
 fprintf(fp,"windex_rffi %s\n\n",(fffile_loaded==FFIMG_NONE || fffile_loaded==FFIMG_NORM)?"[None]":FFFile);

 fprintf(fp,"# Apply flat field division?\n");
 fprintf(fp,"windex_uf %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_uf])));

 fprintf(fp,"# Corrections mask image\n");
 switch(mskfile_loaded){
   case MASK_NONE:
    fprintf(fp,"windex_rmski [None]\n\n");
   break;
   case MASK_YRGB:
    fprintf(fp,"windex_rmski %s\n\n",MaskFile);
   break;
   case MASK_FULL:
    fprintf(fp,"windex_rmski [Full]\n\n");
   break;
   default: // This should not happen
    fprintf(fp,"windex_rmski [UNDF]\n\n");
   break;
 }

 fprintf(fp,"# Use corrections mask?\n");
 fprintf(fp,"windex_um %s\n\n",gtk_label_get_text(GTK_LABEL(CamsetWidget[windex_um])));

 fprintf(fp,"exit\n"); // End of settings
 
 fprintf(fp, "Saved at: %s\n\n", ((time(&ct)) == -1) ? "[Time not available]" : ctime(&ct));


 fflush(fp); fclose(fp); 
 return 0;
}

int camfmt_from_string(char *fmt)
// Get the ID index of the currently selected camera stream format.
// Return -1 on failure.
{
 int idx;
 
 for(idx=0;idx<Ncamfs;idx++)
  if(!strcmp(fmt,camfmt_options[idx])) return idx;
 
 return -1;
}

int saveas_from_string(char *fmt)
// Get the ID index of the currently selected save-as format.
//Return -1 on failure.
{
 int idx;
 
 for(idx=0;idx<Nsafs;idx++)
  if(!strcmp(fmt,safmt_options[idx])) return idx;
 
 return -1;
}

void rgb_to_int(void)
// Converts RGB data in the Frm stores to intensity data using the 'I'
// part of an HSI transform i.e. I=(R+G+B)/3.
// The ourput will always go into Frmr.
{
 size_t pos, lsz;
 double dr,dg,db;
 
 lsz=(size_t)ImHeight*(size_t)ImWidth;
 
 for(pos=0;pos<lsz;pos++){
     dr=Frmr[pos];
     dg=Frmg[pos];
     db=Frmb[pos];
     Frmr[pos]=(dr+dg+db)/3.0;
    }
  
 return;
}

int check_extn(char *fname, char *extn, size_t elen, char *emsgdhr)
// Examine the extension of a given file name strin (fname).
// elen is the mandatory length of the correct extension (you must
// ensure this is >=1 or the program may crash - or worse).
// extn is the actual extension string you want fname to have.
// emsghdr is the title to be applied to error messages that are
// displayed should fname fail to have any of the desired extension
// properties.
// Returns 0 if fname has an extension of the specified type and length.
// Returns 1 if fname does not have the correct extenstion.
{
 char *cptr;
 size_t len;
 char emsg[320];
 
 len=strlen(fname);
 elen++;
 cptr=strrchr(fname,'.');
    if(cptr==NULL){
      show_message("The file name lacks an extension.",emsgdhr,MT_ERR,1); 
      return 1;
    } else cptr++;
    if(fname[len-elen]!='.'){
      sprintf(emsg,"The file extension must be %zu characters long",elen-1);
     show_message(emsg,emsgdhr,MT_ERR,1); 
      return 1;
    }
    if(strcmp(cptr,extn)){
      sprintf(emsg,"The file extension must be %s",extn);
      show_message(emsg,emsgdhr,MT_ERR,1); 
      return 1;
    }  
 return 0;
}

char *name_from_path(char *fullpathname)
// A program to point to the beginning of a file name that is at the end
// of a full-path and file name specifier string. On *nix systems the
// directories are separated by forward slashes '/' but MS Windows uses
// back slashes. Some input strings may not have either type of slash
// in which case the return value points to the start of the string.
{
 char *cptr;
 
 cptr=(strrchr(fullpathname, '\\'));
 if(cptr!=NULL) cptr++;
 else { cptr=(strrchr(fullpathname, '/'));
        if(cptr!=NULL) cptr++;
        else cptr=fullpathname;
      }
 return cptr;
}

int get_pgm_header(char *fname,  int *ht,  int *wd)
// Read header information of a supposed pgm image.
// Returns 0 in success and assigns *ht and *wd.
// Returns 1 on failure due to inability to open file
// Returns 2 on failure due to not recognised as p5 format
{
 FILE *fpo;
 int dummy;
 char headstr[512];

 if( (fpo=fopen(fname,"rb"))==NULL ) return 1;
 if(strncmp(fgets(headstr,4,fpo),"P5",2)){fclose(fpo);return 2;}
 while(fgetc(fpo)=='#')fgets(headstr,512,fpo);
 fseek(fpo,(long)(-1),SEEK_CUR);
 fscanf(fpo,"%d %d %d\n",wd,ht,&dummy);
 fclose(fpo);
 return 0;
}

int get_ppm_header(char *fname, int *ht, int *wd)
// Read header information of a supposed ppm image.
// Returns 0 in success and assigns *ht and *wd.
// Returns 1 on failure due to inability to open file
// Returns 2 on failure due to not recognised as p5 format
{
 FILE *fpo;
 int dummy;
 char headstr[512];

 if( (fpo=fopen(fname,"rb"))==NULL ) return 1;
 if(strncmp(fgets(headstr,4,fpo),"P6",2)){fclose(fpo);return 2;}
 while(fgetc(fpo)=='#')fgets(headstr,512,fpo);
 fseek(fpo,-2L,SEEK_CUR);
 fscanf(fpo,"%d%d%d\n",wd,ht,&dummy);
 fclose(fpo);
 return 0;
}

int get_bmp_header(char *fname, int *ht, int *wd, int16_t *bitcount)
// Read the dimentions and bit depth of a BMP formatted image from disk.
// Return 0 on success and assigne *ht, *wd and *bitcount (8 or 24)
// Returns 1 on failure due to inability to open file
// Returns 2 on failure due to not recognised as bmp format
{
    FILE *fp;
    BMPHead ImgHead;

    if ((fp = fopen(fname, "rb")) == NULL) return 1;
    fread(&(ImgHead.Type), sizeof(int16_t), 1, fp);
    // Is is a BMP?
    if (ImgHead.Type != 19778) { fclose(fp); return 2; }
    fread(&(ImgHead.FSize), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Res1),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Res2),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Offs),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.IHdSize), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Width),   sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Height),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Planes),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Bitcount), sizeof(int16_t), 1, fp);
    fread(&(ImgHead.Compresn), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ImgSize),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Xpixelsm), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Ypixelsm), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ClrsUsed), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ClImport), sizeof(uint32_t), 1, fp);
    fclose(fp);


    *wd = (int)(ImgHead.Width);
    *ht = (int)(ImgHead.Height);
    *bitcount = ImgHead.Bitcount;

    return 0;
}

int get_pgm(char *fname, unsigned char **cptr,  int *ht,  int *wd)
// Read a PGM p5 formatted image from disk.
{
 FILE *fpo;
 int dummy;
 char headstr[512];

 if( (fpo=fopen(fname,"rb"))==NULL ) return 1;
 if(strncmp(fgets(headstr,4,fpo),"P5",2)){show_message("Header not  recognised as P5.","Error: ",MT_ERR,0);fclose(fpo);return 1;}
 while(fgetc(fpo)=='#')fgets(headstr,512,fpo);
 fseek(fpo,(long)(-1),SEEK_CUR);
 fscanf(fpo,"%d %d %d\n",wd,ht,&dummy);
 fread((*cptr),sizeof(unsigned char),((*ht)*(*wd)),fpo);
 fclose(fpo);
 return 0;
}

int raw_to_pgm(char *fname, int ht, int wd,void *data)
// Write a PGM p5 formatted image to disk.
{
 FILE *fp;
 
 fp=fopen(fname,"wb");
 if(fp==NULL){ show_message("Failed to open file for writing PGM image.","File Save FAILED: ",MT_ERR,1); return 1;}
 fprintf(fp,"P5\n# pgm, binary, 8bpp\n%u %u\n255\n",(unsigned int)wd,(unsigned int)ht);
 fwrite(data,sizeof(unsigned char),(size_t)(ht*wd), fp);  fflush(fp);  fclose(fp);
 return 0;
}

int get_ppm(char *fname, unsigned char **cptrRGB)
// Read a PPM p6 formatted image from disk.
{
 FILE *fpo;
 unsigned int idx,pos,dummy,ht,wd,ww3;
 char headstr[512];

 if( (fpo=fopen(fname,"rb"))==NULL ) return 1;
 if(strncmp(fgets(headstr,4,fpo),"P6",2)){show_message("Header not  recognised as P6.","Error: ",MT_ERR,0);fclose(fpo);return 1;}
 while(fgetc(fpo)=='#')fgets(headstr,512,fpo);
 fseek(fpo,-2L,SEEK_CUR);
 fscanf(fpo,"%u%u%u\n",&wd,&ht,&dummy);
 ww3=3*wd;
 for(idx=pos=0;idx<ht;idx++,pos+=ww3){
    fread((*cptrRGB)+pos,sizeof(unsigned char),(size_t)ww3,fpo);
   }

 fclose(fpo);
 return 0;
}

int raw_to_ppm(char *fname, int ht, int wd,void *data)
// Write a PPM p6 formatted image to disk.
{
 int y,pos,y_stride;
 FILE *fp;

 fp=fopen(fname,"wb");
 if(fp==NULL){ show_message("Failed to open file for writing raw RGB image.","File Save FAILED: ",MT_ERR,1); return 1;}
 fprintf(fp,"P6\n# ppmh.ppm (options ) binary encoded 24bpp r,g,b\n%u %u\n255\n",(unsigned int)wd,(unsigned int)ht);
 y_stride=wd*3;
 for(y=pos=0;y<ht;y++,pos+=y_stride) fwrite((void *)((unsigned char *)data+pos),sizeof(unsigned char),(size_t)y_stride,fp);
 fflush(fp);  fclose(fp);
 return 0;
}

int get_bmp(char *fname, unsigned char **cptrgb, int *ht, int *wd, void *cref)
// Read a BMP formatted image from disk.
{
 FILE *fp;
 unsigned int pos,ww,ww3,waw,wspill,row,offdiff,size;
 unsigned char *pad,*diffpad,rpix,bpix;
 BMPHead ImgHead;

    if ((fp = fopen(fname, "rb")) == NULL) return 1;
    fread(&(ImgHead.Type),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.FSize), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Res1),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Res2),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Offs),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.IHdSize), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Width),   sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Height),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Planes),  sizeof(int16_t) , 1, fp);
    fread(&(ImgHead.Bitcount), sizeof(int16_t), 1, fp);
    fread(&(ImgHead.Compresn), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ImgSize),  sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Xpixelsm), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.Ypixelsm), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ClrsUsed), sizeof(uint32_t), 1, fp);
    fread(&(ImgHead.ClImport), sizeof(uint32_t), 1, fp);
  *wd=(int)(ImgHead.Width);
  *ht=(int)(ImgHead.Height);
 
 if((unsigned int)(ImgHead.Bitcount)==24) offdiff=ImgHead.Offs-54;
 else if((unsigned int)(ImgHead.Bitcount)==8){
   fread((unsigned char *)cref,sizeof(unsigned char),1024,fp);
   offdiff=ImgHead.Offs-1078;
  } else { fclose(fp); return 2;}
 if((unsigned int)(ImgHead.Compresn)!=0){ fclose(fp); return 2;} 
 // The above should be (ImgHead.Compresn)!=BI_RGB but I don't want to
 // include Windows-specific headers here.

 if(offdiff){
    diffpad=(unsigned char *)calloc((size_t)offdiff,sizeof(unsigned char));
    if(diffpad==NULL){fclose(fp); return 1;}
    fread(diffpad,sizeof(unsigned char),(size_t)offdiff,fp);
    free(diffpad);
   }
 size=(unsigned int)((*ht)*(*wd));
 ww=(unsigned int)(*wd);
 ww3=ww*3;

 if((unsigned int)(ImgHead.Bitcount)==24) ww=ww3;

 wspill=ww/WORDSZ; waw=WORDSZ*wspill;
 if(waw<ww){
            wspill=WORDSZ-(ww-waw);
            pad=(unsigned char *)calloc((size_t)wspill,sizeof(unsigned char));
            if(pad==NULL){fclose(fp); return 1;}
           } else wspill=0;

if(ImgHead.Bitcount==24){
 pos=3*size-ww3;
 for(row=0;row<(*ht);row++,pos-=ww3){
    fread((*cptrgb+pos),sizeof(unsigned char),(size_t)ww3,fp);
    if(wspill) fread(pad,sizeof(unsigned char),(size_t)wspill,fp);
   }

} else if(ImgHead.Bitcount==8){
 pos=size-ww;
 for(row=0;row<(*ht);row++,pos-=ww){
    fread((*cptrgb+pos),sizeof(unsigned char),(size_t)ww,fp);
    if(wspill) fread(pad,sizeof(unsigned char),(size_t)wspill,fp);
   }
}
 fclose(fp);
 if(wspill) free(pad); 
 
 // If colour, reverse R and B because BMP is BGR
 if(ImgHead.Bitcount==24){
   for(row=0;row<RGBsize;row+=3){
       bpix=*(*cptrgb+row);
       rpix=*(*cptrgb+row+2);
       *(*cptrgb+row)=rpix;
       *(*cptrgb+row+2)=bpix;
     }
  } 
 
        
 return 0;
}

int raw_to_bmp(unsigned int iht,unsigned int iwd, unsigned char **ptr, char *fname,int format) 
// Write a BMP formatted image to disk.
{
 FILE *fpo;
 long int lsz;
 unsigned int x,y,pos,wspill,waw,wawsize,idx,ww3,nonwaw;
 unsigned char *pad,tmpbuff[3],cref[1024],cn;
 BMPHead ImgHead;
 char imsg[320];
 
#define BM8 1 // 8bpp greyscale BMP file
#define BMP 2 // 24bpp colour BMP file 

#define WORDSZ    4

 if( (fpo=fopen(fname,"wb"))==NULL ){
    sprintf(imsg,"raw_to_bmp: Cannot write to file %s.",fname);
    show_message(imsg,"Error: ",MT_ERR,0);
    return 1;
   }
 pad=NULL;
 switch(format){
    case BM8:
          wspill=iwd/WORDSZ; waw=WORDSZ*wspill;
          if(waw<iwd){
              wspill=WORDSZ-(iwd-waw);
              waw+=WORDSZ;
              pad=(unsigned char *)calloc((size_t)wspill,sizeof(unsigned char));
              if(pad==NULL){fclose(fpo); return 1;}
             } else {wspill=0; waw=iwd;}
          wawsize=waw*iht;
          for(idx=0,cn=0;idx<1024;idx+=4,cn++){
               cref[idx]=cref[idx+1]=cref[idx+2]=cn; cref[idx+3]=0;    
             }
          ImgHead.Type=19778;
          ImgHead.Res1=0;
          ImgHead.Res2=0;
          ImgHead.Offs=(54+4*256);
          ImgHead.IHdSize=40;
          ImgHead.Planes=1;
          ImgHead.Bitcount=8;
          ImgHead.Compresn=0;  // BI_RGB is the proper value to use but 
          ImgHead.Xpixelsm=0;  // I dont want to include MS Windows 
          ImgHead.Ypixelsm=0;  // specific header files here.
          ImgHead.ClrsUsed=0;
          ImgHead.ClImport=0;
        ImgHead.Width =   (uint32_t)iwd;
        ImgHead.Height =  (uint32_t)iht;
        ImgHead.ImgSize = (uint32_t)wawsize;
        fwrite(&(ImgHead.Type),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.FSize), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Res1),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Res2),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Offs),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.IHdSize), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Width),   sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Height),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Planes),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Bitcount), sizeof(int16_t), 1, fpo);
        fwrite(&(ImgHead.Compresn), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ImgSize),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Xpixelsm), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Ypixelsm), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ClrsUsed), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ClImport), sizeof(uint32_t), 1, fpo);
          fwrite(cref,sizeof(unsigned char),1024,fpo);
          pos=(iht-1)*iwd;
          for(y=0;y<iht;y++,pos-=iwd){
                for(x=0;x<iwd;x++){
                      tmpbuff[0]=(*(*(unsigned char **)ptr+pos+x));
                      fwrite(tmpbuff,sizeof(unsigned char),1,fpo);
                     }
                 if(wspill) fwrite(pad,sizeof(unsigned char),(size_t)wspill,fpo);
            }
        if(wspill) free(pad);        
     break;
    case BMP:
          ww3=iwd*3;
          wspill=ww3/WORDSZ; waw=WORDSZ*wspill;
          if(waw<ww3){
              wspill=WORDSZ-(ww3-waw);
              waw+=WORDSZ;
              pad=(unsigned char *)calloc((size_t)wspill,sizeof(unsigned char));
              if(pad==NULL){fclose(fpo); return 1;}
           } else {wspill=0; waw=ww3;}

        lsz=iht*ww3;
        wawsize=waw*iht;
        nonwaw=ww3;

          ImgHead.Type=19778;
          ImgHead.Res1=0;
          ImgHead.Res2=0;
          ImgHead.Offs=54;
          ImgHead.IHdSize=40;
          ImgHead.Planes=1;
          ImgHead.Bitcount=24;
          ImgHead.Compresn=0;
          ImgHead.Xpixelsm=0;
          ImgHead.Ypixelsm=0;
          ImgHead.ClrsUsed=0;
          ImgHead.ClImport=0;
        ImgHead.Width = (uint32_t)iwd;
        ImgHead.Height = (uint32_t)iht;
        ImgHead.ImgSize = (uint32_t)wawsize;
        fwrite(&(ImgHead.Type),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.FSize), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Res1),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Res2),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Offs),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.IHdSize), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Width),   sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Height),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Planes),  sizeof(int16_t) , 1, fpo);
        fwrite(&(ImgHead.Bitcount), sizeof(int16_t), 1, fpo);
        fwrite(&(ImgHead.Compresn), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ImgSize),  sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Xpixelsm), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.Ypixelsm), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ClrsUsed), sizeof(uint32_t), 1, fpo);
        fwrite(&(ImgHead.ClImport), sizeof(uint32_t), 1, fpo);
        pos=lsz-nonwaw;
      for(y=0;y<iht;y++,pos-=nonwaw){
          
           fwrite((*ptr+pos),sizeof(unsigned char),(size_t)nonwaw,fpo);
           if(wspill) fwrite(pad,sizeof(unsigned char),(size_t)wspill,fpo);
        }


        if(wspill) free(pad);        
     break;
    default:
     fclose(fpo);
     show_message("raw_to_bmp: Pixel format not supported by this function.","Error ",MT_ERR,0);
     return 1;
 }

  fclose(fpo);
  return 0;
 }

int write_png_image(char* filename, int width, int height, unsigned char *img, char* title)
// Write a PNG formatted image to disk.
{
   int code = 0;
   FILE *fp = NULL;
   png_structp png_ptr = NULL;
   png_infop info_ptr = NULL;
   png_bytep row = NULL;
   char imsg[320];

   // Open file for writing (binary mode)
   fp = fopen(filename, "wb");
   if (fp == NULL) {
      sprintf(imsg, "Could not open file %s for writing.", filename);
      show_message(imsg,"PNG Error: ",MT_ERR,0);
      code = 1;
      goto finalise;
   }
   
   // Initialize write structure
   png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
   if (png_ptr == NULL) {
      show_message("Could not allocate png write struct.","PNG Error: ",MT_ERR,0);
      code = 1;
      goto finalise;
   }

   // Initialize info structure
   info_ptr = png_create_info_struct(png_ptr);
   if (info_ptr == NULL) {
      show_message("Could not allocate info struct.","PNG Error: ",MT_ERR,0);
      code = 1;
      goto finalise;
   }
   
   // Setup Exception handling
   if (setjmp(png_jmpbuf(png_ptr))) {
      show_message("Error during png creation.","PNG Error: ",MT_ERR,0);
      code = 1;
      goto finalise;
   }
   
   png_init_io(png_ptr, fp);

   // Write header (8 bit colour depth)
   png_set_IHDR(png_ptr, info_ptr, width, height,
         8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
         PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

   // Set title
   if (title != NULL) {
      png_text title_text;
      title_text.compression = PNG_TEXT_COMPRESSION_NONE;
      title_text.key = "Title";
      title_text.text = title;
      png_set_text(png_ptr, info_ptr, &title_text, 1);
   }

   png_write_info(png_ptr, info_ptr);
   
   // Allocate memory for one row (3 bytes per pixel - RGB)
   row = (png_bytep) malloc(3 * width * sizeof(png_byte));
   if (row == NULL) {
      show_message("Could not allocate row.","PNG Error: ",MT_ERR,0);
      code = 1;
      goto finalise;
   }

   // Write image data
   int r,c,sz,ipos,w3;
   sz=height*width;
   w3=width*3;
   ipos=0;
   for (r=0 ; r<sz ; r+=width) {
      for (c=0 ; c<w3 ; c++) {
         row[c]= img[ipos++];
      }
      png_write_row(png_ptr, row);
   }

   // End write
   png_write_end(png_ptr, NULL);
   
   
   finalise:
   if (fp != NULL) fclose(fp);
   if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
   if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
   if (row != NULL) free(row);

   return code;
}

int raw_to_jpeg(int ht, int wd, unsigned char **cptrgb, char *fname, int quality)
// Save a raw rgb image as a JPEG file
{
 FILE* fpo;
 struct jpeg_compress_struct info;
 struct jpeg_error_mgr err;
 unsigned char* lpRowBuffer[1];
 int w3;
 char imsg[320];
 
 fpo = fopen(fname, "wb");
 if(fpo == NULL) {
    sprintf(imsg,"raw_to_jpeg: Cannot write to file %s.",fname);
    show_message(imsg,"Error: ",MT_ERR,0);
    return 1;
  }

 info.err = jpeg_std_error(&err);
 jpeg_create_compress(&info);

 jpeg_stdio_dest(&info, fpo);

 info.image_width = (JDIMENSION)wd;
 info.image_height = (JDIMENSION)ht;
 info.input_components = 3;
 info.in_color_space = JCS_RGB;

 jpeg_set_defaults(&info);
 jpeg_set_quality(&info, quality, TRUE);
 jpeg_start_compress(&info, TRUE);

 w3=wd*3;
 while(info.next_scanline < info.image_height) {
     lpRowBuffer[0] = (*cptrgb+(info.next_scanline * w3));
     jpeg_write_scanlines(&info, lpRowBuffer, 1);
    }

 jpeg_finish_compress(&info);
 fclose(fpo);

 jpeg_destroy_compress(&info);
 return 0;
}

int write_fits_cardimg(FILE *fp,char *fci)
// Output a FITS header 'card image' to file pointed to by fp.
// return 1 on error, 0 on success
{
 size_t len,nobj,idx;
 len=strlen(fci); for(idx=len;idx<80;idx++) fci[idx]=32;
 nobj=fwrite(fci,sizeof(char),80,fp);
 if(nobj!=80){
   show_message("Error writing FITS header - cannot write FITS file.","FITS Header: ",MT_ERR,1); 
   return 1;
  }
 return 0;
}

int double_byteswap(size_t img_sz, void **img)
// Performs byte-swapping for doubles.
// Returns 0 on success, 1 on error.
{
 unsigned char *ctmp;
 size_t bpn,idx,ndx;

 bpn=sizeof(double);

 ctmp=(unsigned char *)calloc(bpn,sizeof(unsigned char));
 if(ctmp==NULL) return 1;
 img_sz*=bpn; 
 for(idx=0;idx<img_sz;idx+=bpn){
    for(ndx=0;ndx<bpn;ndx++) ctmp[ndx]= *(*(unsigned char **)img+idx+ndx);
    for(ndx=0;ndx<bpn;ndx++) *(*(unsigned char **)img+idx+bpn-ndx-1)=ctmp[ndx];
   }
 free(ctmp);
 return 0;
}

int write_fits(char *fname, int colchan, int is_avg)
// Write the image raw doubles data in the img buffer as a FITS file.
// This may be useful for those who want to export the saved frames to
// another program that may not be able to read my raw doubles foramt.
// colchan is the colour channel to write out and must be one of
// CCHAN_Y, CCHAN_R, CCHAN_G, CCHAN_B
// is_avg must be 0 or 1 (1 means this is a multi-frame average). This
// information is used to add information to the comments section of the
// FITS file for good record keeping.
// Return 1 on error, 0 on success.
{
 FILE *fpo;
 char fcardimg[81];
 char emsgdata[256];
 uint8_t padbyte;
 size_t len,nobj,idx,edx,bpp,bpr,pad,nrecords,prec,epad;
 struct tm *dtinfo;
 time_t tnow;
 double *framepos,*img;
 
 // Attempt to open the file for writing
 if( (fpo=fopen(fname,"wb"))==NULL ){
   show_message("Could not open output FITS file for writing.","FITS Output: ",MT_ERR,1); 
   goto error_return_1;
  } 

 // Construct and write the FITS header 'card images'
 sprintf(fcardimg,"SIMPLE  =                    T / file does conform to FITS standard");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;
 bpp=8*sizeof(double);
 sprintf(fcardimg,"BITPIX  =                  -%zu / number of bits per data pixel", bpp);
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;
 sprintf(fcardimg,"NAXIS   =                    2 / number of data axes");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 sprintf(emsgdata,"%d",ImWidth);
 len=strlen(emsgdata);
 sprintf(fcardimg,"NAXIS1  = ");
 for(idx=10;idx<(30-len);idx++) fcardimg[idx]=32;
 for(edx=0;idx<=30;idx++,edx++) fcardimg[idx]=emsgdata[edx];
 fcardimg[idx]='\0';
 strcat(fcardimg," / length of data axis 1, width");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 sprintf(emsgdata,"%d",ImHeight);
 len=strlen(emsgdata);
 sprintf(fcardimg,"NAXIS2  = ");
 for(idx=10;idx<(30-len);idx++) fcardimg[idx]=32;
 for(edx=0;idx<=30;idx++,edx++) fcardimg[idx]=emsgdata[edx];
 fcardimg[idx]='\0';
 strcat(fcardimg," / length of data axis 2, height");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // Set the colour channel info as a comment in the header and select
 // the appropriate data to use:
 switch(colchan)
 {
  case CCHAN_Y:
    if(saveas_fmt == SAF_INT) sprintf(fcardimg,"COMMENT   Image data is the Intensity channel");
    else sprintf(fcardimg,"COMMENT   Image data is the Y channel");
    img=Frmr;
  break;
  case CCHAN_R:
    sprintf(fcardimg,"COMMENT   Image data is the RED channel");
    img=Frmr;
  break;
  case CCHAN_G:
    sprintf(fcardimg,"COMMENT   Image data is the GRN channel");
    img=Frmg;
  break;
  case CCHAN_B:
    sprintf(fcardimg,"COMMENT   Image data is the BLU channel");
    img=Frmb;
  break;
  default: // This should not happen - programming error
       show_message("Unrecognised colour channel.","FITS Save FAILED: ",MT_ERR,1); 
       goto error_return_2;
  break;
 }
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // State if this is a single frame or multi-frame average
 if(is_avg==0) sprintf(fcardimg,"COMMENT   Image data represents a single frame capture");
 else sprintf(fcardimg,"COMMENT   Image data represents the mean average of %d frames",Av_limit);
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // State if dark field correction was done
 if(do_df_correction) sprintf(fcardimg,"COMMENT   Dark field subtraction was applied.");
 else sprintf(fcardimg,"COMMENT   Dark field subtraction was NOT done.");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // State if flat field correction was done
 if(do_ff_correction) sprintf(fcardimg,"COMMENT   Flat field division was applied.");
 else sprintf(fcardimg,"COMMENT   Flat field division was was NOT done.");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // Mask image (if any)
 switch(mskfile_loaded){
   case MASK_NONE:
     sprintf(fcardimg,"COMMENT   Mask file: [None]");
   break;
   case MASK_YRGB:
     sprintf(fcardimg,"COMMENT   Mask file: %.59s",name_from_path(MaskFile));
   break;
   case MASK_FULL:
     sprintf(fcardimg,"COMMENT   Mask file: [Full]");
   break;
   default: // This should not happen
     sprintf(fcardimg,"COMMENT   Mask file: [UNDF]");
   break;
 }
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // Dark field image (if any)
 if(dffile_loaded != DFIMG_NONE) sprintf(fcardimg,"COMMENT   Dark file: %.59s",name_from_path(DFFile));
 else sprintf(fcardimg,"COMMENT   Dark file: [None]");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // Flat field image (if any)
 if((fffile_loaded != FFIMG_NONE) && (fffile_loaded != FFIMG_NORM)) sprintf(fcardimg,"COMMENT   Flat file: %.59s",name_from_path(FFFile));
 else sprintf(fcardimg,"COMMENT   Flat file: [None]");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 sprintf(fcardimg,"COMMENT   Image written by PARD Capture pardcap.c write_fits(...) function.");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 tnow=time(NULL);
 dtinfo = gmtime(&tnow);
 nobj=strftime(emsgdata,21,"%Y-%m-%dT%H:%M:%S",dtinfo);
 if(nobj==19){
   sprintf(fcardimg,"DATE    = \'%.19s\' / file creation date (YYYY-MM-DDThh:mm:ss) UTC",emsgdata);
   if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;
  } else {
   sprintf(fcardimg,"COMMENT   Could not record the date and time of writing.");
   if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;
  }

 sprintf(fcardimg,"END");
 if(write_fits_cardimg(fpo,fcardimg)) goto error_return_2;

 // Up to this point only 1200 bytes of header are written but we must
 // make the header block upto exact multiples of 2880 bytes (because
 // the header is a 'record' and FITS records are always 2880 bytes
 // long). So now we must pad the remaining 1680 bytes with blanks:
 padbyte=32;
 for(edx=0;edx<1680;edx++){
     nobj=fwrite(&padbyte,sizeof(uint8_t),1,fpo);
     if(nobj!=1){
        sprintf(emsgdata, "Checksum error padding FITS header: pad=%zu (expected 1).",nobj);
        show_message(emsgdata,"FITS write FAILED",MT_ERR,1); 
        goto error_return_2;
       }
  }
 // Header written.

 // Write the image data in 'records' (each record = 2880 bytes long)

 bpp/=8;             // The number of bytes per pixel.
 bpr=2880/bpp;       // The numbers of pixels per record.
 pad=2880-(bpr*bpp); // The number of bytes to pad a record so it equals
                     // 2880 bytes after bpr pixels have been written.
 len=(size_t)ImHeight*(size_t)ImWidth; // The number of pixels to write.
 nrecords=(len*bpp)/2880; // The number of full records to write.
 prec=(len*bpp)-(nrecords*2880); // The number of pixel bytes left to
                                 // write after nrecords-worth of pixel
                                 // bytes have been written.
 epad=2880-prec; // The number of bytes to pad any final incomplete
                 // record.
 prec/=bpp; // The number of pixels to write on any final incomplete
            // record.

 padbyte=0;

 framepos=img; // Start at the beginning of the frame store data

 // For little-endian CPUs (like Intel) you need to swap the byte order
 // of the pixel data before writing it to FITS, so check for this:
 if (ntohl(0x12345678) == 0x78563412) { // If little-endian ...
   if(double_byteswap(len,(void **)&img)){
      show_message("Byte-swapping of image pixel data failed","FITS write FAILED",MT_ERR,1); 
      goto error_return_2;
     }
  }

 // Now the writing begins ...
 if(nrecords>0){ // If there are whole records-worth of pixels, write them:
   for(idx=0;idx<nrecords;idx++){ // For each record ...
      // First write the pixels:
      nobj=fwrite(framepos,sizeof(double),bpr,fpo);
      if(nobj<bpr){
         sprintf(emsgdata, "Checksum error writing FITS data: nobj=%zu (expected %zu).",nobj,bpr);
         show_message(emsgdata,"FITS write FAILED",MT_ERR,1); 
         goto error_return_2;
        }
      framepos+=bpr;
      // Then do any padding to ensure the record is 2880 bytes long:
      if(pad>0){
        for(edx=0;edx<pad;edx++){
           nobj=fwrite(&padbyte,sizeof(uint8_t),1,fpo);
           if(nobj!=1){
            sprintf(emsgdata, "Checksum error writing FITS data: pad=%zu (expected 1).",nobj);
            show_message(emsgdata,"FITS write FAILED",MT_ERR,1); 
            goto error_return_2;
           }
          }
        }
     }
  } // Done writing complete records
 // If there are any pixels left to write, write them:
 if(prec>0){
      nobj=fwrite(framepos,sizeof(double),prec,fpo);
      if(nobj<prec){
         sprintf(emsgdata, "Checksum error writing final FITS data: nobj=%zu (expected %zu).",nobj,prec);
         show_message(emsgdata,"FITS write FAILED",MT_ERR,1); 
         goto error_return_2;
        }
      // Then do any padding to ensure the record is 2880 bytes long:
      if(epad>0){
        for(edx=0;edx<epad;edx++){
           nobj=fwrite(&padbyte,sizeof(uint8_t),1,fpo);
           if(nobj!=1){
            sprintf(emsgdata, "Checksum error writing final FITS data: pad=%zu (expected 1).",nobj);
            show_message(emsgdata,"FITS write FAILED",MT_ERR,1); 
            goto error_return_2;
           }
          }
        }
   }

 fclose(fpo);
 return 0;

 error_return_2:
 fclose(fpo); 
 error_return_1:
 return 1;
}

int read_preview_master(char *fname, int corrtype)
// Reads a preview image master dark frame image if corrtype = 1.
// Reads a preview image master flat frame image if corrtype = 2.
// In either case the supplied file must be in raw doubles format with a
// .qih external header confirming its size as PreviewHt x PreviewWd
// - no other size is allowed.
{
 FILE *fph;
 size_t len,nobj;
 char *headername,mstype[32];
 int nmagic,fgc,lht,lwd;
 double dval;

 switch(corrtype){
   case 1: sprintf(mstype,"QIH Error (P.Dark): "); break;
   case 2: sprintf(mstype,"QIH Error (P.Flat): "); break;
  }
 
 // Allocate memory for header file name
 headername=(char *)calloc(sizeof(char),FILENAME_MAX);
 if(headername==NULL){
   show_message("Failed to allocate memory for the qih header.",mstype,MT_ERR,1); 
   return 1;
 }
 
 // Created the expected header file name
 len=strlen(fname);
 if(len>=FILENAME_MAX){
   show_message("Cannot create header (image file name is too long).",mstype,MT_ERR,1); 
   goto error_return;
 }
 sprintf(headername,"%s",fname);
  
 // Read the extension - it must be '.dou' or '.DOU'
 switch(corrtype){
   case 1: sprintf(mstype,"Error reading P.Dark: "); break;
   case 2: sprintf(mstype,"Error reading P.Flat: "); break;
   default: return 1;
  }

 if(check_extn(headername, "dou", 3, mstype)){
    if(check_extn(headername, "DOU", 3, mstype)) goto error_return;
  }


 // Now create the expected QIH file name from the raw image file name
 headername[len-3]='q';
 headername[len-2]='i';
 headername[len-1]='h';
 
 // Attempt to read the header file information and ensure it is
 // consistent:
 if( (fph=fopen(headername,"rb"))==NULL ){
     show_message("Cannot open qih file to read it.",mstype,MT_ERR,1); 
     goto error_return;
    }

 // Read the QIH file to see if it is of the correct format
 nmagic=0;
 fgc=fgetc(fph);
 if(fgc==EOF) goto format_error;
 if(fgc=='{') nmagic++;
 fgc=fgetc(fph);
 if(fgc==EOF) goto format_error;
 if(fgc=='q') nmagic++;
 fgc=fgetc(fph);
 if(fgc==EOF) goto format_error;
 if(fgc=='i') nmagic++;
 fgc=fgetc(fph);
 if(fgc==EOF) goto format_error;
 if(fgc=='h') nmagic++;
 fgc=fgetc(fph);
 if(fgc==EOF) goto format_error;
 if(fgc==':') nmagic++;
 if(nmagic!=5) goto format_error;
 // Ignore the header comments ...
 FOREVER{ 
   if(feof(fph)){
     goto format_error;
    } else if(fgetc(fph)=='}') break;
  }
 //Now read the values
 fgc=fscanf(fph,"%*s %*s %*s %*s %*s %d %*s %d",&lht,&lwd);
 fclose(fph);

 if(fgc==EOF){
   show_message("An error occured when reading the header file.",mstype,MT_ERR,1); 
   goto error_return;
  }
 // Test if the header gives the correct dimensions
 if((lht != PreviewHt) || (lwd != PreviewWd)){
   show_message("The header file does not have the correct preview dimensions.",mstype,MT_ERR,1); 
   goto error_return;
  }
 
 // Now attempt to read the data into the Preview dark or flat buffer
 if( (fph=fopen(fname,"rb"))==NULL ){
     show_message("Cannot open the file to read it.",mstype,MT_ERR,1); 
     goto error_return;
    }
 len=(size_t)PreviewHt*(size_t)PreviewWd;

 switch(corrtype){
   case 1: nobj=fread((void *)Preview_dark,sizeof(double),len,fph); break;
   case 2: nobj=fread((void *)Preview_flat,sizeof(double),len,fph); break;
  }
 
 if(nobj<len){
        show_message("Checksum error reading file.",mstype,MT_ERR,1); 
        fclose(fph); 
        goto error_return;
       }
 fclose(fph);

 switch(corrtype){
   case 1: PrevDark_Loaded=1; break;
   case 2:
   // In the case of a flat field image we must normalise it
     dval=0.0;
     for(nobj=0;nobj<len;nobj++) dval+=Preview_flat[nobj];
     dval/=(double)len; // The mean value
         if(dval<0.5){
           show_message("Preview master flat is not useable (no pixel is greater than 0).","FAILED: ",MT_ERR,1);
           goto error_return;
         }
     for(nobj=0;nobj<len;nobj++) Preview_flat[nobj]/=dval;
     PrevFlat_Loaded=1;
   break;
  } 

 free(headername);

 return 0;
 
 format_error:
   fclose(fph);
   show_message("The header file is not of the correct format.","QIH Read FAILED: ",MT_ERR,1); 
  
 error_return:
 free(headername);
 switch(corrtype){
   case 1: nullify_preview_darkfield();  break;
   case 2: nullify_preview_flatfield(); break;
  }
 return 1;
}

int write_rawdou(char *fname,int colchan)
 // Write the image doubles Frm buffer(s) as raw doubles file(s) with a
 // qih external header. 
 // colchan is the colour channel to write out and must be one of
 // CCHAN_Y, CCHAN_R, CCHAN_G, CCHAN_B
 // Return 1 on error, 0 on success.
{
 FILE *fpo,*fph;
 char *headername;
 char emsgdata[256],emsgheader[32];
 size_t len,nobj;
 
 // Allocate memory for header file name
 headername=(char *)calloc(sizeof(char),FILENAME_MAX);
 if(headername==NULL){
   show_message("Failed to allocate memory for raw doubles header.","Raw doubles Save FAILED: ",MT_ERR,1); 
   return 1;
 }

 // Set the error message title according to the colour channel
 switch(colchan)
 {
  case CCHAN_Y:
    sprintf(emsgdata, "Raw doubles Save (Y) FAILED: ");
    sprintf(emsgheader, "Raw doubles Header (Y) FAILED: ");
  break;
  case CCHAN_R:
    sprintf(emsgdata, "Raw doubles Save (R) FAILED: ");
    sprintf(emsgheader, "Raw doubles Header (R) FAILED: ");
  break;
  case CCHAN_G:
    sprintf(emsgdata, "Raw doubles Save (G) FAILED: ");
    sprintf(emsgheader, "Raw doubles Header (G) FAILED: ");
  break;
  case CCHAN_B:
    sprintf(emsgdata, "Raw doubles Save (B) FAILED: ");
    sprintf(emsgheader, "Raw doubles Header (B) FAILED: ");
  break;
  default: // This should not happen - programming error
       show_message("Unrecognised colour channel.","Raw doubles Save FAILED: ",MT_ERR,1); 
       goto error_return;
  break;
 }

 // Attempt to write the data
 if( (fpo=fopen(fname,"wb"))==NULL ){
   show_message("Could not open output file to write the raw doubles data.",emsgdata,MT_ERR,1); 
   goto error_return;
  }
 
 len=(size_t)ImHeight*(size_t)ImWidth;
 
 switch(colchan)
  {
   case CCHAN_Y:
   case CCHAN_R:
       nobj=fwrite(Frmr,sizeof(double),len,fpo);
       if(nobj<len){
         sprintf(emsgdata, "Checksum error writing raw doubles (Y/R): nobj=%zu (expected %zu).",nobj,len);
        show_message(emsgdata,"Raw write FAILED",MT_ERR,1); 
        fclose(fpo); goto error_return;
       }
   break;
   case CCHAN_G:
       nobj=fwrite(Frmg,sizeof(double),len,fpo);
       if(nobj<len){
         sprintf(emsgdata, "Checksum error writing raw doubles (G): nobj=%zu (expected %zu).",nobj,len);
        show_message(emsgdata,"Raw write FAILED",MT_ERR,1); 
        fclose(fpo); goto error_return;
       }
   break;
   case CCHAN_B:
       nobj=fwrite(Frmb,sizeof(double),len,fpo);
       if(nobj<len){
         sprintf(emsgdata, "Checksum error writing raw doubles (B): nobj=%zu (expected %zu).",nobj,len);
        show_message(emsgdata,"Raw write FAILED",MT_ERR,1); 
        fclose(fpo); goto error_return;
       }
   break;
   default: // This should not happen - we checked already
   break;
  }
 fclose(fpo);
 
 // Now write the header file
 len=strlen(fname);
 if(len>=FILENAME_MAX){
   show_message("Cannot create header (image file name is too long).",emsgheader,MT_ERR,1); 
   goto error_return;
 }
 sprintf(headername,"%s",fname);
 if(headername[len-4]!='.'){
   show_message("Cannot create header file name extension.",emsgheader,MT_ERR,1); 
   goto error_return;
  }
 headername[len-3]='q';
 headername[len-2]='i';
 headername[len-1]='h';
 if( (fph=fopen(headername,"wb"))==NULL ){
   show_message("Cannot open output file to write qih header data.",emsgheader,MT_ERR,1); 
   goto error_return;
  }
 fprintf(fph,"{qih: BiaQIm Header File }\n[Signed_?] depends\n[Datatype] depends\n[Height] %d\n[Width] %d\n",ImHeight,ImWidth);
 fclose(fph);

 free(headername);
 return 0;
 
 error_return:
 free(headername);
 return 1;
}

int read_qih_file(char *fname,int *ht, int *wd, int *coltype)
// Looks for a qih header file for a raw doubles image file given the
// input image file name (fname). It expects fname to have a 3 letter
// extension and the corresponding hiq header file to be named the same
// as the image but with the extension '.qih' in place of the original
// extension in fname.
// coltype is an int value which represent the colour type of the 
// dark field image. Although all dark field images for this program
// are raw doubles arrays, they may be only a single image representing
// a monochrome correction or a set of 3 seprate images representing
// corrections for an RGB image. The monochrome correction will have a
// '_Y' just prior to the extention and the colour set will have a '_R'
// '_G' and '_B' just prior to the extension for each image in the set
// (the remainder of the file name will be identical for each image in
// the RGB set).
// Coltype return values: DFIMG_Y, DFIMG_RGB, DFIMG_NONE
// Return 0 on success and puts the image dimensions read from the qih
// file into ht and wd and the colour type number into coltype.
// Returns 1 on failure (failure may be not finding a qih file of the
// expencted name or some error in reading it). On error the return
// values will be set to ht=0, wd=0, coltype=DFIMG_NONE
{
 FILE *fph;
 size_t len;
 char *headername,*cptr;
 int nmagic,fgc,lcoltype,lht,lwd;
 
 // Allocate memory for header file name
 headername=(char *)calloc(sizeof(char),FILENAME_MAX);
 if(headername==NULL){
   show_message("Failed to allocate memory for the qih header.","QIH Read FAILED: ",MT_ERR,1); 
   return 1;
 }
 
 // Created the expected header file name
 len=strlen(fname);
 if(len>=FILENAME_MAX){
   show_message("Cannot create header (image file name is too long).","QIH Read FAILED: ",MT_ERR,1); 
   goto error_return;
 }
 sprintf(headername,"%s",fname);
  
 // Read the extension - it must be '.dou' or '.DOU'
 if(check_extn(headername, "dou", 3, "QIH Read FAILED: ")){
    if(check_extn(headername, "DOU", 3, "QIH Read FAILED: ")) goto error_return;
  }
 cptr=strrchr(headername,'.');
 cptr-=2;
 // Check if the image name is of a recognised colour type
 lcoltype=DFIMG_NONE;
 if(!strncmp(cptr,"_Y",2) || !strncmp(cptr,"_y",2) || !strncmp(cptr,"_i",2) || !strncmp(cptr,"_I",2)) lcoltype=DFIMG_Y;
 else {
  if(!strncmp(cptr,"_R",2) || !strncmp(cptr,"_r",2)) lcoltype=DFIMG_RGB;
  if(!strncmp(cptr,"_G",2) || !strncmp(cptr,"_g",2)) lcoltype=DFIMG_RGB;
  if(!strncmp(cptr,"_B",2) || !strncmp(cptr,"_b",2)) lcoltype=DFIMG_RGB;
 }  
 if(lcoltype==DFIMG_NONE){
   show_message("Raw image file name is not of a recognised colour type.","QIH Read FAILED: ",MT_ERR,1); 
   goto error_return;
 }

 // Now create the expected QIH file name from the raw image file name
 headername[len-3]='q';
 headername[len-2]='i';
 headername[len-1]='h';
 
 // Attempt to read the header file information and ensure it is
 // consistent:
 
 if(lcoltype==DFIMG_Y){ // Only one header to check - monochrome
     
   if( (fph=fopen(headername,"rb"))==NULL ){
     show_message("Cannot open qih file to read it (Y/I).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }

   // Read the QIH file to see if it is of the correct format
   nmagic=0;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='{') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='q') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='i') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='h') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc==':') nmagic++;
   if(nmagic!=5) goto format_error;
   // Ignore the header comments ...
   FOREVER{ 
     if(feof(fph)){
       goto format_error;
      } else if(fgetc(fph)=='}') break;
    }
   //Now read the values
   fgc=fscanf(fph,"%*s %*s %*s %*s %*s %d %*s %d",ht,wd);
   fclose(fph);
   if(fgc==EOF){
     show_message("An error occured when reading the header file (Y/I).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }
  // End of checking for a Y image header.
  }  else { // Check for the triplet of colour channel headers

   // Create the expected Red channel header name
   cptr++;
   cptr[0]='R';
   // Now check it   
   if( (fph=fopen(headername,"rb"))==NULL ){
     show_message("Cannot open qih file to read it (R).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }

   // Read the QIH file to see if it is of the correct format
   nmagic=0;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='{') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='q') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='i') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='h') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc==':') nmagic++;
   if(nmagic!=5) goto format_error;
   // Ignore the header comments ...
   FOREVER{ 
     if(feof(fph)){
       goto format_error;
      } else if(fgetc(fph)=='}') break;
    }
   //Now read the values
   fgc=fscanf(fph,"%*s %*s %*s %*s %*s %d %*s %d",ht,wd);
   fclose(fph);
   if(fgc==EOF){
     show_message("An error occured when reading the header file (R).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }
   lht=*ht; // Make copies to compare to the dimensions
   lwd=*wd; // values in the other 2 files - they must be identical.
   
    // Create the expected Green channel header name
   cptr[0]='G';
   // Now check it   
   if( (fph=fopen(headername,"rb"))==NULL ){
     show_message("Cannot open qih file to read it (G).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }

   // Read the QIH file to see if it is of the correct format
   nmagic=0;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='{') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='q') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='i') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='h') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc==':') nmagic++;
   if(nmagic!=5) goto format_error;
   // Ignore the header comments ...
   FOREVER{ 
     if(feof(fph)){
       goto format_error;
      } else if(fgetc(fph)=='}') break;
    }
   //Now read the values
   fgc=fscanf(fph,"%*s %*s %*s %*s %*s %d %*s %d",ht,wd);
   fclose(fph);
   if(fgc==EOF){
     show_message("An error occured when reading the header file (G).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }
   // Check if the dimensions are the same as for the Red image header
   if(lht!=*ht || lwd!=*wd){
     show_message("Header for green channel has different dimensions to red channel.","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }
   
   
    // Create the expected Green channel header name
   cptr[0]='B';
   // Now check it   
   if( (fph=fopen(headername,"rb"))==NULL ){
     show_message("Cannot open qih file to read it (B).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }

   // Read the QIH file to see if it is of the correct format
   nmagic=0;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='{') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='q') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='i') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc=='h') nmagic++;
   fgc=fgetc(fph);
   if(fgc==EOF) goto format_error;
   if(fgc==':') nmagic++;
   if(nmagic!=5) goto format_error;
   // Ignore the header comments ...
   FOREVER{ 
     if(feof(fph)){
       goto format_error;
      } else if(fgetc(fph)=='}') break;
    }
   //Now read the values
   fgc=fscanf(fph,"%*s %*s %*s %*s %*s %d %*s %d",ht,wd);
   fclose(fph);
   if(fgc==EOF){
     show_message("An error occured when reading the header file (B).","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }
   // Check if the dimensions are the same as for the Red image header
   if(lht!=*ht || lwd!=*wd){
     show_message("Header for blue channel has different dimensions to red channel.","QIH Read FAILED: ",MT_ERR,1); 
     goto error_return;
    }

  }
 
 
 *coltype=lcoltype;
 free(headername);
 return 0;
 
 format_error:
   fclose(fph);
   show_message("The header file is not of the correct format.","QIH Read FAILED: ",MT_ERR,1); 
  
 error_return:
 free(headername);
 *ht=0, *wd=0, *coltype=DFIMG_NONE;
 return 1;
}

int read_raw_doubles(char *fname,double **rdptr,int ht, int wd, int coltype)
// Reads a raw array of doubles (or 3 separate raw arrays of doubles)
// into the memory storage pointed to by rdptr. This must be allocated
// to have enough storage to hold the array(s).
// Coltype specifies whether one or 3 arrays are to be read and must
// take one of these: DFIMG_Y (read one array), DFIMG_RGB (read 3).
// If 3 arrays are to be read they will be read into one block of RAM
// with alternating values from each array: R,G,B,R,G,B,... etc.
// Before calling this function, the function read_qih_file should be
// called to ensure approptiate header files are present. These headers
// are not read again here. It is ASSUMED that the files on disc have
// the required amount of data in them according to the dimensions
// specified in the header files. If they have less then a segmentation
// fault may occur or other crash condition. If they have more or wrong
// dimentions then the loaded data will be garbled and unreliable for
// dark field correction. It is the user's responsibility to get this
// right.
// Return 0 on success.
// Returns 1 on failure.
{
 FILE *fpr,*fpg,*fpb;
 size_t len,nobj,dfsize;
 char *rname,*gname,*bname,*cptr;
 char emsgtype[]="Raw doubles read FAILED: ";
 char emsg[256];
 int rgbpos;
 
 dfsize=(size_t)ht*(size_t)wd;
 
 // One channel or 3?
 switch(coltype){
     case DFIMG_Y:
       if( (fpr=fopen(fname,"rb"))==NULL ){
         show_message("Cannot open raw file to read it (Y/I).",emsgtype,MT_ERR,1); 
         return 1;
        }
       nobj=fread((*rdptr),sizeof(double),dfsize,fpr);
       if(ferror(fpr)){
         show_message("File read error occurred when reading raw file (Y/I).",emsgtype,MT_ERR,1); 
         fclose(fpr);
         return 1;
        }
       if(nobj!=dfsize){
         sprintf(emsg,"File read checksum error in raw file (Y/I): nobj=%zu (expected %zu)",nobj,dfsize);
         show_message(emsg,emsgtype,MT_ERR,1); 
         fclose(fpr);
         return 1;
        }
       fclose(fpr); // All seems good. 
     break;
     case DFIMG_RGB:
      // Create the expected colour channel image file names
      len=strlen(fname);
      if(len>=FILENAME_MAX){
        show_message("Image file name is too long to process.",emsgtype,MT_ERR,1); 
        return 1;
       }

      rname=(char *)calloc(sizeof(char),FILENAME_MAX);
      if(rname==NULL){
        show_message("Failed to allocate memory for the red channel file name.",emsgtype,MT_ERR,1); 
        return 1;
       }

      gname=(char *)calloc(sizeof(char),FILENAME_MAX);
      if(gname==NULL){
        show_message("Failed to allocate memory for the green channel file name.",emsgtype,MT_ERR,1); 
        free(rname);
        return 1;
       }
      
      bname=(char *)calloc(sizeof(char),FILENAME_MAX);
      if(bname==NULL){
        show_message("Failed to allocate memory for the blue channel file name.",emsgtype,MT_ERR,1); 
        free(rname);
        free(gname);
        return 1;
       }

      // If read_qih_file() was called successfully before calling this
      // function (and with the same fname argument) then we know fname
      // is of the correct general structure so no need to repeat those
      // checks here - just get on with it.
      sprintf(rname,"%s",fname);
      cptr=strrchr(rname,'.');
      cptr--; cptr[0]='R';
      sprintf(gname,"%s",fname);
      cptr=strrchr(gname,'.');
      cptr--; cptr[0]='G';
      sprintf(bname,"%s",fname);
      cptr=strrchr(bname,'.');
      cptr--; cptr[0]='B';
      
      // Now open all three colour channel files simultaneously
      if( (fpr=fopen(rname,"rb"))==NULL ){
         // Try lower case 'r'
         sprintf(rname,"%s",fname);
         cptr=strrchr(rname,'.');
         cptr--; cptr[0]='r';
         if( (fpr=fopen(rname,"rb"))==NULL ){
           show_message("Cannot open raw file to read it (R).",emsgtype,MT_ERR,1); 
           goto error_return;
          }
        }
      if( (fpg=fopen(gname,"rb"))==NULL ){
         // Try lower case 'g'
         sprintf(gname,"%s",fname);
         cptr=strrchr(gname,'.');
         cptr--; cptr[0]='g';
         if( (fpg=fopen(gname,"rb"))==NULL ){
           show_message("Cannot open raw file to read it (G).",emsgtype,MT_ERR,1); 
           fclose(fpr);
           goto error_return;
          }
        }
      if( (fpb=fopen(bname,"rb"))==NULL ){
         // Try lower case 'b'
         sprintf(bname,"%s",fname);
         cptr=strrchr(bname,'.');
         cptr--; cptr[0]='b';
         if( (fpb=fopen(bname,"rb"))==NULL ){
           show_message("Cannot open raw file to read it (B).",emsgtype,MT_ERR,1); 
           fclose(fpr);
           fclose(fpg);
           goto error_return;
         }
        }
        
      // Now read the data into *ptr, one pixel at a time
      rgbpos=0;
      for(len=0;len<dfsize;len++,rgbpos+=3){
         // Red pixel
         nobj=fread((*rdptr+rgbpos),sizeof(double),1,fpr);
         if(ferror(fpr)){
           show_message("File read error occurred when reading raw file (R).",emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
         if(nobj!=1){
           sprintf(emsg,"File read checksum error in raw file (R): nobj=%zu (expected 1), pos=%d",nobj,rgbpos+2);
           show_message(emsg,emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
         // Green pixel
         nobj=fread((*rdptr+rgbpos+1),sizeof(double),1,fpg);
         if(ferror(fpg)){
           show_message("File read error occurred when reading raw file (G).",emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
         if(nobj!=1){
           sprintf(emsg,"File read checksum error in raw file (G): nobj=%zu (expected 1), pos=%d",nobj,rgbpos+2);
           show_message(emsg,emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
         // Blue pixel
         nobj=fread((*rdptr+rgbpos+2),sizeof(double),1,fpb);
         if(ferror(fpb)){
           show_message("File read error occurred when reading raw file (B).",emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
         if(nobj!=1){
           sprintf(emsg,"File read checksum error in raw file (B): nobj=%zu (expected 1), pos=%d",nobj,rgbpos+2);
           show_message(emsg,emsgtype,MT_ERR,1); 
           goto error_return_fclose;
          }
        }
        
        fclose(fpr);fclose(fpg);fclose(fpb); // All went well
        free(rname);
        free(gname);
        free(bname);
     break;
     default: // This should not happen - programmer made a mistake 
         show_message("Programmer error: failed attempt to read raw file.",emsgtype,MT_ERR,1); 
         return 1;     
     break;
 }
 

 return 0;
 
 error_return_fclose:
    fclose(fpr);fclose(fpg);fclose(fpb);
  
 error_return:
  free(rname);
  free(gname);
  free(bname);
  return 1;
}

int update_preview_settings(int selected)
// Update the preview settings if there has been a change is any one of:
// 1. Main image dimensions
// 2. Camera stream format
// 3. Preview field type selected by the user (full frame or tile)
// Returns 0 on success and 1 on failure (so preview is impossible).
{
 // Blank the current preview image in case the new preview required
 // padding (we don't want part of a static old image showing in any new
 // padded regions).
 memset(PreviewImg, 127, PreviewImg_rgb_size*sizeof(unsigned char));

 // Don't allow activation the full-size tile selection mode if the full
 // frame dims are <= the preview dims
 if(selected){
   if(ImHeight<=PreviewHt && ImWidth<=PreviewWd) Preview_fullsize=0;
   else Preview_fullsize=1;
  }
  
 // If the user wants full-size tile mode:
 if(Preview_fullsize){
   // Set the currently selected tile to the default (centre) tile:
   Img_startrow=(ImHeight/2)-(PreviewHt/2); if(Img_startrow<0) Img_startrow=0;
   Img_startcol=(ImWidth/2)-(PreviewWd/2); if(Img_startcol<0) Img_startcol=0;
   // Reset the 'tile already selected' flag so user gets to select:
   Preview_tile_selection_made=0;
  }

 // Calculate the preview paramters
 Preview_impossible=calculate_preview_params(); 
 if(Preview_impossible){
     show_message("Live preview is not possible with current image dimensions.","Error: ",MT_ERR,1);
     return 1;
   }
 return 0;
}

void create_icon(void)
// Generates the program's 16x16 pixel icon.
// If unsuccessful the global icon flags will remain set to indicate
// that no icon can be used. If it succeeds they will change to indicate
// that an icon is ready and available.
{
 uint32_t icondata[64]={4294967295,3450788351,4294967292,4294967295,4294967295,23248,4294943018,4294967295,4294967295,5542,4288806912,4294967295,4294967295,10176,3932422144,4294967295,4294967295,4167661,2926772224,4294966724,3723165695,2126244834,1653286912,4294947907,1654844671,3723677552,1336642197,4292823552,4967659,4193032464,1187241454,4290117632,1086966,3939597824,1337126655,4288610304,35561,3870115840,376951551,4069720064,33256,3597402112,2462938,3869514496,34809,2921988096,11428,4040451840,1617663,2938830848,5019,4293303335,6349823,3429302272,805322926,4293779096,1472004095,4156910119,3046750956,4294967275,4211081215,4294962658,4294770687,4294967295};
 uint32_t ival;
 unsigned char ur;
 int idx,rgbpos;
 
// Allocate RAM for the 16x16x3 bytes per pixel program icon
 PardIcon_data = (unsigned char *)calloc(16*16*3,sizeof(unsigned char));
 if(PardIcon_data==NULL) return; // Would be unlikely but could happen.

// Extract the RGB values from the int icondata into the byte array
 for(idx=0,rgbpos=0;idx<64;idx++){
   ival=icondata[idx];
    ur = (unsigned char)(ival & 0xFFFFFF); // Red 1
    PardIcon_data[rgbpos++]=ur;
    PardIcon_data[rgbpos++]=ur*0.89;
    PardIcon_data[rgbpos++]=ur*0.74;
    ival >>= 8;
    ur = (unsigned char)(ival & 0xFFFF);   // Red 2
    PardIcon_data[rgbpos++]=ur;
    PardIcon_data[rgbpos++]=ur*0.89;
    PardIcon_data[rgbpos++]=ur*0.74;
    ival >>= 8;
    ur = (unsigned char)(ival & 0xFF);     // Red 3
    PardIcon_data[rgbpos++]=ur;
    PardIcon_data[rgbpos++]=ur*0.89;
    PardIcon_data[rgbpos++]=ur*0.74;
    ur = (unsigned char)(ival >> 8);       // Red 4
    PardIcon_data[rgbpos++]=ur;
    PardIcon_data[rgbpos++]=ur*0.89;
    PardIcon_data[rgbpos++]=ur*0.74; 
  } 

 PardIcon_pixbuf = gdk_pixbuf_new_from_data (PardIcon_data,GDK_COLORSPACE_RGB,FALSE,8,16,16,48,NULL,NULL);
 PardIcon_ready=1;
 return;
}

int change_image_dimensions(void)
{
 int tmpht,tmpwd,tmpstream,returnval;
 
  returnval=CID_OK; // We will change this if there is a problem.
  
  // Check if the selected dimensions for image capture differ to the
  // current ones (if so, we need to reinitialise the imaging buffers to
  // the new dimensions and recalculate preview subsampling variables)
  if((Selected_Ht != ImHeight) || (Selected_Wd != ImWidth))
   {
    show_message("Attempting to recalculate image capture dimensions:","FYI: ",MT_INFO,0);
    // Store the current values in case things go wrong and we need to
    // revert back:
    tmpht=ImHeight; tmpwd=ImWidth; 
    // Check if we are streaming - we should restart the stream at the
    // end of this if so:
    if(camera_status.cs_opened){
      if(camera_status.cs_streaming){
        tmpstream = 1;
        if(stop_streaming()){
          show_message("FAILED to change resolution.","Error: ",MT_ERR,1);
          return CID_NOCLOSE;   
         } 
       } else tmpstream=0;
      // Free the current frame buffers (that were sized according to
      // the previous dimensions):
      if(camera_status.cs_initialised) uninit_device(); 
      if(close_device()) return CID_NOCLOSE; 
     }
    // Try and resize all dimensions accoriding to the current user
    // selected values and allocate memory to the frame stores:
    if(set_dims_as_per_selected()){ 
    // If we enter this block it means we failed to get memory for the
    // new image structure so abort gracefully:
        show_message("Couldn't get enough RAM for new image size.\nAttempting to revert to previous.","Image Resize FAILED: ",MT_ERR,1);
        Selected_Ht=tmpht; Selected_Wd=tmpwd;
        if(set_dims_as_per_selected()){ 
        // We can't even go back to what we had before so something bad
        // is happening, RAM-wise. Advise user to quit while they're
        // ahead:
          show_message("FAILED to revert to the previous image dimensions.\n"
                       "This is a big problem. PARDUS will try to return control\n"
                       "to you without crashing but you should save your work\n"
                       "and exit immediately to avoid a program crash.","Image Resize FAILED: ",MT_ERR,1);
          return CID_NOREVERT;
         }
       // If we've made it to here then we have survived but only by
       // reverting back to the previous dimensions:
       returnval = CID_REVERTED;
      }
    // If we get here then the new image dims have been set.
    // Set the update preview flag:
    Preview_changed=1;
    // Now check that the current dimensions fit any selected dark field
    // or flat field or mask image - nullify those images if not:
    if(dffile_loaded!=DFIMG_NONE){
       if(DFht!=ImHeight || DFwd!=ImWidth){
         show_message("New image dimensions incompatible with currently loaded dark field image.","FYI: ",MT_INFO,0);
         nullify_darkfield();
       }
     } 
    if(fffile_loaded!=FFIMG_NONE){
       if(FFht!=ImHeight || FFwd!=ImWidth){
         show_message("New image dimensions incompatible with currently loaded flat field image.","FYI: ",MT_INFO,0);
         nullify_flatfield();
       }
     } 
    if(mask_alloced!=MASK_NO){
      if(MKht!=ImHeight || MKwd!=ImWidth){
         show_message("New image dimensions incompatible with current mask image.","FYI: ",MT_INFO,0);
         nullify_mask();
       }
     } 
    // Attempt to re-start the imaging device 
    if(open_device()){
         show_message("FAILED to re-open the imaging device after resolution change.","Error: ",MT_ERR,1);
         returnval = CID_NOSTREAM; 
     }
    if(init_device()){
         show_message("FAILED to reinitialise the device after resolution change.","Error: ",MT_ERR,1);
         returnval = CID_NOSTREAM; 
     }
    // Attempt to restart the capture stream if it was on before we
    // started all this
    if(tmpstream){
      if(start_streaming()){
         show_message("FAILED to re-start the capture stream after resolution change.","Error: ",MT_ERR,1);
         returnval = CID_NOSTREAM; 
       }
     }

   
  }                 

 return returnval;
}

static void change_cam_status(int field, char value)
// Changes the status of the <field> field of camera_status to <value>
// and sets any GUI indicators accordingly
{
 gchar *statstr;
 GtkWidget *btnlabel;
 gchar *btn_markup;
 const char *strm_onformat = "%s <span foreground=\"green\" weight=\"bold\">\%s</span>\n%s";
 const char *strm_offformat = "%s <span foreground=\"red\" weight=\"bold\">\%s</span>\n%s";

 switch(field){
     case CS_OPENED:
       camera_status.cs_opened=value;
       if(value) statstr = g_strdup_printf("Opened"); else statstr = g_strdup_printf("Closed");
      break;
     case CS_INITIALISED:
       camera_status.cs_initialised=value;
       if(value){
             statstr = g_strdup_printf("Opened->Initialised");
            } else {
             statstr = g_strdup_printf("Opened");
            }
      break;
     case CS_STREAMING:
       camera_status.cs_streaming=value;
       if(value){
             statstr = g_strdup_printf("Opened->Initialised->Streaming");
             if(gui_up){
                btn_markup = g_markup_printf_escaped (strm_offformat, "Turn","OFF","Streaming");
                btnlabel = gtk_bin_get_child(GTK_BIN(btn_cam_stream));
                gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
                g_free (btn_markup);
               }
            } else {
             statstr = g_strdup_printf("Opened->Initialised");
             if(gui_up){
                btn_markup = g_markup_printf_escaped (strm_onformat, "Turn","ON","Streaming");
                btnlabel = gtk_bin_get_child(GTK_BIN(btn_cam_stream));
                gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
                g_free (btn_markup);
              }
            }
      break;
     default: // This can only happen if the programer makes an error
        show_message("Invalid camera status value.","Program Error: ",MT_ERR,1);
      return;
   }
 if(gui_up) gtk_label_set_text (GTK_LABEL(lab_cam_status),statstr);
  
 g_free(statstr);
 return;  
}

static int calculate_preview_params(void) 
// Sets the image index arrays that are used to copy all or part of the
// main image into the preview image. This function does not deal with
// GUI aspects of the preview like displaying the overlay text or check
// boxes. The function 'update_preview_settings' does all that and also
// calls this function.
{
 double scaleh,scalew;
 int idx,dooffs,imgpos;
 
 // The preview area is fixed at PreviewHt x PreviewWd resolution so the
 // method of making a preview image depends on whether the user wants
 // to use a cropped 'preview-area-sized' tile from the full-resolution
 // image as captured from the camera as a preview image or if they want
 // to scale the whole full sized image to fit into the fixed previewing
 // area:
 
 if(Preview_fullsize){
   // This tiling process is only relevant if the full frame image is
   // bigger in any dimension than fixed preview image rectangle. So
   // check for this and revert to full-frame scale mode if not:
   if(ImHeight<=PreviewHt && ImWidth<=PreviewWd)
     goto Scale_the_full_frame_into_the_preview_box;

   if(Preview_tile_selection_made){
   // If the user made their choice of tile location by clicking on the
   // preview window, select the relevant tile for preview:
   
    // Now generate the tile integer arrays for height and width:
    for(idx=0;idx<PreviewHt;idx++){
     imgpos=(Img_startrow+idx);
     if(imgpos>=ImHeight) imgpos=-1;
     SSrow[idx]=ImWidth*imgpos;
    }
    for(idx=0;idx<PreviewWd;idx++){
     imgpos=(Img_startcol+idx);
     if(imgpos>=ImWidth) imgpos=-1;
     SScol[idx]=imgpos;
    }
   } else {
   // Otherwise display the 'Select area to zoom' overlay label and go 
   // to the full frame fit preview while the user makes their choice:
    
    goto Scale_the_full_frame_into_the_preview_box;
   }
   
  } else {
 
  Scale_the_full_frame_into_the_preview_box:
  // Find the subsampling scale factor for height and width

  scaleh = ((double)ImHeight/(double)PreviewHt);
  scalew = ((double)ImWidth/(double)PreviewWd);

  // We do not distort the image so we need to chose only one scale
  // factor to apply to both rows and columns. To avoid overshooting the
  // preview window, we chose the greater of the 2 (the higher the scale
  // factor, the more shrinkage occurs):
  Prev_scaledim=(scaleh>=scalew)?scaleh:scalew;
  
  // Now generate the subsampling integer arrays for height and width:
  for(idx=0;idx<PreviewHt;idx++){
     imgpos=(int)(Prev_scaledim*(double)idx);
     if(imgpos>=ImHeight) imgpos=-1;
     SSrow[idx]=ImWidth*imgpos;
    }
  for(idx=0;idx<PreviewWd;idx++){
     imgpos=(int)(Prev_scaledim*(double)idx);
     if(imgpos>=ImWidth) imgpos=-1;
     SScol[idx]=imgpos;
    }

 }
 
 // Now centre the preview in the PreviewWd x PreviewHt area
  for(idx=dooffs=0;idx<PreviewHt;idx++) if(SSrow[idx]<0){dooffs=1; break;}
  if(dooffs) Prev_startrow=(PreviewHt-idx)/2; else Prev_startrow=0;
  Prev_startrow1=Prev_startrow; // Needed to calculate tile offset.
  Prev_startrow*=PreviewWd*3;
  
  for(idx=dooffs=0;idx<PreviewWd;idx++) if(SScol[idx]<0){dooffs=1; break;}
  if(dooffs) Prev_startcol=(PreviewWd-idx)/2; else Prev_startcol=0;
  Prev_startcol*=3;
 
 // Adjust coords for use of RGB image instead of p
 switch(CamFormat){
   case V4L2_PIX_FMT_YUYV:
   break;
   case V4L2_PIX_FMT_MJPEG:
     for(idx=0;idx<PreviewHt;idx++)SSrow[idx]*=3;
     for(idx=0;idx<PreviewWd;idx++)SScol[idx]*=3;
   break;
   default:
   break;
  }
    
  return 0; // success
}

static int set_dims_as_per_selected(void) 
// Sets image capture size and preview subsampling dimension variables
// according to the values of the globals Selected_Ht and Selected_Wd
{
  ImHeight = Selected_Ht;
  ImWidth = Selected_Wd;
  ImWidth_stride = ImWidth*3;
  ImSize = ImHeight*ImWidth;
  // Allocate memory for the full size camera output image (also needed
  // to grab images for the preview)
  RGBsize = 3*ImSize;
  if(resize_memblk((void **)&RGBimg,(size_t)RGBsize, sizeof(unsigned char),"RGBimg")){
     show_message("No RAM for RGB image.","Error: ",MT_ERR,0);
     return 1;
   }
  if(resize_memblk((void **)&Frmr,(size_t)ImSize, sizeof(double),"Frmr")){
     show_message("No RAM for Frmr image.","Error: ",MT_ERR,0);
     return 1;
   }
  if(resize_memblk((void **)&Frmg,(size_t)ImSize, sizeof(double),"Frmg")){
     show_message("No RAM for Frmg image.","Error: ",MT_ERR,0);
     return 1;
   }
  if(resize_memblk((void **)&Frmb,(size_t)ImSize, sizeof(double),"Frmb")){
     show_message("No RAM for Frmb image.","Error: ",MT_ERR,0);
     return 1;
   }
 Frame_status=FRM_ALLOCED;

  return 0; // success
}

static void calculate_yuyv_luts(void) 
// Call each time Gain_conv or Bias_conv are altered.
{
 int ipos;
 double ky,kcc,kcr,kcb,kcrR,kcrG,kcbB,kcbG;
 double tky,tkcrR,tkcrG,tkcbB,tkcbG,conr,cong,conb,fval;
 
 if(!luts_alloced) return;
 //ITU-R BT.601 coefficients: 

 kcc = (255.0/224.0);//  = 1.138392857142857
 kcr = kcc*1.402;//      = 1.596026785714286
 kcb = kcc*1.772;//      = 2.017232142857143

 ky   = 255.0/219.0;//        (= 1.164383561643836)
 kcrR = kcr;//                (= 1.596026785714286)
 kcrG = kcr*(0.299/0.587);//  (= 0.812967647237771)
 kcbG = kcb*(0.114/0.587);//  (= 0.391762290094912)
 kcbB = kcb;//                (= 2.017232142857143)

 // The transformed coefficients are now loaded into the LUTs

 tky   = Gain_conv*ky;
 tkcrR = Gain_conv*kcrR;
 tkcrG = Gain_conv*kcrG;
 tkcbB = Gain_conv*kcbB;
 tkcbG = Gain_conv*kcbG;

 conr = Gain_conv*Bias_conv - Gain_conv*16.0*ky - 128.0*tkcrR;
 cong = Gain_conv*Bias_conv - Gain_conv*16.0*ky + 128.0*tkcrG + 128.0*tkcbG;
 conb = Gain_conv*Bias_conv - Gain_conv*16.0*ky - 128.0*tkcbB;

 for(ipos=0,fval=0.0;ipos<256;ipos++,fval++){
    lut_yR[ipos]  = tky*fval + conr;
    lut_yG[ipos]  = tky*fval + cong;
    lut_yB[ipos]  = tky*fval + conb;
    lut_crR[ipos] = tkcrR*fval;
    lut_crG[ipos] = tkcrG*fval;
    lut_cbG[ipos] = tkcbG*fval;
    lut_cbB[ipos] = tkcbB*fval;
   } 

}

unsigned char uchar_from_d(double dval)
// Returns the value 'dval' as an unsigned char without scaling but with
// clipping above 255 and below 0
{
 if(dval<0.0) dval=0.0;
 if(dval>255.0) dval=255.0;
 return (unsigned char)(dval+0.5);
}

static int jpeg_convert(const unsigned char *p, int sz)
// Decode image from the JPEG stream. Some of this code is based on the
// libjpeg-turbo GitHub repository here:
// https://github.com/leapmotion/libjpeg-turbo/blob/master/example.c
{
 struct jpeg_decompress_struct info;
 // struct jpeg_error_mgr err; // The default 'exit(1)' error handler
 struct my_error_mgr err; // Our custom error handler.
 int numComponents,retval;
 unsigned char* lpRowBuffer[1];

  // We set up the normal JPEG error routines, then override error_exit.
  info.err = jpeg_std_error(&err.pub);
  err.pub.error_exit = my_error_exit;
  // Establish the setjmp return context for my_error_exit to use.
  if (setjmp(err.setjmp_buffer)) {
    // If we get here, the JPEG code has signaled an error.
    // We need to clean up the JPEG object and return
    jpeg_destroy_decompress(&info);
    return 1; // Let caller know an error occurred
  }
 // The above code replaces this standard fatal 'exit(1)' error handler
 // info.err = jpeg_std_error(&err);
 jpeg_create_decompress(&info);
 jpeg_mem_src(&info, p, (unsigned long int)sz);
 retval = jpeg_read_header(&info, TRUE);

 if (retval != JPEG_HEADER_OK) {
     show_message("Error reading (M)JPEG frame header. Cannot make a preview image.","JPEG Error: ",MT_ERR,1);
     jpeg_destroy_decompress(&info);
     Preview_impossible=1;          // Abort preview attempt
     return 1;
    }
    
 jpeg_start_decompress(&info); 
 numComponents = info.num_components;
 if(info.output_width!=(JDIMENSION)ImWidth || info.output_height!=(JDIMENSION)ImHeight){
     show_message("Dimensions of (M)JPEG frame header don't match current dimension. Cannot make a preview image.","JPEG Error: ",MT_ERR,1);
     goto Fail_return;
 }
 if(numComponents!=3){
     show_message("(M)JPEG frame header does not have exactly 3 colour channels. Cannot make a preview image.","JPEG Error: ",MT_ERR,1);
     goto Fail_return;
 }

 // Read the decompressed image, one horizontal line at a time
 while(info.output_scanline < info.output_height) {
      lpRowBuffer[0] = (unsigned char *)(&RGBimg[3*info.output_width*info.output_scanline]);
      jpeg_read_scanlines(&info, lpRowBuffer, 1);
    }

 jpeg_finish_decompress(&info);
 jpeg_destroy_decompress(&info);

 return 0;
 
 Fail_return:
     jpeg_finish_decompress(&info);
     jpeg_destroy_decompress(&info);
     Preview_impossible=1;          // Abort preview attempt
     return 1;
}

static int colour_convert(const unsigned short *p)
// This function converts the raw data from the frame grabber buffer p
// (which will be in YUYV format) or from the JPEG frame grabber buffer
// that will have been previously decoded into the global RGBimg buffer,
// into a format that can be more easily manipulated in this program
// (e.g. for frame averaging, saving or generating a preview image).
{
 double r,g,b,fval,mn_r,mn_g,mn_b,dval1,dval2,dval3;
 int ipos,rgbpos,prow,pcol,iposp,fidx,ival,pipos;
 unsigned short pixval,y1,y2,cb,cr;
 unsigned char uy1,uy2,uy3,max;
 
        
 if(Need_to_preview==PREVIEW_ON){// We need a preview image only, not
                                 // a full-frame converion.
                                 
    if(Preview_impossible) return 0; // No point continuing but return
                                     // without error because there was
                                     // no problem in this function.
                                     
    switch(CamFormat){
        case V4L2_PIX_FMT_YUYV:
         switch(col_conv_type){
          case CCOL_TO_Y:    // Just extract the Y component (a fast op)
             rgbpos=Prev_startrow; 
              if(PreviewIDX>=Preview_integral) PreviewIDX=0;
               pipos=0;
               for(prow=0;prow<PreviewHt;prow++){ 
                  if(SSrow[prow]<0) continue;
                  for(pcol=0;pcol<PreviewWd;pcol++){
                     if(SScol[pcol]<0) continue;
                     ipos=SSrow[prow]+SScol[pcol];
                     PreviewBuff[PreviewIDX][pipos]=(int)(p[ipos] & 0xff);
                     for(fidx=0,dval1=(double)Preview_bias;fidx<Preview_integral;fidx++){
                        dval2=(double)(PreviewBuff[fidx][pipos])-Preview_dark[pipos];
                        dval2/=Preview_flat[pipos];
                        dval1+=dval2;
                       }
                     uy1=uchar_from_d(dval1);
                     pipos++;

                     PreviewImg[Prev_startcol+rgbpos++]=uy1;
                     PreviewImg[rgbpos++]=uy1;
                     PreviewImg[rgbpos++]=uy1;
                  }
              }
              PreviewIDX++;
              preview_stored = PREVIEW_STORED_MONO;
          break;
          case CCOL_TO_RGB:  // Convert to full RGB
          case CCOL_TO_BGR:  // Convert to full RGB (for preview)
              rgbpos=Prev_startrow; 
              for(prow=0;prow<PreviewHt;prow++){ 
                  if(SSrow[prow]<0) continue;
                  // The following only applies to making a colour
                  // preview from a YUYV image stream:
                  // If the preview is scaling UP the original image
                  // instead of scaling it down to a smaller size
                  // we need to scan a full row of the original and then
                  // then up-scale that (because YUYV pixels contain
                  // overlapping colour information)
                  if(Prev_scaledim<1.0){
                     ival=(SSrow[prow]+ImWidth);
                     for(ipos=SSrow[prow],pipos=0;ipos<ival;ipos+=2){
                         if(ipos%2) ipos--;
                         iposp=ipos+1;
                         //First pixel
                         pixval=p[ipos];
                         y1= pixval & 0xff; // Y1
                         cb= pixval >> 8;   // Cb1
                         // Second pixel
                         pixval=p[iposp];
                         y2= pixval & 0xff; // Y2
                         cr= pixval >> 8;   // Cr1
                         // The following is common to both pixels so
                         // calculate it only once:
                         fval = lut_crG[cr] + lut_cbG[cb]; 
                         // Get RGB from first pixel via the LUTs
                         r = lut_yR[y1] + lut_crR[cr];
                         g = lut_yG[y1] - fval; 
                         b = lut_yB[y1] + lut_cbB[cb];
                         r=r<0.0?0.0:r;g=g<0.0?0.0:g;b=b<0.0?0.0:b;
                         PreviewRow[pipos++]=(unsigned char)(r>255.0?255:r);
                         PreviewRow[pipos++]=(unsigned char)(g>255.0?255:g);
                         PreviewRow[pipos++]=(unsigned char)(b>255.0?255:b);
                         // Get RGB from second pixel via the LUTs
                         r = lut_yR[y2] + lut_crR[cr];
                         g = lut_yG[y2] - fval; 
                         b = lut_yB[y2] + lut_cbB[cb];
                         r=r<0.0?0.0:r;g=g<0.0?0.0:g;b=b<0.0?0.0:b;
                         PreviewRow[pipos++]=(unsigned char)(r>255.0?255:r);
                         PreviewRow[pipos++]=(unsigned char)(g>255.0?255:g);
                         PreviewRow[pipos++]=(unsigned char)(b>255.0?255:b);
                        }
                      for(pcol=0,ipos=0;pcol<PreviewWd;pcol++){
                         if(SScol[pcol]<0) continue;
                         ipos=SScol[pcol]; ipos*=3;
                         PreviewImg[Prev_startcol+rgbpos++]= PreviewRow[ipos++];
                         PreviewImg[rgbpos++]= PreviewRow[ipos++]; // G
                         PreviewImg[rgbpos++]= PreviewRow[ipos];   // B
                      }
                    } else {

                  for(pcol=0;pcol<PreviewWd;pcol+=2){

                     if(SScol[pcol]<0) continue;
                     ipos=SSrow[prow]+SScol[pcol];

                     // For colour conversion, we must start at an even
                     // number or we will split YUYV pixels in the wrong
                     // reading frame to get YVYU) and so switch Cr,Cb
                     // order (the colour will look wrong).
                     if(ipos%2) ipos--; 
                     iposp=ipos+1;

                     //First pixel
                     pixval=p[ipos];
                     y1= pixval & 0xff; // Y1
                     cb= pixval >> 8;   // Cb1

                     // Second pixel
                     pixval=p[iposp];
                     y2= pixval & 0xff; // Y2
                     cr= pixval >> 8;   // Cr1

                     // The following is common to both pixels so
                     // calculate it only once:
                     fval = lut_crG[cr] + lut_cbG[cb]; 
                     
                     // Get RGB from first pixel via the LUTs
                     r = lut_yR[y1] + lut_crR[cr];
                     g = lut_yG[y1] - fval; 
                     b = lut_yB[y1] + lut_cbB[cb];
            
                     // Clamp the values between 0 and 255 inclusive,
                     // convert to unsigned char and output to preview: 
                     r=r<0.0?0.0:r;g=g<0.0?0.0:g;b=b<0.0?0.0:b;
                     PreviewImg[Prev_startcol+rgbpos++]=(unsigned char)(r>255.0?255:r);
                     PreviewImg[rgbpos++]=(unsigned char)(g>255.0?255:g); 
                     PreviewImg[rgbpos++]=(unsigned char)(b>255.0?255:b);

                     // Only calculate the second pixel if we are not
                     // oversampling the main image (otherwise it would
                     // take a lot of work to get the oversmpling right
                     // in YUYV colour conversion mode.

                     // Get RGB from second pixel via the LUTs
                     r = lut_yR[y2] + lut_crR[cr];
                     g = lut_yG[y2] - fval; 
                     b = lut_yB[y2] + lut_cbB[cb];

                     // Clamp the values between 0 and 255 inclusive,
                     // convert to unsigned char and output to preview: 
                     r=r<0.0?0.0:r;g=g<0.0?0.0:g;b=b<0.0?0.0:b;
                     PreviewImg[rgbpos++]=(unsigned char)(r>255.0?255:r);
                     PreviewImg[rgbpos++]=(unsigned char)(g>255.0?255:g); 
                     PreviewImg[rgbpos++]=(unsigned char)(b>255.0?255:b);
            
                  }
                }

             }
             preview_stored = PREVIEW_STORED_RGB;
            break;
            default: break;
          }
        break;
        case V4L2_PIX_FMT_MJPEG:
         switch(col_conv_type){
          case CCOL_TO_Y:    // Calculate the Y component from RGB
             rgbpos=Prev_startrow; 
              if(PreviewIDX>=Preview_integral) PreviewIDX=0;
               pipos=0;
               for(prow=0;prow<PreviewHt;prow++){ 
                  if(SSrow[prow]<0) continue;
                  for(pcol=0;pcol<PreviewWd;pcol++){
                     if(SScol[pcol]<0) continue;
                     ipos=SSrow[prow]+SScol[pcol];
            
                     uy1= RGBimg[ipos++]; // R
                     uy2= RGBimg[ipos++]; // G
                     uy3= RGBimg[ipos];   // B

                     max=(uy2>uy1)?uy2:uy1;
                     if(uy3>=max) max=uy3;
                     PreviewBuff[PreviewIDX][pipos]=(int)max;
                     for(fidx=0,dval1=(double)Preview_bias;fidx<Preview_integral;fidx++){
                        dval2=(double)(PreviewBuff[fidx][pipos])-Preview_dark[pipos];
                        dval2/=Preview_flat[pipos];
                        dval1+=dval2;
                       }
                     uy1=uchar_from_d(dval1);
                     pipos++;

                     PreviewImg[Prev_startcol+rgbpos++]=uy1;
                     PreviewImg[rgbpos++]=uy1;
                     PreviewImg[rgbpos++]=uy1;
                  }
              }
              PreviewIDX++;
              preview_stored = PREVIEW_STORED_MONO;
          break;
          case CCOL_TO_RGB:  // Convert to full RGB
          case CCOL_TO_BGR:  // Convert to full RGB (for preview)
              rgbpos=Prev_startrow; 
              for(prow=0;prow<PreviewHt;prow++){ 
                  if(SSrow[prow]<0) continue;
                  for(pcol=0;pcol<PreviewWd;pcol++){
                     if(SScol[pcol]<0) continue;
                     ipos=SSrow[prow]+SScol[pcol];

                     PreviewImg[Prev_startcol+rgbpos++]= RGBimg[ipos++];
                     PreviewImg[rgbpos++]= RGBimg[ipos++]; // G
                     PreviewImg[rgbpos++]= RGBimg[ipos];   // B
                  }
              }
             preview_stored = PREVIEW_STORED_RGB;
            break;
            default: break;
           }
        }
      
      return 0; // Preview image created, so return.
   }

 // Anything more than PREVIEW_ON requires a full size conversion.

 // First, get the single frame into the Frm[r,g,b] frame buffers and
 // calculate the mean average of each frame while we do it:
 switch(CamFormat){
       
    case V4L2_PIX_FMT_YUYV:
    
      switch(col_conv_type){
        case CCOL_TO_Y:    // Just extract the Y component - a quick op.
              rgbpos=0;    // We only use the 'red' channel of the 
                           // Frame buffer arrays to store this.
              mn_r=0.0;
              for(ipos=0;ipos<ImSize;ipos+=2){ 
                  // Y1 - intensity value of first pixel
                  dval1=(double)(p[ipos] & 0xff);
                  Frmr[rgbpos++] = dval1; 
                  // Calculate the mean within the support of the mask
                  if(MaskIm[ipos]>0) mn_r=+ dval1; 
                  // Y2 - intensity value of second pixel
                  dval2=(double)(p[ipos+1] & 0xff);  
                  Frmr[rgbpos++] = dval2;
                  // Calculate the mean within the support of the mask
                  if(MaskIm[ipos+1]>0) mn_r=+ dval2; 
                 }
              mn_r/=Mask_supp_size;
          break;
        case CCOL_TO_RGB : // Convert to full RGB
        case CCOL_TO_BGR:  // Convert to full RGB for saving as BMP file
              mn_r=0.0; mn_g=0.0; mn_b=0.0;
              for(ipos=0;ipos<ImSize;ipos+=2){ iposp=ipos+1; 
                 //First pixel
                 pixval=p[ipos];
                 y1= pixval & 0xff; // Y1
                 cb= pixval >> 8;   // Cb1
                 // Second pixel
                 pixval=p[iposp];
                 y2= pixval & 0xff; // Y2
                 cr= pixval >> 8;   // Cr1
                 // The following is common to both pixels so calculate
                 // it only once:
                 fval = lut_crG[cr] + lut_cbG[cb];
                 // Get RGB from first pixel via the LUTs
                 dval1=(lut_yR[y1] + lut_crR[cr]);
                 dval2=(lut_yG[y1] - fval);
                 dval3=(lut_yB[y1] + lut_cbB[cb]);
                 Frmr[ipos] = dval1;
                 Frmg[ipos] = dval2; 
                 Frmb[ipos] = dval3;
                 // Calculate the mean within the support of the mask
                 if(MaskIm[ipos]>0){
                    mn_r+=dval1;
                    mn_g+=dval2;
                    mn_b+=dval3;
                  }
                 // Get RGB from second pixel via the LUTs
                 dval1=(lut_yR[y2] + lut_crR[cr]);
                 dval2=(lut_yG[y2] - fval);
                 dval3=(lut_yB[y2] + lut_cbB[cb]);
                 Frmr[iposp] = dval1;
                 Frmg[iposp] = dval2; 
                 Frmb[iposp] = dval3;
                 // Calculate the mean within the support of the mask
                 if(MaskIm[iposp]>0){
                    mn_r+=dval1;
                    mn_g+=dval2;
                    mn_b+=dval3;
                  }
               }
              mn_r/=Mask_supp_size;
              mn_g/=Mask_supp_size;
              mn_b/=Mask_supp_size;
            break;
          default: break;
        }
        
    break;
    case V4L2_PIX_FMT_MJPEG:
    
      switch(col_conv_type){
        case CCOL_TO_Y:    // Convert from RGB to Y
                           // We only use the 'red' channel of the 
                           // frame buffer to store this.
        // Because all JPEG images are RGB we convert the RGB data to
        // intensity data using the 'I' part of an HSI transform i.e.
        // I=(R+G+B)/3.
              mn_r=0.0;
              for(ipos=rgbpos=0;ipos<ImSize;ipos++){ 
                  uy1= RGBimg[rgbpos++]; // R
                  uy2= RGBimg[rgbpos++]; // G
                  uy3= RGBimg[rgbpos++]; // B
                  dval1=(double)uy1+(double)uy2+(double)uy3;
                  dval1/=3.0;
                  Frmr[ipos] = dval1;
                  // Calculate the mean within the support of the mask
                  if(MaskIm[ipos]>0) mn_r=+ dval1; 
                 }
              mn_r/=Mask_supp_size;
          break;
        case CCOL_TO_RGB : // Convert to full RGB
        case CCOL_TO_BGR:  // Convert to full RGB for saving as BMP file
              mn_r=0.0; mn_g=0.0; mn_b=0.0;
              for(ipos=rgbpos=0;ipos<ImSize;ipos++){
                 dval1=(double)RGBimg[rgbpos++];
                 dval2=(double)RGBimg[rgbpos++];
                 dval3=(double)RGBimg[rgbpos++];
                 Frmr[ipos] = dval1; // R
                 Frmg[ipos] = dval2; // G 
                 Frmb[ipos] = dval3; // B
                 // Calculate the mean within the support of the mask
                 if(MaskIm[ipos]>0){
                    mn_r+=dval1;
                    mn_g+=dval2;
                    mn_b+=dval3;
                  }
               }
              mn_r/=Mask_supp_size;
              mn_g/=Mask_supp_size;
              mn_b/=Mask_supp_size;
            break;
          default: break;
        }
        
    default: break;
      
   }
 
 // Now we apply dark field correction if requested:
 if(do_df_correction){

   switch(CamFormat){
       
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_MJPEG:
   
      switch(col_conv_type){
        case CCOL_TO_Y:
              for(ipos=0;ipos<ImSize;ipos++){
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0) Frmr[ipos] -= DF_Image[ipos];
                }
          break;
        case CCOL_TO_RGB: 
        case CCOL_TO_BGR:
              for(ipos=rgbpos=0;ipos<ImSize;ipos++,rgbpos+=3){ 
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0){
                    Frmr[ipos] -= DF_Image[rgbpos];
                    Frmg[ipos] -= DF_Image[rgbpos+1];
                    Frmb[ipos] -= DF_Image[rgbpos+2];
                   }
               }
            break;
          default: break;
        }
        
    break;
    default: break;
      
   }

  }

 // Now we apply flat field correction if requested:
 if(do_ff_correction){

   switch(CamFormat){
       
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_MJPEG:
   
      switch(col_conv_type){
        case CCOL_TO_Y:
              for(ipos=0;ipos<ImSize;ipos++){
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0) Frmr[ipos] /= FF_Image[ipos];
                }
          break;
        case CCOL_TO_RGB: 
        case CCOL_TO_BGR:
              for(ipos=rgbpos=0;ipos<ImSize;ipos++,rgbpos+=3){ 
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0){
                    Frmr[ipos] /= FF_Image[rgbpos];
                    Frmg[ipos] /= FF_Image[rgbpos+1];
                    Frmb[ipos] /= FF_Image[rgbpos+2];
                   }
               }
            break;
          default: break;
        }
        
    break;
    default: break;
      
   }

  }

 // If we are doing multiframe averaging, do any mean scaling and then
 // accumulate the values in the average stores. 
 if(Av_limit>1 && Accumulator_status==ACC_ALLOCED){

 // Scale the mean of each frame to the same value as the mean of the
 // very first frame (if the user asked for this):
 if(Av_scalemean){
  
    // If we are at the very first frame just copy its mean into the
    // global variables:
    if(Av_denom_idx==1){  
      Av_meanr = mn_r;
      Av_meang = mn_g;
      Av_meanb = mn_b;
     } else {
     // For all subsequent frames calculate the scale factor and do the
     // the scaling.

     // First check to avoid divide-by-zero). For practcal reasons I use
     // a small number instead of absolute zero and I use 1.0e-10. This
     // is reasonable becuase the OptArc camera pixel values will always
     // be above 1 (usually the lowest value recordable is 16) and I
     // most other cameras will likewise give a lowest value of either
     // zero or some integer (so 1 will be the lowest after 0). Thus if
     // a mean < 1.0e-10 means the mean will be actually zero on all
     // resolutions even upto over 32K. The above calculations only hold
     // true if there is no dark field subtraction because that could
     // tip pixels into negative values but still the mean across all
     // pixels should be positive unless you are subtracting a dark
     // field image from a dark field image (which will not normally be
     // the case and should not be done): 
       switch(col_conv_type){
        case CCOL_TO_Y:
         if(mn_r<1.0e-10){
            show_message("Mean mn_r is too low - setting to 1.0","WARNING: ",MT_INFO,0);
            mn_r=1.0;
           }
          break;
        case CCOL_TO_RGB: 
        case CCOL_TO_BGR:
         if(mn_r<1.0e-10){
            show_message("Mean mn_r is too low - setting to 1.0","WARNING: ",MT_INFO,0);
            mn_r=1.0;
           }
         if(mn_g<1.0e-10){
            show_message("Mean mn_g is too low - setting to 1.0","WARNING: ",MT_INFO,0);
            mn_g=1.0;
           }
         if(mn_b<1.0e-10){
            show_message("Mean mn_b is too low - setting to 1.0","WARNING: ",MT_INFO,0);
            mn_b=1.0;
           }
          break;
          default: break;
        }


      switch(CamFormat){
       
       case V4L2_PIX_FMT_YUYV:
       case V4L2_PIX_FMT_MJPEG:
   
         switch(col_conv_type){
           case CCOL_TO_Y:
              r=Av_meanr/mn_r;
              for(ipos=0;ipos<ImSize;ipos++){
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0) Frmr[ipos] *= r;
                }
           break;
           case CCOL_TO_RGB: 
           case CCOL_TO_BGR:
              r=Av_meanr/mn_r;
              g=Av_meang/mn_g;
              b=Av_meanb/mn_b;
              for(ipos=0;ipos<ImSize;ipos++){ 
                 // Only correct within the support of the mask
                 if(MaskIm[ipos]>0){
                    Frmr[ipos] *= r;
                    Frmg[ipos] *= g;
                    Frmb[ipos] *= b;
                   }
               }
           break;
           default: break;
         }
        
       break;
       default: break;
      
      }


     }

  }

 // Now accumulate the frame into the average buffer
   switch(CamFormat){
       
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_MJPEG:
   
      switch(col_conv_type){
        case CCOL_TO_Y:
              for(ipos=0;ipos<ImSize;ipos++) Avr[ipos] += Frmr[ipos];
          break;
        case CCOL_TO_RGB: 
        case CCOL_TO_BGR:
              for(ipos=0;ipos<ImSize;ipos++){ 
                    Avr[ipos] += Frmr[ipos];
                    Avg[ipos] += Frmg[ipos];
                    Avb[ipos] += Frmb[ipos];
               }
            break;
          default: break;
        }
        
    break;
    default: break;
      
   }


 } else {
 // If we are NOT doing multiframe averging, ensure the Frm frame store
 // image is placed into the RGBim unsigned char array ready for output
 // with suitable clipping into the range [0-255] for each colour
 // channel.
     
   switch(CamFormat){
       
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_MJPEG:
    
     switch(col_conv_type){
        case CCOL_TO_Y:
              for(ipos=0;ipos<ImSize;ipos++){ 
                  RGBimg[ipos] = uchar_from_d(Frmr[ipos]);
                 }
          break;
        case CCOL_TO_RGB :  // Convert to full RGB
              rgbpos=0; 
              for(ipos=0;ipos<ImSize;ipos++){ 
                  RGBimg[rgbpos++] = uchar_from_d(Frmr[ipos]);
                  RGBimg[rgbpos++] = uchar_from_d(Frmg[ipos]);
                  RGBimg[rgbpos++] = uchar_from_d(Frmb[ipos]);
               }
            break;
        case CCOL_TO_BGR:  // Convert to full RGB for saving as BMP file
              rgbpos=0; 
              for(ipos=0;ipos<ImSize;ipos++){ 
                  RGBimg[rgbpos++] = uchar_from_d(Frmb[ipos]);
                  RGBimg[rgbpos++] = uchar_from_d(Frmg[ipos]);
                  RGBimg[rgbpos++] = uchar_from_d(Frmr[ipos]);
               }
            break;
          default: break;
        }
        
    break;
    default: break;
      
   }

 }
 
 return 0; // success   
}

static int next_windex(void)
{
  windex++;
  if(windex==MAX_CAM_SETTINGS){
     show_message("There are too many settings to display them all!","Error: ",MT_ERR,1);
     return 1;
    }
 return 0; 
}

static int xioctl(int fh, int request, void *arg)
{
 int r;

 do {
     r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

 return r;
}

static void process_image(const void *p, int size)
{
 char imsg[128];
 int cresult,tmp_colconvtype,idx,fnum_used=0;
 int averaging_done=0;

 // Reset the preview stored status. We just captured a new image so any
 // existing preview image is out of date.
 preview_stored=PREVIEW_STORED_NONE; 
 
 // Store currently selected preview colour conversion type. Processing
 // may change this so we will need to restore it at the end.
 tmp_colconvtype=col_conv_type; 

// We only do frame averaging for saved images so if a saved image is
// not required set the frame average loop to the end immediately and
// set Av_limit to 1 (i.e. single frame, no averaging):
 if(!Need_to_save) Av_denom_idx=Av_limit=1; 
 
 if(Need_to_save){ //  We are outputting the image.                                          
     
    // First initialise the frame averaging stores if averaging more
    // than 1 frame.
    if(Av_limit>1 && Av_denom_idx==1){  
       // If we are at the very first frame ...                                     

       switch(saveas_fmt){
           case SAF_YUYV:// Not yet supported for multiframe averaging
                show_message("No multi-frame averging will be done because YUYV save-as format does not support it.","Error: ",MT_ERR,0);
                goto av_fail;
           break;
           case SAF_YP5: // Only the first ImSize bytes of RGBimg are
           case SAF_BM8: // used for these options
               if(resize_memblk((void **)&Avr,(size_t)ImSize, sizeof(double),"Avr")) goto av_fail;
               // Even though resize_memblk uses calloc, setting all
               // bits to binary zero does not guarantee that the float
               // representation will also be zero (although it likely
               // will be). So, to be sure, I initialise to 0.0:
               for(idx=0;idx<ImSize;idx++) Avr[idx]=0.0;
               Accumulator_status=ACC_ALLOCED;
           break;
           case SAF_RGB: // The whole RGBimg array is used 
           case SAF_BMP: // (RGBsize = 3xImSize) for these options.
           case SAF_PNG:
           case SAF_JPG: 
           case SAF_INT: 
               if(resize_memblk((void **)&Avr,(size_t)ImSize, sizeof(double),"Avr")) goto av_fail;
               if(resize_memblk((void **)&Avg,(size_t)ImSize, sizeof(double),"Avg")) goto av_fail;
               if(resize_memblk((void **)&Avb,(size_t)ImSize, sizeof(double),"Avb")) goto av_fail;
               for(idx=0;idx<ImSize;idx++){ // See comment above.
                  Avr[idx]=0.0;
                  Avg[idx]=0.0;
                  Avb[idx]=0.0;
                 }
               Accumulator_status=ACC_ALLOCED;
           break;
           default: // Should not happen - ther is a programming error
                show_message("No multi-frame averging will be done due to a programming error.","Error: ",MT_ERR,0);
              goto av_fail;
           break;
        }       
       // Success - continue to colour conversion with clear frame
       // averaging buffers:
       goto colour_calcs; 
       // Failure - set the averaging flags to indicate no averaging can
       // be done, then continue to colour conversion:
       av_fail: 
       Av_denom_idx=Av_limit=1;
       averaging_done=0;
       Accumulator_status=ACC_FREED;
       resize_memblk((void **)&Avr,1, sizeof(double),"Avr");
       resize_memblk((void **)&Avg,1, sizeof(double),"Avg");
       resize_memblk((void **)&Avb,1, sizeof(double),"Avb");
     } 
      
    // Now do the appropriate RGB conversion for the 'save as' format 
    colour_calcs:
    switch(CamFormat){ // Perform any YUYV conversion required by the save as format.
          case V4L2_PIX_FMT_YUYV:
           switch(saveas_fmt){
               case SAF_YUYV:// Requires no conversion at all - quickest method
               break;
               case SAF_YP5: // Requires Y-extraction from YUYV - a quick process
               case SAF_BM8: // Requires Y-extraction from YUYV - a quick process
                 col_conv_type=CCOL_TO_Y;  // Change current conversion type to Y-only
                 cresult = colour_convert((const unsigned short *)p); // Do the conversion
                 if(cresult){ // Test for success of colour conversion
                   show_message("Y-extraction failed. Cannot save this image.","Save Error: ",MT_ERR,1);
                   Need_to_save=0; // Abort save process if colour conversion failed
                  }
               break;
               case SAF_BMP: // Requires YUYV to BGR conversion
                 col_conv_type=CCOL_TO_BGR;// Change current conversion type to full RGB conversion - ordered as BGR
                 goto convert_rgb;
               case SAF_RGB: // These require YUYV to RGB conversion
               case SAF_JPG:
               case SAF_PNG:
               case SAF_INT:
                 col_conv_type=CCOL_TO_RGB ;// Change current conversion type to full RGB conversion
                 convert_rgb:
                 cresult = colour_convert((const unsigned short *)p); // Do the conversion
                 if(cresult){ // Test for success - can only proceed to saving the file if the conversion worked.
                   show_message("YUYV to RGB conversion failed. Cannot save this image.","Save Error: ",MT_ERR,1);
                   Need_to_save=0; // Abort save process
                  }
               break;
               default: // This should not happen unless there has been a Program Error
                 show_message("An invalid save as format was encountered.","Program Error: ",MT_ERR,1);
                 Need_to_save=0; // abort save process
               break;
            }
          break;
          case V4L2_PIX_FMT_MJPEG:
           // Decode the MJPEG stream image (which is in JPEG format)
           if(jpeg_convert((const unsigned char *)p,size)){
            sprintf(imsg,"Failed to decode the JPEG image from the camera.");
            show_message(imsg,"Error: ",MT_ERR,0);
           }
           switch(saveas_fmt){
               case SAF_YUYV: // This is only available if the CamFormat 
                              // is V4L2_PIX_FMT_YUYV
                   show_message("Can't save image in YUYV format when the camera is in MJPEG mode. Cannot save this image.","Save Error: ",MT_ERR,1);
                   Need_to_save=0;          // Abort the save process
               break;
               case SAF_YP5: // Need Y-calculation from JPEG decoded RGB
               case SAF_BM8: // Need Y-calculation from JPEG decoded RGB
                 col_conv_type=CCOL_TO_Y;  // Change to Y-only
                 cresult = colour_convert(NULL); // Do the conversion
                 // Test for success - can only proceed to saving the
                 // file if the conversion worked.
                 if(cresult){ 
                   show_message("Y-calculation failed. Cannot save this image.","Save Error: ",MT_ERR,1);
                   Need_to_save=0;          // Abort the save process
                  }
               break;
               case SAF_BMP: // Requires JPEG to BGR conversion
                 col_conv_type=CCOL_TO_BGR;// Ensure full BGR conversion
                 goto convert_jrgb;
               case SAF_JPG:
                // Only need convert a JPEG if we are doing averaging 
                if(Av_limit>1){
                    col_conv_type=CCOL_TO_RGB;
                    goto convert_jrgb;
                }
               break;
               case SAF_RGB: // Require JPEG to RGB conversion
               case SAF_PNG:
               case SAF_INT:
                 col_conv_type=CCOL_TO_RGB;// Ensure full RGB conversion
                 convert_jrgb:
                 cresult = colour_convert(NULL); // Do the conversion
                 // Test for success - can only proceed to saving the
                 // file if the conversion worked.
                 if(cresult){               
                   show_message("JPEG to RGB conversion failed. Cannot save this image.","Save Error: ",MT_ERR,1);
                   Need_to_save=0;          // Abort the save process
                  }
               break;
          break;
          default: // This should not happen
                 show_message("An invalid 'save as' format was encountered.","Program Error: ",MT_ERR,1);
                 Need_to_save=0;          // Abort the save process
                 Av_denom_idx=Av_limit=1; // Cancel any frame averaging
          break;
        }
    }
   }
                                           
 if(Av_limit>1){ // If we are doing multi-frame averaging ...

   if(Av_denom_idx<Av_limit)        // ...and have not yet accumulated
     goto skip_write;               //  the last frame, don't write the
                                    // file to disc just yet. 
                                    
   else if(Av_denom_idx==Av_limit){ // However, if we have accumulated
     int ipos,rgbpos;               // the last frame then do the final 
     double r,g,b;                  // division and transfer the
                                    // resulting averages to the write
                                    // buffer.
     averaging_done=1; // Lets the file save code know that we have an
                       // average file to save if the user selected to
                       // save the average images as raw doubles files.
   
     switch(saveas_fmt){
       case SAF_YUYV:// This is not currently supported for multiframe averaging
       break;
       case SAF_YP5: // Only the first ImSize bytes of RGBimg are used
       case SAF_BM8: // Only the first ImSize bytes of RGBimg are used
          // First divide by accumulation array by the average
          // denominator so it holds the average pixel values:
          for(ipos=0;ipos<ImSize;ipos++) Avr[ipos]/=(double)Av_limit;
          // Now transfer the result, unaltered, into the doubles frame
          // buffer and clamp the values between 0 and 255 and insert
          // them into the unsigned char write buffer:
          for(ipos=0;ipos<ImSize;ipos++){
              r=Avr[ipos];
              Frmr[ipos]=r; // Original value goes here
              RGBimg[ipos]=uchar_from_d(r); // Clipped value goes here
            }
       break;
       case SAF_RGB: // The whole RGBimg array is used
       case SAF_PNG: // 
       case SAF_JPG: //
       case SAF_INT: //
          // First divide by accumulation arrays by the average
          // denominator so they hold the average pixel values:
          for(ipos=0;ipos<ImSize;ipos++){ 
              Avr[ipos]/=(double)Av_limit;
              Avg[ipos]/=(double)Av_limit;
              Avb[ipos]/=(double)Av_limit;
            }
          // Now transfer the result, unaltered, into the doubles frame
          // buffers and clamp the values between 0 and 255 and insert
          // them into the unsigned char write buffer:
          rgbpos=0; 
          for(ipos=0;ipos<ImSize;ipos++){ 
              r=Avr[ipos];
              g=Avg[ipos];
              b=Avb[ipos];
              Frmr[ipos]=r; // Original value goes here
              Frmg[ipos]=g; // Original value goes here
              Frmb[ipos]=b; // Original value goes here
              RGBimg[rgbpos++]=uchar_from_d(r); // Clipped value goes here 
              RGBimg[rgbpos++]=uchar_from_d(g); // Clipped value goes here
              RGBimg[rgbpos++]=uchar_from_d(b); // Clipped value goes here
             }
       break;
       case SAF_BMP: // Same as for the RGB procedure but reorder to BGR
          // First divide by accumulation arrays by the average
          // denominator so they hold the average pixel values:
          for(ipos=0;ipos<ImSize;ipos++){ 
              Avr[ipos]/=(double)Av_limit;
              Avg[ipos]/=(double)Av_limit;
              Avb[ipos]/=(double)Av_limit;
            }
          // Now transfer the result, unaltered, into the doubles frame
          // buffers and clamp the values between 0 and 255 and insert
          // them into the unsigned char write buffer:
          rgbpos=0; 
          for(ipos=0;ipos<ImSize;ipos++){ 
              r=Avr[ipos];
              g=Avg[ipos];
              b=Avb[ipos];
              Frmr[ipos]=r; // Original value goes here
              Frmg[ipos]=g; // Original value goes here
              Frmb[ipos]=b; // Original value goes here
              RGBimg[rgbpos++]=uchar_from_d(b); // Clipped value goes here 
              RGBimg[rgbpos++]=uchar_from_d(g); // Clipped value goes here
              RGBimg[rgbpos++]=uchar_from_d(r); // Clipped value goes here
             }
       break;
       default:
       break;
      }

    // Now 'free' averaging buffers now
    show_message("Freeing accumulator buffers.","FYI: ",MT_INFO,0);
    Accumulator_status=ACC_FREED; // Always set this BEFORE freeing
                                  // in case a GUI button interrupt
                                  // occurs before all the resize
                                  // commands are completed.
    resize_memblk((void **)&Avr,1, sizeof(double),"Avr");
    resize_memblk((void **)&Avg,1, sizeof(double),"Avg");
    resize_memblk((void **)&Avb,1, sizeof(double),"Avb");


   } // End of if...else we are at the last frame of multi-frame averaging
  } // End of if we are doing multiframe averaging
    
 if(Need_to_save){
    // Construct the appropriate file name using ImRoot, frame_number
    // and the save as format then write the data to disk.
    FILE *fp;                                           
    
    // Save image to local disk.
    // Note: Always do the 'if(Save_as_FITS)' option last because the
    // FITS save function may byte-swap the original image data.
    switch(saveas_fmt){
          case SAF_YUYV:
            // In YUYV mode we just save the frame buffer as it comes
            // out of the camera, no masking, dark field, flat field or
            // averaging process are applied and we do not allow saving
            // as raw doubles or FITS so no need to check for those:
            sprintf(Ser_name, "%s_%04d_yuyv.raw",ImRoot,frame_number);fnum_used=1;
            fp=fopen(Ser_name,"wb");
            if(fp==NULL){
               show_message("Failed to open file for writing raw YUYV image.","File Save FAILED: ",MT_ERR,1); 
               break;
              }
            fwrite(p, size, 1, fp);  fflush(fp);  fclose(fp);
          break;
          case SAF_YP5:
            sprintf(Ser_name, "%s_%04d_Y.pgm",ImRoot,frame_number);fnum_used=1;
            if(raw_to_pgm(Ser_name,ImHeight,ImWidth,(void *)RGBimg)) break;
            // If the user requested to save raw doubles, do it now:
            if(Save_raw_doubles){
              sprintf(Ser_name, "%s_%04d_Y.dou",ImRoot,frame_number);fnum_used=1;
              if(write_rawdou(Ser_name,CCHAN_Y)) break;
            }
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_Y.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_Y,averaging_done)) break;
            }
          break;
          case SAF_BM8:
            sprintf(Ser_name, "%s_%04d_Y.bmp",ImRoot,frame_number);fnum_used=1;
            if(raw_to_bmp((unsigned int)ImHeight,(unsigned int)ImWidth,&RGBimg,Ser_name,BM8)){ show_message("Failed to save 8 bpp BMP image.","File Save FAILED: ",MT_ERR,1); break;}
            // If the user requested to save raw doubles, do it now:
            if(Save_raw_doubles){
              sprintf(Ser_name, "%s_%04d_Y.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_Y)) break;
            }
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_Y.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_Y,averaging_done)) break;
            }
          break;
          case SAF_PNG:
            sprintf(Ser_name, "%s_%04d.png",ImRoot,frame_number);fnum_used=1;
            if(write_png_image(Ser_name, ImWidth, ImHeight, RGBimg, Ser_name))
               { show_message("Error writing PNG image.\n","File Save FAILED: ",MT_ERR,1); break;}
            // If the user requested to save raw doubles, do it now:
            if(Save_raw_doubles){
              sprintf(Ser_name, "%s_%04d_R.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_R)) break;
              sprintf(Ser_name, "%s_%04d_G.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_G)) break;
              sprintf(Ser_name, "%s_%04d_B.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_B)) break;
            }
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_R.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_R,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_G.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_G,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_B.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_B,averaging_done)) break;
            }
          break;
          case SAF_RGB:
            sprintf(Ser_name, "%s_%04d_rgb.ppm",ImRoot,frame_number);fnum_used=1;
            if(raw_to_ppm(Ser_name,ImHeight,ImWidth,(void *)RGBimg)) break;
            // If the user requested to save raw doubles, do it now:
            if(Save_raw_doubles){
              sprintf(Ser_name, "%s_%04d_R.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_R)) break;
              sprintf(Ser_name, "%s_%04d_G.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_G)) break;
              sprintf(Ser_name, "%s_%04d_B.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_B)) break;
            }
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_R.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_R,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_G.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_G,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_B.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_B,averaging_done)) break;
            }
          break;
          case SAF_INT:
            // This is always saved as a raw doubles file so no need to
            // check 'if(Save_raw_doubles)'
            sprintf(Ser_name, "%s_%04d_I.dou",ImRoot,frame_number);fnum_used=1;
            // Now do the RGB to Intensity conversion and put the result
            // into Frmr array:
            rgb_to_int();
            // Save the result as raw doubles (always use CCHAN_Y, not
            // ichan because, by now, the result will be in Frmr and we
            // are saving only this channel as a monochrome image):
            write_rawdou(Ser_name,CCHAN_Y);
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_I.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_Y,averaging_done)) break;
            }
          break;
          case SAF_BMP:
            sprintf(Ser_name, "%s_%04d_rgb.bmp",ImRoot,frame_number);fnum_used=1;
            if(raw_to_bmp((unsigned int)ImHeight,(unsigned int)ImWidth,&RGBimg,Ser_name,BMP)){ show_message("Failed to save 24 bpp BMP image.","File Save FAILED: ",MT_ERR,1); break;}
            if(Save_raw_doubles){
              sprintf(Ser_name, "%s_%04d_R.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_R)) break;
              sprintf(Ser_name, "%s_%04d_G.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_G)) break;
              sprintf(Ser_name, "%s_%04d_B.dou",ImRoot,frame_number);
              if(write_rawdou(Ser_name,CCHAN_B)) break;
            }
            if(Save_as_FITS){
              sprintf(Ser_name, "%s_%04d_R.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_R,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_G.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_G,averaging_done)) break;
              sprintf(Ser_name, "%s_%04d_B.fit",ImRoot,frame_number);
              if(write_fits(Ser_name,CCHAN_B,averaging_done)) break;
            }
          break;
          case SAF_JPG:
            sprintf(Ser_name, "%s_%04d.jpg",ImRoot,frame_number);fnum_used=1;
            // If the camera was not in MJPEG stream mode or if we did
            // multi-frame averaging, the image data will be in RGBim
            // and the Frm stores, so save those as appropriate:
            if(averaging_done || CamFormat!=V4L2_PIX_FMT_MJPEG){
              if(raw_to_jpeg(ImHeight,ImWidth,&RGBimg,Ser_name,JPG_Quality)){
                 show_message("Failed to save JPEG image.","File Save FAILED: ",MT_ERR,1);
                 break;
                }
              if(Save_raw_doubles){
                sprintf(Ser_name, "%s_%04d_R.dou",ImRoot,frame_number);
                if(write_rawdou(Ser_name,CCHAN_R)) break;
                sprintf(Ser_name, "%s_%04d_G.dou",ImRoot,frame_number);
                if(write_rawdou(Ser_name,CCHAN_G)) break;
                sprintf(Ser_name, "%s_%04d_B.dou",ImRoot,frame_number);
                if(write_rawdou(Ser_name,CCHAN_B)) break;
              }            
              if(Save_as_FITS){
                sprintf(Ser_name, "%s_%04d_R.fit",ImRoot,frame_number);
                if(write_fits(Ser_name,CCHAN_R,averaging_done)) break;
                sprintf(Ser_name, "%s_%04d_G.fit",ImRoot,frame_number);
                if(write_fits(Ser_name,CCHAN_G,averaging_done)) break;
                sprintf(Ser_name, "%s_%04d_B.fit",ImRoot,frame_number);
                if(write_fits(Ser_name,CCHAN_B,averaging_done)) break;
              }
            } else {
            // Otherwise we just save the data directly from the frame
            // grabber as a single frame. Saving as raw doubles or FITS
            // is not permitted in this case so we don't check for those
            // options. No masking, dark field or flat field correction
            // will be applied.
              fp=fopen(Ser_name,"wb");
              if(fp==NULL){
                 show_message("Failed to open file for writing JPEG image.","File Save FAILED: ",MT_ERR,1);
                 break;
                }
              fwrite(p, size, 1, fp);  fflush(fp);  fclose(fp);
            }
          break;
          default: // This case won't happen, we checked for it above.
          break;
        }

   }

skip_write:
 col_conv_type=tmp_colconvtype; // Restore current preview colour conversion type
 if(fnum_used) frame_number++;  // Increment the frame counter.
   
 if(Need_to_preview){
    if(preview_stored==PREVIEW_STORED_NONE){
    // No other process (e.g. save image) has generated a preview image
    // but a preview is required - so generate one now:
       switch(CamFormat){
         case V4L2_PIX_FMT_MJPEG:   
           // Decode the MJPEG stream image (which is in JPEG format)
           // to an uncompressed bitmap form for previewing.
           if(jpeg_convert((const unsigned char *)p,size)){
            sprintf(imsg,"Failed to decode a JPEG preview image. Previewing will be turned off.");
            show_message(imsg,"Error: ",MT_ERR,1);
            // Switch off the preview
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_cam_preview),FALSE);
           }
           if(colour_convert(NULL)){ // Now try making the preview image
            sprintf(imsg,"Failed to colour convert a JPEG preview image. Previewing will be turned off.");
            show_message(imsg,"Error: ",MT_ERR,1);
            // Switch off the preview
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_cam_preview),FALSE);
           }
         break; 
         case V4L2_PIX_FMT_YUYV:
          if(colour_convert((const unsigned short *)p)){
            sprintf(imsg,"Failed to subsample a YUYV preview image. Previewing will be turned off.");
            show_message(imsg,"Error: ",MT_ERR,1);
            // Switch off the preview
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_cam_preview),FALSE);
           }
         break; 
         default:
          if(preview_only_once){
              sprintf(imsg,"Preview is only available for YUYV and MJPEG image streams");
              show_message(imsg,"FYI: ",MT_INFO,0);
            }
          preview_only_once = 0;
          break;
        }
     }
  // Update the preview window with the latest preview image (if one was successfully generated).
  if(Need_to_preview && preview_stored) gtk_image_set_from_pixbuf (GTK_IMAGE(Img_preview),gdkpb_preview);
 }

 return;
}

static int read_frame(void)
{
 struct v4l2_buffer buf;
 unsigned int i;
 char msgtxt[1024];

  switch (io) {
        case IO_METHOD_READ:
                if(-1 == read(fd, buffers[0].start, buffers[0].length)){
                   switch (errno) {
                        case EAGAIN: return 0; // Need to try again
                        case EIO: // Could ignore EIO, see spec. Fall through.
                        default:
                            sprintf(msgtxt,"%s error %d, %s", "read", errno, strerror(errno));
                            show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                            return GRAB_ERR_READIO; 
                    }
                }
                if(skipframe==skiplim)
                 process_image(buffers[0].start, buffers[0].length);
                break;

        case IO_METHOD_MMAP:
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf)){
                   switch (errno) {
                        case EAGAIN: return 0; // Need to try again
                        case EIO: // Could ignore EIO, see spec. Fall through.
                        default:
                            sprintf(msgtxt,"%s error %d, %s", "VIDIOC_DQBUF", errno, strerror(errno));
                            show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                            return GRAB_ERR_MMAPD; 
                    }
                }
                assert(buf.index < n_buffers);
                if(skipframe==skiplim)
                 process_image(buffers[buf.index].start, buf.bytesused);
                if(-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
                   sprintf(msgtxt,"%s error %d, %s", "VIDIOC_QBUF", errno, strerror(errno));
                   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                   return GRAB_ERR_MMAPQ;
                  }
                break;

        case IO_METHOD_USERPTR:
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                   switch (errno) {
                        case EAGAIN: return 0; // Need to try again
                        case EIO: // Could ignore EIO, see spec. Fall through.
                        default:
                            sprintf(msgtxt,"%s error %d, %s.\nYou may need to quit the program, check the camera connection and re-start.", "VIDIOC_DQBUF", errno, strerror(errno));
                            show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                            // This error can happen if the camera is
                            // accidentally disconnected (e.g. the USB
                            // cable is dislodged by accident). In this
                            // case the user cannot recover and if live
                            // preview is happening this error popup
                            // will just continue endlessly with every
                            // attempt to read a preview image. So, to
                            // give the user the change to exit
                            // gracefully, switch off previewing if it
                            // is active:
                            Need_to_preview=PREVIEW_OFF;
                            gtk_label_set_text(GTK_LABEL(Label_preview)," Preview is OFF ");
                            gtk_widget_show(Ebox_lab_preview);
                            
                            return GRAB_ERR_USERPD; 
                    }
                }

                for(i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start && buf.length == buffers[i].length) break;
                assert(i < n_buffers);
                if(skipframe==skiplim)
                 process_image((void *)buf.m.userptr, buf.bytesused);
                if(-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
                   sprintf(msgtxt,"%s error %d, %s.\nYou may need to quit the program, check the camera connection and re-start.", "VIDIOC_QBUF", errno, strerror(errno));
                   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                    // This error can happen if the camera is
                    // accidentally disconnected (e.g. the USB
                    // cable is dislodged by accident). In this
                    // case the user cannot recover and if live
                    // preview is happening this error popup
                    // will just continue endlessly with every
                    // attempt to read a preview image. So, to
                    // give the user the change to exit
                    // gracefully, switch off previewing if it
                    // is active:
                    Need_to_preview=PREVIEW_OFF;
                    gtk_label_set_text(GTK_LABEL(Label_preview)," Preview is OFF ");
                    gtk_widget_show(Ebox_lab_preview);
                   return GRAB_ERR_USERPQ;
                  }
                break;
    }
  
 return GRAB_ERR_NONE; 
}

static int grab_image(void)
{
 int returnval,tmp_av_denom;
 char imsg[64];
 
 if(image_being_grabbed) return GRAB_ERR_BUSY;
 
 // This should have been checked by the calling function but just in
 // case ...
 if(!camera_status.cs_streaming) return GRAB_ERR_NOSTREAM; 

 // In this function we return error codes if something goes wrong and
 // avoid pop up interactive dialogue boxes because this could be part 
 // of a multicapture auto process (like preview streaming) and it would
 // be annoying to have lots of pop-up messages. Let the function caller
 // decide what to do on receipt of an error code.
 
 image_being_grabbed = 1; // We mark the grabber as busy now ...
 
 // If this grab operation is called from the live preview timeout
 // function then we ignore any averaging and frame skips. Also we don't
 // want to hang the GUI waiting for a frame for the live preview so
 // give up after 100 microseconds if no frame could be grabbed in that
 // time. We also ignore any request for multi-frame averaging.
 // These special measures are taken because we don't use a separate
 // thread for our live preview window:
 if(from_preview_timeout){ 
      skipframe=skiplim;  
      frame_timeout_sec  = 0;
      frame_timeout_usec = 100;
      goto just_the_one;   // Skip any averaging - just get one frame.
  } 
  
 Av_limit=Av_denom; // Set the frame averaging limit to the desired
                    // number of frames. If this is >1 it indicates we
                    // are going to do on a multi-frame average loop.
 // If we are going into a muti-frame average loop then activate the
 // 'Cancel averaging' button so the user can get out of it if they need
 // to:
 if(Av_limit>1){
    gtk_widget_show(btn_av_interrupt);
    // Update the GUI
    UPDATE_GUI
  }

 // Loop for multi-frame averaging ...
 for(Av_denom_idx=1;Av_denom_idx<=Av_limit;Av_denom_idx++){ 

  if(Av_limit>1 && Need_to_save){
     a_beep(25, 4);
     sprintf(imsg,"Accumulating frame: %d",Av_denom_idx);
     show_message(imsg,"FYI: ",MT_INFO,0);
   }
  for(skipframe=0;skipframe<=skiplim;skipframe++) // Loop for buffer clearing
     just_the_one:
     FOREVER {
         fd_set fds;
         struct timeval tv;
         int r;
    
         FD_ZERO(&fds);
         FD_SET(fd, &fds);

         // Seconds and microseconds to wait before giving a timout
         // error signal
         tv.tv_sec = frame_timeout_sec;   
         tv.tv_usec = frame_timeout_usec; 
    
         r = select(fd + 1, &fds, NULL, NULL, &tv);
    
         if(-1 == r){
            //if (EINTR == errno) continue;
            returnval = GRAB_ERR_SELECT; // Error selecting frame
            goto end_of;
           }
    
         // Timeout error
         if (0 == r) {returnval = GRAB_ERR_TIMEOUT; goto end_of;} 

         returnval = read_frame();

         if (returnval)  break; // This could be success or an error
                                // sent from read_frame(). Either way
                                // exit the forever loop and return it.
        }

    // Keep the GUI updated (so it does not freeze and be unresponsive)
    // if we are doing multi-frame averaging so the user can have a 
    // chance to click the 'Cancel averaging' button:
    if(Av_limit>1) UPDATE_GUI

    if(from_preview_timeout) break;
    // If user cancelled averaging, force the next capture to be the
    // last (unless we are already at the last).
    tmp_av_denom=Av_denom_idx;
    if(From_av_cancel==1){
       if(Av_denom_idx<Av_limit){
         Av_limit=tmp_av_denom;
         Av_denom_idx=Av_limit-1;
        }
       From_av_cancel=2;
      }

  }
 
end_of:
 Av_limit=0; // Reset the averaging flag (in case it was used).
 // Hide the 'Cancel averaging' button if it was shown:
 if(Av_denom>1){
    gtk_widget_hide(btn_av_interrupt);
    // Update the GUI
    UPDATE_GUI
  }
 if(From_av_cancel){
   sprintf(imsg,"CANCELLED Multiframe averaging at %d frames.",tmp_av_denom);
   show_message(imsg,"FYI: ",MT_INFO,0);
   From_av_cancel=0;
  }  
 
 image_being_grabbed = 0; // Not busy any more.
 from_preview_timeout = 0; // Preview just gets one shot.
 return returnval;
}

static int stop_streaming(void)
{
 enum v4l2_buf_type type;
 char msgtxt[1024];


 switch (io) {
        case IO_METHOD_READ: break; // Nothing to do.
                break;
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
                   if(errno == 9) break; // Stream already stopped
                   sprintf(msgtxt,"%s error %d, %s", "VIDIOC_STREAMOFF", errno, strerror(errno));
                   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                   return 1;
                  }
                break;
        }
 change_cam_status(CS_STREAMING,0);
 return 0;  
}

static int start_streaming(void)
{
 unsigned int i;
 enum v4l2_buf_type type;
 char msgtxt[1024];

 switch (io) {
        case IO_METHOD_READ: break; // Nothing to do.
        case IO_METHOD_MMAP:
                for(i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;
                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
                           sprintf(msgtxt,"%s error %d, %s", "VIDIOC_QBUF", errno, strerror(errno));
                           show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                           return 1;
                          }
                   }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if(-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
                   sprintf(msgtxt,"%s error %d, %s", "VIDIOC_STREAMON", errno, strerror(errno));
                   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                   return 1;
                  }
                break;

        case IO_METHOD_USERPTR:
                for(i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;
                        if(-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
                           sprintf(msgtxt,"%s error %d, %s", "VIDIOC_QBUF", errno, strerror(errno));
                           show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                           return 1;
                          }
                   }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if(-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
                   sprintf(msgtxt,"%s error %d, %s", "VIDIOC_STREAMON", errno, strerror(errno));
                   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                   return 1;
                  }
                break;
        }
 change_cam_status(CS_STREAMING,1);
 return 0;  
}

static int uninit_device(void)
{
 unsigned int i;
 char msgtxt[1024];

 switch (io) {
    case IO_METHOD_READ:
      free(buffers[0].start);
     break;
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length)){ // This is messy. The user should save their work and re-start.
          sprintf(msgtxt,"%s error %d, %s\nYou should save your work\nand re-start the program.", "munmap", errno, strerror(errno));
          show_message(msgtxt,"Camera Error: ",MT_ERR,1);
          free(buffers);
          return 1;
         }
     break;
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) free(buffers[i].start);
     break;
  }
 free(buffers);
 
 change_cam_status(CS_INITIALISED,0);
 return 0;
}

static int init_read(unsigned int buffer_size)
{
 buffers = calloc(1, sizeof(*buffers));
 if(!buffers){
     show_message("Memory allocation failed\non Read i/o video buffers.","Camera Error: ",MT_ERR,1);
     return 1;
   }
 buffers[0].length = buffer_size;
 buffers[0].start = malloc(buffer_size);

 if(!buffers[0].start){
     show_message("Memory allocation failed\non Read buffer[0].","Camera Error: ",MT_ERR,1);
     free(buffers);
     return 1;
   }
 return 0;
}

static int init_mmap(void)
{
 struct v4l2_requestbuffers req;
 char msgtxt[1024];

 CLEAR(req);
 req.count = 4;
 req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 req.memory = V4L2_MEMORY_MMAP;
 if(-1 == xioctl(fd, VIDIOC_REQBUFS, &req)){
   if(EINVAL == errno) {
     sprintf(msgtxt,"%s does not support user memory mapping.", dev_name);
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    } else {
     sprintf(msgtxt,"%s error %d, %s", "VIDIOC_REQBUFS", errno, strerror(errno));
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    }
   }
 if(req.count < 2){
     sprintf(msgtxt,"Insufficient MMAP buffer memory on %s",dev_name);
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
   }

 buffers = calloc(req.count, sizeof(*buffers));
 if(!buffers) {
     sprintf(msgtxt,"Memory allocation failed\nfor video MMAP buffers array");
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    }

 for(n_buffers = 0; n_buffers < req.count; ++n_buffers){
     struct v4l2_buffer buf;
     CLEAR(buf);
     buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     buf.memory      = V4L2_MEMORY_MMAP;
     buf.index       = n_buffers;

     if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){ // Because complete memory cleanup may not be possible, advise the user to re-start
        sprintf(msgtxt,"%s error %d, %s\nYou should save your work\nand re-start the program.", "VIDIOC_QUERYBUF", errno, strerror(errno));
        show_message(msgtxt,"Camera Error: ",MT_ERR,1);
        free(buffers);
        return 1;
       }

     buffers[n_buffers].length = buf.length;
     buffers[n_buffers].start = mmap(NULL,buf.length,PROT_READ | PROT_WRITE,MAP_SHARED,fd, buf.m.offset);

     if (MAP_FAILED == buffers[n_buffers].start){
        sprintf(msgtxt,"%s error %d, %s\nYou should save your work\nand re-start the program.", "mmap", errno, strerror(errno));
        show_message(msgtxt,"Camera Error: ",MT_ERR,1);
        free(buffers);
        return 1;
       }
   }
   
 return 0;
}

static int init_userp(unsigned int buffer_size)
{
 struct v4l2_requestbuffers req;
 char msgtxt[1024];

 CLEAR(req);
 req.count  = 4;
 req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 req.memory = V4L2_MEMORY_USERPTR;
 if(-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
   if(EINVAL == errno) {
     sprintf(msgtxt,"%s does not support user pointer i/o.", dev_name);
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    } else {
     sprintf(msgtxt,"%s error %d, %s", "VIDIOC_REQBUFS", errno, strerror(errno));
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    }
  }
 buffers = calloc(4, sizeof(*buffers));
 if(!buffers) {
     sprintf(msgtxt,"Memory allocation failed\nfor video userptr buffers array");
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    }
 for(n_buffers = 0; n_buffers < 4; ++n_buffers){
     buffers[n_buffers].length = buffer_size;
     buffers[n_buffers].start = malloc(buffer_size);
     if(!buffers[n_buffers].start) {
        sprintf(msgtxt,"Memory allocation failed\nfor video userptr buffer[%d]",n_buffers);
        show_message(msgtxt,"Camera Error: ",MT_ERR,1);
        switch(n_buffers){
            case 3:
             free(buffers[2].start);free(buffers[1].start);free(buffers[0].start);
            break;
            case 2:
             free(buffers[1].start);free(buffers[0].start);
            break;
            case 1:
             free(buffers[0].start);
            break;
            default: break;
           }
        free(buffers);
        return 1;
       }
    }
 return 0;
}

static int init_device(void)
{
 struct v4l2_capability cap;
 struct v4l2_cropcap cropcap;
 struct v4l2_crop crop;
 struct v4l2_format fmt;
 unsigned int uinum;
 char msgtxt[1024];
 int returnval;
 int formats_exhausted = 0;

 if(-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
   if (EINVAL == errno) {
     sprintf(msgtxt,"%s is not a V4L2 device", dev_name);
     show_message(msgtxt,"Camera Error: ",MT_ERR,1);
     return 1;
    } else {
            sprintf(msgtxt,"%s error %d, %s", "VIDIOC_QUERYCAP", errno, strerror(errno));
            show_message(msgtxt,"Camera Error: ",MT_ERR,1);
            return 1;
           }
  }

 if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
   sprintf(msgtxt,"%s is not a video capture device", dev_name);
   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
   return 1;
  }

 switch (io) {
        case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                  sprintf(msgtxt,"%s does not support read i/o", dev_name);
                  show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                  return 1;
                }
         break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                  sprintf(msgtxt,"%s does not support streaming i/o", dev_name);
                  show_message(msgtxt,"Camera Error: ",MT_ERR,1);
                  return 1;
                }
                break;
        }


 // Select video input, video standard and tune here.
 CLEAR(cropcap);
 cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 if(0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)){
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default rectangle
    if(-1 == xioctl(fd, VIDIOC_S_CROP, &crop)){
       switch (errno){
                      case EINVAL: break; // Cropping not supported.
                      default: break;     // Errors ignored. 
                     }
      }
   } else {// Errors ignored.
          }
          
 // Set video stream pixel and buffer format paramters
 // Initialise the format structure with the original settings (as set by v4l2-ctl for example)

 CLEAR(fmt);
 fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 if(-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)){
    sprintf(msgtxt,"%s error %d, %s", "VIDIOC_G_FMT", errno, strerror(errno));
    show_message(msgtxt,"Camera Error: ",MT_ERR,1);
    return 1;
   }
 // Now modify those settings that the user is given control over in
 // this program via the GUI to the current user choices (or program
 // defaults if user has not yet modified them)
 fmt.fmt.pix.width       = ImWidth;
 fmt.fmt.pix.height      = ImHeight;
 fmt.fmt.pix.field = V4L2_FIELD_ANY; // Let the driver decide about field interlacing

 // Now test whether either or both of the YUYV and MJPEG camera formats
 // are not supported:
 show_message("Testing for YUYV format support ...","FYI: ",MT_INFO,0);
 fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
 if(-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
    sprintf(msgtxt,"%s error %d, %s", "VIDIOC_S_FMT", errno, strerror(errno));
    show_message(msgtxt,"Camera Error: ",MT_ERR,1);
    return 1;
   }
 if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV){
   show_message("The camera driver does not support YUYV format.","FYI: ",MT_INFO,0);
   FormatForbidden=CAF_YUYV;
   formats_exhausted++;
  } else show_message("YUYV support is OK.","FYI: ",MT_INFO,0);
 show_message("Testing for MJPEG format support ...","FYI: ",MT_INFO,0);
 fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
 if(-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
    sprintf(msgtxt,"%s error %d, %s", "VIDIOC_S_FMT", errno, strerror(errno));
    show_message(msgtxt,"Camera Error: ",MT_ERR,1);
    return 1;
   }
 if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG){
   show_message("The camera driver does not support MJPEG format.","FYI: ",MT_INFO,0);
   FormatForbidden=CAF_MJPEG;
   formats_exhausted++;
  } else show_message("MJPEG support is OK.","FYI: ",MT_INFO,0);
 if(formats_exhausted==2){
   FormatForbidden=CAF_ALLBAD;
   show_message("The Camera driver does not support YUYV or MJPEG format.\nPlease save your work and restart the program with a different camera.","Camera Error: ",MT_ERR,1);
   return 1; 
 }


 switch(CamFormat){
    case V4L2_PIX_FMT_YUYV:
        if(FormatForbidden==CAF_YUYV){
          show_message("YUYV not possible. Re-seting image output to MJPEG.","FYI: ",MT_INFO,0);
           fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
           gtk_combo_box_set_active (GTK_COMBO_BOX (combo_camfmt), 1);
           CamFormat=V4L2_PIX_FMT_MJPEG;
          } else {
          show_message("Seting image output to YUYV.","FYI: ",MT_INFO,0);
          fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
          }
     break;
    case V4L2_PIX_FMT_MJPEG:
        if(FormatForbidden==CAF_MJPEG){
           show_message("MJPEG not possible. Re-seting image output to YUYV.","FYI: ",MT_INFO,0);
           fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
           gtk_combo_box_set_active (GTK_COMBO_BOX (combo_camfmt), 0);
           CamFormat=V4L2_PIX_FMT_YUYV;
          } else {
           show_message("Setting image output to MJPEG.","FYI: ",MT_INFO,0);
           fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
          }
     break;
    default: // This should not happen unless this program (or a modification thereof) is buggy
        show_message("Invalid camera image format selected","Program Error: ",MT_ERR,1);
        return 1;
     break;
   }
 // Now to apply these settings to the capture device. Note VIDIOC_S_FMT may change width and height.
 if(-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
    sprintf(msgtxt,"%s error %d, %s", "VIDIOC_S_FMT", errno, strerror(errno));
    show_message(msgtxt,"Camera Error: ",MT_ERR,1);
    return 1;
   }
   
 // Now check we got what we asked for in case there was some 
 // incompatibility in the driver or firmware interface to the camera
 uinum=fmt.fmt.pix.width;
 if(uinum!=(unsigned int)ImWidth){
    show_message("The Camera driver failed to set your selected image width.\nPlease save your work and restart the program.","Camera Error: ",MT_ERR,1);
    return 1;  
   }
 uinum=fmt.fmt.pix.height;
 if(uinum!=(unsigned int)ImHeight){
    show_message("The Camera driver failed to set your selected image height.\nPlease save your work and restart the program.","Camera Error: ",MT_ERR,1);
    return 1;  
   }

 switch(CamFormat){
    case V4L2_PIX_FMT_YUYV:
        if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV){
               show_message("The Camera driver does not support YUYV or MJPEG format.\nPlease save your work and restart the program with a different camera.","Camera Error: ",MT_ERR,1);
               FormatForbidden=CAF_ALLBAD;
               return 1;  
              }
     break;
    case V4L2_PIX_FMT_MJPEG:
        if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG){
               show_message("The Camera driver does not support MJPEG or YUYV format.\nPlease save your work and restart the program with a different camera.","Camera Error: ",MT_ERR,1);
               FormatForbidden=CAF_ALLBAD;
               return 1;  
              }
     break;
    default: // We can't get see due to prior switch
     break;
   }

 // Ensure the driver has not messed up - try to avoid a seg fault.
 uinum = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
 if (fmt.fmt.pix.sizeimage < uinum) fmt.fmt.pix.sizeimage = uinum;

 sprintf(msgtxt,"Driver sets frame WxH to %d x %d (requested %d x %d)",fmt.fmt.pix.width,fmt.fmt.pix.height,ImWidth,ImHeight);
 show_message(msgtxt,"FYI: ",MT_INFO,0);

 returnval=0;
 switch (io) {
    case IO_METHOD_READ:
         returnval=init_read(fmt.fmt.pix.sizeimage);
     break;
    case IO_METHOD_MMAP:
         returnval=init_mmap();
     break;
    case IO_METHOD_USERPTR:
         returnval=init_userp(fmt.fmt.pix.sizeimage);
     break;
     default: break;
   }

 if(returnval==0) change_cam_status(CS_INITIALISED,1);   
 return returnval;
}

static int close_device(void)
{
 char msgtxt[1024];
 
 if (-1 == close(fd)){
   sprintf(msgtxt,"%s error %d, %s", "close", errno, strerror(errno));
   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
   return 1;
 }
 fd = -1;
 change_cam_status(CS_OPENED,0);
 return 0;
}

static int open_device(void)
{
 struct stat st;
 char msgtxt[1024];

 if(-1 == stat(dev_name, &st)) {
   sprintf(msgtxt,"Cannot identify '%s':\n%d, %s", dev_name, errno, strerror(errno));
   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
   return 1;
  }

 if(!S_ISCHR(st.st_mode)) {
   sprintf(msgtxt,"%s is not a device.", dev_name);
   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
   return 1;
  }

 fd = open(dev_name, O_RDWR | O_NONBLOCK, 0); // O_RDWR is required 

 if(-1 == fd){
   sprintf(msgtxt,"Cannot open '%s':\n%d, %s",dev_name, errno, strerror(errno));
   show_message(msgtxt,"Camera Error: ",MT_ERR,1);
   return 1;
  }
  
  change_cam_status(CS_OPENED,1);
  return 0;
}

int try_running_camera(void)
{
 if(!camera_status.cs_opened)      if(open_device()) return 1;
 if(!camera_status.cs_initialised) if(init_device()) return 2;
 if(!camera_status.cs_streaming)   if(start_streaming()) return 3;

 return 0;
}

static int re_init_device(void)
{
  if(camera_status.cs_opened){
  show_message("> Closing camera connection.","",MT_INFO,0);
  if(camera_status.cs_streaming) stop_streaming();
  if(camera_status.cs_initialised) uninit_device();
  close_device();
 }
 return try_running_camera(); 
}

void tidy_up(void) 
// Release resources prior to ending the program.
{
 char msgtxt[128];
 int idx;

 show_message("\nTidying up.","",MT_INFO,0);
 
 if(camera_status.cs_opened){
  show_message("> Closing camera connection.","",MT_INFO,0);
  if(camera_status.cs_streaming) stop_streaming();
  if(camera_status.cs_initialised) uninit_device();
  close_device();
 }
 if(ImRoot!=NULL){
   show_message("> Freeing image file name.","",MT_INFO,0);
   free(ImRoot);
  }
 if(FFFile!=NULL){
   show_message("> Freeing flat field correction image name.","",MT_INFO,0);
   free(FFFile);
  }
 if(DFFile!=NULL){
   show_message("> Freeing dark field correction image name.","",MT_INFO,0);
   free(DFFile);
  }
 if(CSFile!=NULL){
   show_message("> Freeing camera settings file name.","",MT_INFO,0);
   free(CSFile);
  }
 if(MaskFile!=NULL){
   show_message("> Freeing mask image file name.","",MT_INFO,0);
   free(MaskFile);
  }
 if(dev_name!=NULL){
   show_message("> Freeing device name.","",MT_INFO,0);
   free(dev_name);
  }
 if(SSrow!=NULL){
   show_message("> Freeing preview row sampler.","",MT_INFO,0);
   free(SSrow);
  }
 if(SScol!=NULL){
   show_message("> Freeing preview col sampler.","",MT_INFO,0);
   free(SScol);
  }
 if(PreviewImg!=NULL){
   show_message("> Freeing preview image.","",MT_INFO,0);
   free(PreviewImg);
  }
 if(Preview_dark!=NULL){
   show_message("> Freeing preview master dark.","",MT_INFO,0);
   free(Preview_dark);
  }
 if(Preview_flat!=NULL){
   show_message("> Freeing preview master flat.","",MT_INFO,0);
   free(Preview_flat);
  }
 if(PreviewRow!=NULL){
   show_message("> Freeing preview row buffer.","",MT_INFO,0);
   free(PreviewRow);
  }
 if(RGBimg!=NULL){
   show_message("> Freeing full-size image.","",MT_INFO,0);
   free(RGBimg);
  }
 if(FF_Image!=NULL){
   show_message("> Freeing flat field image.","",MT_INFO,0);
   free(FF_Image);
  }
 if(DF_Image!=NULL){
   show_message("> Freeing dark field image.","",MT_INFO,0);
   free(DF_Image);
  }
 if(MaskIm!=NULL){
   show_message("> Freeing mask image.","",MT_INFO,0);
   free(MaskIm);
  }
 if(PardIcon_ready){
   show_message("> Freeing PardIcon image.","",MT_INFO,0);
   if(PardIcon_data!=NULL) free(PardIcon_data);
  }
 if(CSlist!=NULL){
   show_message("> Freeing camera settings list.","",MT_INFO,0);
   cs_listfree();
 }
 if(luts_alloced){
   show_message("> Freeing colourspace conversion LUTs.","",MT_INFO,0);
   free(lut_yR);
   free(lut_yG);
   free(lut_yB);
   free(lut_crR);
   free(lut_crG);
   free(lut_cbG);
   free(lut_cbB);
   luts_alloced=0;
 }
 if(Audio_status!=AS_NULL){
   if( AudioUnInit() )
     sprintf(msgtxt,"> Audio error: Uninitialising failed\n");
   else sprintf(msgtxt,"> Terminating Audio.");
   show_message(msgtxt,"",MT_INFO,0);
  }


 show_message("> Freeing frame averaging accumultors.","",MT_INFO,0);
 free(Avr); free(Avg); free(Avb);
 show_message("> Freeing frame stores.","",MT_INFO,0);
 free(Frmr); free(Frmg); free(Frmb);
 show_message("> Freeing preview integration buffers.","",MT_INFO,0);
 for(idx=0;idx<PREVINTMAX;idx++) free(PreviewBuff[idx]);

 show_message("\nPARD Capture says: Bye!","",MT_INFO,0);
  
 return;    
}

void show_popup(char *msg,char *title)
// Show a modal popup window with a message
{
  gtk_window_set_title (GTK_WINDOW (dlg_info),title);
  gtk_message_dialog_set_markup(GTK_MESSAGE_DIALOG(dlg_info),msg);
  gtk_dialog_run(GTK_DIALOG(dlg_info));
}

void close_logfile(void)
// Closes log file (if open) and sets fplog to NULL.
{
    if (fplog) {
        fflush(fplog); fclose(fplog);
    }
    fplog = NULL;
}

void open_logfile(void)
// Opens log file for appending. The initial opening of the log file is
// done on program startup (where LogFilename is also set)
{
    if (Log_wanted) {
        if (fplog) close_logfile();
        fplog = fopen(LogFilename, "ab");
    } else fplog=NULL;
}

void show_message(char *msg,char *title,int mtype,int popup) 
// All written information to user is channelled through this function
{
 if(popup){ // User requests this to be a GUI popup message - but that
            // request should be denied in some circumstances ...
   if(!gui_up) popup=0; // ... such as if there is no GUI up and running
  } 
 open_logfile(); // if successful fplog will not be NULL
 if (fplog){
     fprintf(fplog,"%s%s\n",title,msg);
     close_logfile();
   }

 switch(mtype){
    case MT_INFO : // Equivalent of printf - usually for info only
        fprintf(stdout,"%s%s\n",title,msg);
        if(popup) show_popup(msg,title);
    break;
    case MT_ERR  : // Equivalent of fprintf(sderr, - to indicate an error
        fprintf(stderr,"%s%s\n",title,msg);
        if(popup) show_popup(msg,title);
    break;
    case MT_QUIT : // Message given upon program quit request.
        fprintf(stdout,"%s%s\n",title,msg);
        if(popup) show_popup(msg,title);
    goto end_program;
    default: // This really should not happen - only a programming error
             // can get here. It is a terminal event.
        fprintf(stderr,"Invalid message type received. This is a programming error. The program will terminate now.\n");
    goto end_program;
  } 

 return; 
 
 end_program:
   tidy_up();
   exit(1);
}

static void btn_cam_settings_click(GtkWidget *widget, gpointer data)
{
 char msg[64];
 int enumresult;

 if(Ser_active>0) return; // Don't allow changes during a series capture
 if(Av_limit>1) return; // Don't allow changes during an average capture

 // If none of the supported formats are provided by the camera, you
 // cannot allow settings to be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot apply settings because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }

 if(!camera_status.cs_opened){
    if(open_device()){
        show_message("Can't find a connected camera device to display settings for.","Notice!: ",MT_INFO,1);
         return;
     }
  }
 // Need this to check camera format compatibility
 if(!camera_status.cs_initialised) if(init_device()) return;
 // We check this again down here because the very first time this
 // button is clicked the forbidden flag will not be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot apply settings because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }
  
 enumresult=enumerate_camera_settings();
 if(enumresult!=CSE_SUCCESS){
      sprintf(msg,"Failed to get %s settings from camera (VIDIOC_QUERYCTRL, %d)",NCSs?"some":"any",enumresult);
      show_message(msg,"Error: ",MT_ERR,1);
      if(!NCSs) return; 
  }
  
 gtk_widget_show_all(win_cam_settings);
 gtk_widget_set_sensitive (widget,FALSE); 
 return;
}

static void toggled_audio(GtkWidget *toggle_button,gpointer user_data)
{

  if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(toggle_button))){
      Use_audio=AU_YES;
      a_beep(25, 5); a_beep(25, 5); a_beep(25, 5);
      show_message("Audio signals ENABLED","FYI: ",MT_INFO,0);
  } else {
      a_beep(125, 1);
      Use_audio=AU_NO;
      show_message("Audio signals DISABLED","FYI: ",MT_INFO,0);
  }
      Audio_sounding=0;
}

static void toggled_cam_preview(GtkWidget *toggle_button,gpointer user_data)
{
 // If none of the supported formats are provided by the camera, you
 // cannot allow settings to be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot preview because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }
  if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(toggle_button))){
      preview_only_once = 1;
      Need_to_preview=PREVIEW_ON;
      if(Preview_fullsize)
          gtk_label_set_text(GTK_LABEL(Label_preview)," Click to zoom ");

      if(!(Preview_fullsize && Preview_tile_selection_made==0))
         gtk_widget_hide(Ebox_lab_preview);
  } else {
      Need_to_preview=PREVIEW_OFF;
      gtk_label_set_text(GTK_LABEL(Label_preview)," Preview is OFF ");
      gtk_widget_show(Ebox_lab_preview);
  }
}

static void btn_cam_stream_click(GtkWidget *widget, gpointer data)
{
 if(Ser_active>0) return; // Don't allow changes during a series capture
 if(Av_limit>1) return; // Don't allow changes during an average capture
 // If none of the supported formats are provided by the camera, you
 // cannot allow settings to be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot start the image stream because the camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }

 if(camera_status.cs_streaming){
  stop_streaming();
 } else {
  if(try_running_camera()) return;
 }
 return;
}

void series_end_status(void)
// State the reason for terminating a capture series in a human-
// readable string for signing off the series log file
{
 // In case the function is called when not supposed to:
 if(Ser_active==0) return;
 
 // Otherwise print the status to the log file
 FPseries=fopen(Ser_logname,"ab");
 if(FPseries==NULL) return;  // Can't do anything if can't open file.
 
 fprintf(FPseries, "\nSeries terminated due to: ");

 switch(grab_report){
     case GRAB_ERR_NONE: // Success!
        // There are two possibilites for no grab error termination:
        // 1. Completions of the full series
        // 2. Cancellation of the series before completion
        if(Ser_cancel){
          fprintf(FPseries, "Cancellation with %d captures not done.\n", (Ser_Number - Ser_lastidx)-1);
        } else fprintf(FPseries, "Successful completion.\n");
      break;
     case GRAB_ERR_SELECT: // Error selecting a frame from the stream
        fprintf(FPseries,"Failed to get an image from the camera stream.");
       break;
     case GRAB_ERR_BUSY:   // Camera may be in use by another program
        fprintf(FPseries,"Frame grabber busy. Is the camera being used by another program?");
       break;
     case GRAB_ERR_TIMEOUT:// Took too long to retreive a frame from the stream
        fprintf(FPseries,"Camera taking too long to respond.");
       break;
     case GRAB_ERR_READIO: // Error reading frame via the read io method
         fprintf(FPseries,"Error reading frame via the read-io method.");
       break;
     case GRAB_ERR_MMAPD:  // MMAP DQBUFF Error
         fprintf(FPseries,"MMAP DQBUFF Error.");
       break;
     case GRAB_ERR_MMAPQ:  // MMAP QBUFF Error
         fprintf(FPseries,"MMAP QBUFF Error.");
       break;
     case GRAB_ERR_USERPD: // User pointer DQBUFF Error
         fprintf(FPseries,"User pointer DQBUFF Error.");
       break;
     case GRAB_ERR_USERPQ: // User pointer QBUFF Error
         fprintf(FPseries,"User pointer QBUFF Error.");
       break;
     case GRAB_ERR_NOSTREAM: // The camera is not streaming images -
                             // probably something stopped the stream
                             // during or just before capture. 
        fprintf(FPseries,"Camera stream has ceased");
       break;
    }
 fflush(FPseries);
 fclose(FPseries);
 
 return;
}

int grab_n_save(void) 
// Takes a picture and saves it using the local image capture device
// and user preferences
{
 int preview_tmp,returnvalue;
 int retry;
 
// Save preview image: Experimental / debugging
// raw_to_ppm("preview.ppm",PreviewHt, PreviewWd,(void *)PreviewImg);

 if(try_running_camera()) return GNS_ECAM;
 // If we've got this far we know the camera is ready to take pictures

 // Disable changing control values while grabbing images to save:
 gtk_widget_set_sensitive (btn_cs_apply,FALSE);
 gtk_widget_set_sensitive (ISlider,FALSE);
 
 // Initialise return value to all OK:
 returnvalue = GNS_OKIS;
  
 // Check if the user wants a flat field corrected image and see if it
 // is possible.
 // Use non-interactive messaging if this is being called from a script
 // (i.e. if remote_control==1) so as not to interrupt the script.
 do_ff_correction = DOFF_NO; // The default position is not to do it.
                             // Let's see if we can change that ...
 // If the user wants ff correction and a ff image is loaded ... 
 if(ffcorr_status == FFCORR_ON){ 
     switch(saveas_fmt){
         case SAF_BM8:  // ... and they have chosen 
         case SAF_YP5:  // a greyscale save as format ...
          if(fffile_loaded!=FFIMG_Y){ // ... but the flat field image is
                                      // NOT greyscale - so can't do it.
            show_message("Can't do flat field correction - image save as format (Y) does not match flat field image format.","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
          } else do_ff_correction = DOFF_Y; // ... and the flat field
                                            // image is greyscale - so
                                            // do it with Y-only. 
         break;
         case SAF_BMP:  // ... and they have chosen to capture a colour
         case SAF_PNG:  // image for saving (in either a colour format
         case SAF_RGB:  // or the intensity component of a colour format
         case SAF_JPG:  // even though that will be a monochrome image).
         case SAF_INT:
          if(fffile_loaded!=FFIMG_RGB){ // ... but the flat field image
                                        // is NOT colour so can't do it.
            show_message("Can't do flat field correction - image save as format (RGB) does not match flat field image format","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
          } else do_ff_correction = DOFF_RGB; // ... and the flat field
                                              // image is colour - so do
                                              // it with RGB. 
         break;
         case SAF_YUYV: // ... but YUYV does not support ff correction
                        // so they can't have it.
            show_message("Can't do flat field correction - image save as format does not currently support this function.","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
         break;
     }
 }

 // Check if the user wants a dark field corrected image and see if it
 // is possible.
 // Use non-interactive messaging if this is being called from a script
 // (i.e. if remote_control==1) so as not to interrupt the script.
 do_df_correction = DODF_NO; // The default position is not to do it.
                             // Let's see if we can change that ...
 // If the user wants df correction and a df image is loaded ... 
 if(dfcorr_status == DFCORR_ON){ 
     switch(saveas_fmt){
         case SAF_BM8:  // ... and they have chosen 
         case SAF_YP5:  // a greyscale save as format ...
          if(dffile_loaded!=DFIMG_Y){ // ... but the dark field image is
                                      // NOT greyscale - so can't do it.
            show_message("Can't do dark field correction - image save as format (Y) does not match dark field image format.","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
          } else do_df_correction = DODF_Y; // ... and the flat field
                                            // image is greyscale - so
                                            // do it with Y-only. 
         break;
         case SAF_BMP:  // ... and they have chosen to capture a colour
         case SAF_PNG:  // image for saving (in either a colour format
         case SAF_RGB:  // or the intensity component of a colour format
         case SAF_JPG:  // even though that will be a monochrome image).
         case SAF_INT:
          if(dffile_loaded!=DFIMG_RGB){ // ... but the dark field image
                                        // is NOT colour so can't do it.
            show_message("Can't do dark field correction - image save as format (RGB) does not match dark field image format","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
          } else do_df_correction = DODF_RGB; // ... and the flat field
                                              // image is colour - so do
                                              // it with RGB. 
         break;
         case SAF_YUYV: // ... but YUYV does not support dark field
                        // correction so they can't have it.
            show_message("Can't do dark field correction - image save as format does not currently support this function.","Warning: ",MT_ERR,1);
            returnvalue = GNS_OKNB;
         break;
     }
 }
 
 // We suspend any previewing during capture
 preview_tmp=Need_to_preview;
 if(Need_to_preview) Need_to_preview=PREVIEW_OFF; 

 // Try grabbing the image upto Gb_Retry number of attempts ...
 for(retry=0;retry<=Gb_Retry;retry++){
    Need_to_save = 1; // Tell grabber to write the frame it grabs to disk.
    frame_timeout_sec  = Gb_Timeout; // Time to wait before giving up on
    frame_timeout_usec = 0;          // getting a frame from the camera.
    grab_report=grab_image();
    if(grab_report!=GRAB_ERR_BUSY) break;
    UPDATE_GUI
   }
 
 Need_to_save = 0; // Tell grabber NOT to write further frames it grabs to disk.
 if(preview_tmp) Need_to_preview=PREVIEW_ON; // Restore preview mode if it was suspended

  switch(grab_report){
     case GRAB_ERR_NONE: // Success!
        returnvalue=GRAB_ERR_NONE;
      break;
     case GRAB_ERR_SELECT:      // Error selecting a frame from the stream
        show_message("Couldn't get image from stream.","Image Capture FAILED: ",MT_ERR,1);
        returnvalue = GNS_EGRB;
       break;
     case GRAB_ERR_BUSY:      // Live preview may be preventing image grabbing - try again (shouldn't happen)
        show_message("Grabber was busy - try disabling live preview and try again.","Image Capture FAILED: ",MT_ERR,1);
        returnvalue = GNS_EGRB;
       break;
     case GRAB_ERR_TIMEOUT:
        a_beep(20, 5);a_beep(20, 4);a_beep(20, 3);a_beep(20, 2);a_beep(20, 1);
        // Took too long to retreive a frame from the stream
        show_message("Camera taking too long to respond.","Image Capture FAILED: ",MT_ERR,1);
        returnvalue = GNS_EGRB;
       break;
     case GRAB_ERR_READIO: // Error reading frame via the read io method
     case GRAB_ERR_MMAPD:  // MMAP DQBUFF Error
     case GRAB_ERR_MMAPQ:  // MMAP QBUFF Error
     case GRAB_ERR_USERPD: // User pointer DQBUFF Error
     case GRAB_ERR_USERPQ: // User pointer QBUFF Error
        returnvalue = GNS_EGRB;
       break; // These all produce popup errors at source so no need for
              // another popup in each case.
     case GRAB_ERR_NOSTREAM: // The camera is not streaming images
       // - probably something stopped the stream during or just before
       // capture because the stream would have been OK at the beginning
       // of this function ('if(try_running_camera())' checks for that).
        show_message("Camera stream is off","Image Capture FAILED: ",MT_ERR,1);
        returnvalue = GNS_EGRB;
       break;
  }

 if(Ser_active){ // We are capturing a series
    time_t t1,t2;
     
    Ser_idx++; // Increment the counter
    // Write the log entry for last image captured
     FPseries=fopen(Ser_logname,"ab");
     if(FPseries!=NULL){
      fprintf(FPseries,"%d\t%g\t%s\n",Ser_lastidx+1,difftime(time(NULL),Ser_ts),Ser_name);
      fflush(FPseries); fclose(FPseries);
     }
    // Reset the clock
    t1=time(&Ser_ts);

    if(Ser_idx<Ser_Number){ // We need more captures but first ...
      Ser_lastidx=Ser_idx;
      // Test to see if the previous capture succeeded
      if(returnvalue==GRAB_ERR_NONE && !Ser_cancel){
        // Do another capture after the delay interval
        // Test for the passage of time. Wait till at least
        // Ser_Delay seconds have elapsed since the start of the
        // last capture but only if time is available:
         if(t1>=0 && Ser_Delay>0){
                 while((t2=time(&Ser_tc))>=0){
                     if(difftime(t2,t1)>=(double)Ser_Delay) break;
                     UPDATE_GUI
                 }
          }
          UPDATE_GUI
          // Start another image capture
          grab_n_save();
        } // Series ended successfully, return
      } // Series ended with error, return
  } 

 // Re-enable changing control values:
 gtk_widget_set_sensitive (btn_cs_apply,TRUE);
 gtk_widget_set_sensitive (ISlider,TRUE);


 return returnvalue;
}

static void btn_av_interrupt_click(GtkWidget *widget, gpointer data)
// User wants to terminate a multiframe average. Pressing this button
// during a series capture will only terminate the current multi-frame
// averge sequence - it will not terminate the series. To terminate the
// series (including any multiframe average in progress) use the 'Cancel
// series' button instead - see btn_cam_save_click().
{
 // Hide the cancel button and acknowledge the request
 gtk_widget_hide(btn_av_interrupt);
 show_message("CANCELLING Multiframe averaging ...","FYI: ",MT_INFO,0);
 UPDATE_GUI

 // Set a global flag to let the image capture function know to stop
 // averaging after the next capture (this flag will be reset to 0 by
 // the image capture function):
 From_av_cancel=1;

}

static void btn_cam_save_click(GtkWidget *widget, gpointer data)
{
 
 if(Ser_cancel) return; // This is a series capture that is in the
                        // process of cancellation so do not allow
                        // any action of this capture button till
                        // the cancel process is complete.

 // If none of the supported formats are provided by the camera, you
 // cannot allow images to be captured:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot save images because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }

 if(Delayed_start_on && Delayed_start_in_progress) 
  {
   Delayed_start_in_progress=0;
   return;
  }

 if(Ser_Number>1){ // This is a series capture
     gchar *markup;
     const char *format = "<span foreground=\"red\" weight=\"bold\">\%s</span>";
     GtkWidget *btnlabel;

     // Pressing this button during a series capture means the user is
     // trying to cancel the series - so do it:
     if(Ser_active){
       // Reset Cam Save button appearance
       show_message("Series capture CANCELLING ...","FYI: ",MT_INFO,0);
       gtk_button_set_label(GTK_BUTTON(widget),"Save Image");

       // If cancel a series while a multiframe average is taking
       // place, first cancel the multiframe average:
       if(Av_limit>1 && Accumulator_status==ACC_ALLOCED)
       btn_av_interrupt_click(widget,data);

       // Set the cancellation-in-progress flag
       Ser_cancel=1;
       // If we are not already at the last step, ensure cancellation
       // at the next step by setting the series index to the end.
       if(Ser_idx<Ser_Number){
           Ser_idx=Ser_Number-1;
       }
       return;
     } else { // Prepare to begin a series capture and start capturing
       // Change appearance of Cam Save button to 'CANCEL'
       markup = g_markup_printf_escaped (format, "CANCEL\nSeries");
       btnlabel = gtk_bin_get_child(GTK_BIN(widget));
       gtk_label_set_markup(GTK_LABEL(btnlabel), markup);
       g_free (markup);
       // Set Ser_active flag
       Ser_active=1;
       // Reset indices
       Ser_lastidx=Ser_idx=0;
       // Update the GUI
       UPDATE_GUI
       // Open the series log and register the start time
       sprintf(Ser_logname, "Series_%s.txt",ImRoot);
       FPseries=fopen(Ser_logname,"wb");
       if(FPseries==NULL){
          show_message("Failed to open the series log file.\nSeries capture will commence but without a log file.","Series FAILED: ",MT_ERR,1);
         } else {
          fprintf(FPseries, "Log for PARD Capture Series\n");
          fprintf(FPseries, "Start at: %s\n\n", ((time(&Ser_ts)) == -1) ? "[Time not available]" : ctime(&Ser_ts));
          fprintf(FPseries, "Index\tInterval\tImage\n");
          fflush(FPseries);  fclose(FPseries);
         } // Failure to log is not fatal to capturing a series.
       // Begin series capture. 
       show_message("Series capture START ...","FYI: ",MT_INFO,0);
     }
 } else if(Av_limit>1) {
 // Allow no further use of this button while multi-frame averaging is
 // going on. If the user tries, put a 'Busy averaging!' notice on the
 // button to let them know.
  gtk_button_set_label(GTK_BUTTON(widget),"Busy\naveraging!");
  return; 
 }

 // Check if we are in a countdown delay loop. If so, put up a modal
 // dialogue with the count down:
 if(Delayed_start_on){
   gchar *cdstr;
   time_t t1,t2;
   double elapsed;

    Delayed_start_in_progress=1;     
   // Disable changing control values while grabbing images to save:
   gtk_widget_set_sensitive (btn_cs_apply,FALSE);
   gtk_widget_set_sensitive (ISlider,FALSE);
   gtk_widget_set_sensitive (btn_cam_stream,FALSE);
   gtk_widget_set_sensitive (btn_cam_settings,FALSE);
   a_beep(25, 3); a_beep(25, 2); a_beep(25, 1);
   // Start the countdown loop:
     t1=time(NULL);
     // Abort is timing facilities are not available
     if(t1<0) goto abort_countdown;
      while((t2=time(NULL))>=0 && Delayed_start_in_progress==1){
            elapsed=difftime(t2,t1);
            if(elapsed>=Delayed_start_seconds) break;
            // Display current remaining time on the button:
            cdstr = g_strdup_printf("Starting in\n%.0f (s)",Delayed_start_seconds-elapsed);
            gtk_button_set_label(GTK_BUTTON(widget),cdstr);
            g_free (cdstr);
            UPDATE_GUI
          }
   
   abort_countdown:
   // Re-sensitise the 'Apply All Settings' button and slider
   gtk_widget_set_sensitive (btn_cs_apply,TRUE);
   gtk_widget_set_sensitive (ISlider,TRUE);
   gtk_widget_set_sensitive (btn_cam_stream,TRUE);
   gtk_widget_set_sensitive (btn_cam_settings,TRUE);
   // Reset the Save Image button label
   gtk_button_set_label(GTK_BUTTON(widget),"Save Image");
  // Don't progress to image capture if aborted and cancel any series
  // that may have just initialised
  if(Delayed_start_in_progress==1) Delayed_start_in_progress=0;
   else{
        if(Ser_active>0){
          // Reset Ser_active flag
          Ser_active=0;
          Ser_idx=0;
          Ser_cancel=0;
         }
        Delayed_start_in_progress=0;
        return;
       }

 }

 a_beep(25, 4); a_beep(25, 4);
 grab_n_save();
 // We don't use the return value from grab_n_save() when this is a
 // single interactive button call because an error message will have
 // popped up and/or appeared on the console if anything went wrong
 // during image capture (see grab_n_save()).

 
  if(Ser_active>0){ // This is the end of a series capture
     // Complete and close the series file.
     // Write the reason for termination and time of completion:
     series_end_status();
     FPseries=fopen(Ser_logname,"ab");
     if(FPseries!=NULL){
       fprintf(FPseries, "\nEnd at: %s\n", ((time(&Ser_ts)) == -1) ? "[Time not available]" : ctime(&Ser_ts));
       fflush(FPseries); fclose(FPseries);
      }
     // Reset Ser_active flag
     Ser_active=0;
     Ser_idx=0;
     show_message("Series capture ENDED","FYI: ",MT_INFO,0);
    // Re-enable Cam Save button:
    Ser_cancel=0;
  }

 // Reset Cam Save button appearance in case it was changed above
 gtk_button_set_label(GTK_BUTTON(widget),"Save Image");

 a_beep(50, 1); a_beep(50, 1); a_beep(50, 1);

 return;
}

static void btn_help_about_click(GtkWidget *widget, gpointer data)
{
// Create the About box
  About_dialog = gtk_about_dialog_new ();
  gtk_about_dialog_set_program_name (GTK_ABOUT_DIALOG(About_dialog),"PARD Capture (Stand Alone)");
  if(PardIcon_ready) gtk_about_dialog_set_logo (GTK_ABOUT_DIALOG(About_dialog),PardIcon_pixbuf);
  gtk_about_dialog_set_version (GTK_ABOUT_DIALOG(About_dialog),"v. 1.0.0");
  gtk_about_dialog_set_copyright (GTK_ABOUT_DIALOG(About_dialog),"Copyright © 2000-2022 Dr Paul J. Tadrous");
  gtk_about_dialog_set_comments (GTK_ABOUT_DIALOG(About_dialog),"Image capture for scientific applications. This version is optimised for OptArc cameras. This is an offshoot of the PARDUS robotic microscopy project.");
  gtk_window_set_title (GTK_WINDOW (About_dialog), "About PARD Capture");  
  gtk_about_dialog_set_website (GTK_ABOUT_DIALOG(About_dialog),"https://github.com/TadPath/PARDUS");  
  gtk_about_dialog_set_website_label (GTK_ABOUT_DIALOG(About_dialog),"Project GitHub page");  
  gtk_about_dialog_set_license (GTK_ABOUT_DIALOG(About_dialog),License_note);
 gtk_dialog_run(GTK_DIALOG(About_dialog));
 gtk_widget_destroy(About_dialog);
}

static gboolean img_preview_click(GtkWidget  *event_box,  GdkEventButton *event, gpointer data)
// Response to a user click inside the preview window. This will be for
// when the user is in 'Click to zoom' mode and wants to select a region
// of the preview to zoom in on (and conversely to zoom out if they are
// already zoomed in).
{ 
 char imsg[128];
 
 // Do nothing if not in preview tile select mode or if not previewing.
 if(Need_to_preview==PREVIEW_OFF) return TRUE;
 if(!Preview_fullsize) return TRUE; 
 if(Preview_tile_selection_made){
 // If we get here it means we are in full-size tile preview mode and
 // the user had already selected a tile then clicked again on the
 // preview window to re-set the preview to full-frame mode and allow
 // them to select another tile:
   Need_to_preview=PREVIEW_OFF; // Disable preview to apply the changes
    Preview_tile_selection_made=0;
    // Blank the current preview image because when we shrink down to
    // full frame view there may be unused areas (for wide format 16:9
    // video and these will be full of the previous zoomed image if we
    // don't reset the full preview image area:
    memset(PreviewImg, 127, PreviewImg_rgb_size*sizeof(unsigned char));
    gtk_widget_show(Ebox_lab_preview);
    calculate_preview_params(); // Select the tile into the preview image
   Need_to_preview=PREVIEW_ON;  // Re-enable preview.
   return TRUE;
  }

 // If we get here it means the user is selecting a tile position, so
 // set it:

 Prevclick_X=(int)event->x;
 Prevclick_Y=(int)event->y;

    // Centre the tile on the user's XY coordinates
    Img_startcol=(int)(Prev_scaledim*(double)Prevclick_X)-PreviewWd/2;
    Img_startrow=(int)(Prev_scaledim*(double)Prevclick_Y)-PreviewHt/2;
    // First do some checks (and adjustments if required) to the chosen
    // co-ordinates so as to avoid undershoot and overshoot of the tile:
    if((Img_startcol+PreviewWd)>ImWidth) Img_startcol=ImWidth-PreviewWd;
    if((Img_startrow+PreviewHt)>ImHeight) Img_startrow=ImHeight-PreviewHt;
    if(Img_startcol<0)  Img_startcol=0; 
    if(Img_startrow<0) Img_startrow=0;

 sprintf(imsg,"Preview tile selected at top left x,y = %d,%d", Img_startcol, Img_startrow); 
 show_message(imsg,"FYI: ",MT_INFO,0);

Need_to_preview=PREVIEW_OFF; // Disable preview to apply the changes
 Preview_tile_selection_made=1;
 gtk_widget_hide(Ebox_lab_preview);
 calculate_preview_params(); // Select the tile into the preview image
Need_to_preview=PREVIEW_ON;  // Re-enable preview.

 return TRUE;
}

static gboolean elp_preview_click(GtkWidget  *event_box,  GdkEventButton *event, gpointer data)
// This responds to a user click on the label inside the preview window.
// It is essentially the same as for 'img_preview_click(...)' but
// because the label is in the foreground it blocks any clicks getting
// through to that function to we need a function to respond if the
// user clicks over the label. This is it.
{ 
 char imsg[128];
 GtkAllocation allocation;

 // Do nothing if not in preview tile select mode or if not previewing.
 if(Need_to_preview==PREVIEW_OFF) return TRUE;
 if(!Preview_fullsize) return TRUE;

  
 if(Preview_tile_selection_made){
 // If we get here it means we are in full-size tile preview mode and
 // the user had already selected a tile then clicked again on the
 // preview window to re-set the preview to full-frame mode and allow
 // them to select another tile:
   Need_to_preview=PREVIEW_OFF; // Disable preview to apply the changes
    Preview_tile_selection_made=0;
    // Blank the current preview image because when we shrink down to
    // full frame view there may be unused areas (for wide format 16:9
    // video and these will be full of the previous zoomed image if we
    // don't reset the full preview image area:
    memset(PreviewImg, 127, PreviewImg_rgb_size*sizeof(unsigned char));
    gtk_widget_show(Ebox_lab_preview);
    calculate_preview_params(); // Select the tile into the preview image
   Need_to_preview=PREVIEW_ON;  // Re-enable preview.
   return TRUE;
  }

 // If we get here it means the user is selecting a tile position, so
 // set it:
 gtk_widget_get_allocation (event_box,&allocation);

 Prevclick_X=(PreviewWd/2)-(int)(allocation.width)/2+(int)event->x;
 Prevclick_Y=(PreviewHt/2)-(int)(allocation.height)/2+(int)event->y;

    // Centre the tile on the user's XY coordinates
    Img_startcol=(int)(Prev_scaledim*(double)Prevclick_X)-PreviewWd/2;
    Img_startrow=(int)(Prev_scaledim*(double)Prevclick_Y)-PreviewHt/2;
    // First do some checks (and adjustments if required) to the chosen
    // co-ordinates so as to avoid undershoot and overshoot of the tile:
    if((Img_startcol+PreviewWd)>ImWidth) Img_startcol=ImWidth-PreviewWd;
    if((Img_startrow+PreviewHt)>ImHeight) Img_startrow=ImHeight-PreviewHt;
    if(Img_startcol<0)  Img_startcol=0; 
    if(Img_startrow<0) Img_startrow=0;

 sprintf(imsg,"Preview tile selected at top left x,y = %d,%d", Img_startcol, Img_startrow); 
 show_message(imsg,"FYI: ",MT_INFO,0);

Need_to_preview=PREVIEW_OFF; // Disable preview to apply the changes
 Preview_tile_selection_made=1;
 gtk_widget_hide(Ebox_lab_preview);
 calculate_preview_params(); // Select the tile into the preview image
Need_to_preview=PREVIEW_ON;  // Re-enable preview.

 return TRUE;
}

static gboolean update_cam_preview(void)
// Timeout function to update the preview window image
{
    if(Need_to_save) return TRUE; // The save function will update the preview if needed so don't duplicate
    if(change_preview_fps){
     change_preview_fps=0;
     g_timeout_add(preview_fps, G_SOURCE_FUNC(update_cam_preview),NULL);
     return FALSE;
    }
    if(Need_to_preview==PREVIEW_ON && camera_status.cs_streaming){
         from_preview_timeout=1;
         grab_image(); // No need to check error - if it works, great. If not, carry on.
     }
 
 return TRUE;
}

int test_selected_df_filename(char *filename)
// Attempt to read the file header and see if it is suitable for use as
// a master dark for dark field correction. If successful, copy the
// file name into the global Selected_DF_filename variable and set the 
// df_pending flag and GUI widgets accordingly. 
// Return 0 on success, 1 on failure.
{
 char msgtxt[256];
 int imfmt,lht,lwd;
     
 // Read the image file header to get dimensions and file type
 if(read_qih_file(filename,&lht,&lwd, &imfmt)){
    show_message("Selected dark field image(s) cannot be loaded. No dark field correction can be done. Try selecting another file.","FAILED: ",MT_ERR,1);
    return 1;
   }  
    
 sprintf(Selected_DF_filename,"%s",filename);
 df_pending=1;
 gtk_widget_set_sensitive (chk_usedfcor,TRUE);
 gtk_widget_set_sensitive (CamsetWidget[windex_ud],TRUE);
 gtk_widget_set_sensitive (CamsetWidget[windex_ud2],TRUE);

 sprintf(msgtxt,"You selected dark field image: %s (%d)\nWill attempt to load and process it when you click 'Apply',",name_from_path(Selected_DF_filename),imfmt);
 show_message(msgtxt,"FYI: ",MT_INFO,1);
 return 0;
}

void nullify_preview_darkfield(void)
// Eject a loaded live preview master dark.
{
 int idx;
 if(PrevDark_Loaded==0) return;
 for(idx=0;idx<PreviewImg_size;idx++) Preview_dark[idx]=0.0;
 PrevDark_Loaded=0;
 PrevCorr_BtnStatus=PD_LOADD;
 gtk_button_set_label(GTK_BUTTON(preview_corr_button),"Load P.Dark");
 show_message("Preview dark field image has been nullified.","FYI: ",MT_INFO,1);
 return;
}

void nullify_preview_flatfield(void)
// Eject a loaded live preview master flat.
{
 int idx;
 if(PrevFlat_Loaded==0) return;
 for(idx=0;idx<PreviewImg_size;idx++) Preview_flat[idx]=1.0;
 PrevFlat_Loaded=0;
 PrevCorr_BtnStatus=PD_LOADF;
 gtk_button_set_label(GTK_BUTTON(preview_corr_button),"Load P.Flat");
 show_message("Preview flat field image has been nullified.","FYI: ",MT_INFO,1);
 return;
}

static void btn_io_prev_corrfield_click(GtkWidget *widget, gpointer data) 
// This attempts to load/save/eject a live preview master dark / flat.
{
 gint res;
 gint choice;
 GtkFileChooserAction fca_open = GTK_FILE_CHOOSER_ACTION_OPEN;
 GtkFileChooser *prevmaster_file_chooser;
 GtkFileFilter  *prevmaster_file_filter;
 gchar *btn_markup;
 char *loaded_df_format = "<span foreground=\"red\" weight=\"bold\">\%s</span>";
 char *loaded_ff_format = "<span foreground=\"green\" weight=\"bold\">\%s</span>";
 char *loaded_af_format = "<span foreground=\"blue\" weight=\"bold\">\%s</span>";
 char *loaded_nf_format = "<span foreground=\"black\" weight=\"normal\">\%s</span>";
 char *mfb_format;
 GtkWidget *btnlabel;
 int lfval=1;

 // Take action according to current status of button
 switch(PrevCorr_BtnStatus){
    case PD_LOADD:
    // Set up the 'file load' dialogue with the dark field image 
    // choosing settings:
    prevmaster_file_filter  = gtk_file_filter_new();
    gtk_file_filter_set_name (prevmaster_file_filter,"Preview Dark Field Images");
    gtk_file_filter_add_pattern (prevmaster_file_filter, "*.dou");
    load_file_dialog = gtk_file_chooser_dialog_new ("Load a Preview Dark Field Image",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);
                                    
    prevmaster_file_chooser = GTK_FILE_CHOOSER (load_file_dialog);
    gtk_file_chooser_add_filter (GTK_FILE_CHOOSER (prevmaster_file_chooser),prevmaster_file_filter);
    gtk_file_chooser_set_filter (GTK_FILE_CHOOSER (prevmaster_file_chooser),prevmaster_file_filter);
    // Run the 'file load' dialogue
    res = gtk_dialog_run (GTK_DIALOG (load_file_dialog));
    if (res == GTK_RESPONSE_ACCEPT)
     {
      gchar *filename;
      filename = gtk_file_chooser_get_filename (prevmaster_file_chooser);
      lfval=read_preview_master((char *)filename,1);
      if(lfval==0) show_message(filename,"P.Dark: ",MT_INFO,0);
      g_free(filename);
     }
     // Remove the filter from the file load dialogue and close it:
     gtk_file_chooser_remove_filter(prevmaster_file_chooser,prevmaster_file_filter);
     gtk_widget_destroy(load_file_dialog);
    break;
    case PD_LOADF:
    // Set up the 'file load' dialogue with the flat field image 
    // choosing settings:
    prevmaster_file_filter  = gtk_file_filter_new();
    gtk_file_filter_set_name (prevmaster_file_filter,"Preview Flat Field Images");
    gtk_file_filter_add_pattern (prevmaster_file_filter, "*.dou");
    load_file_dialog = gtk_file_chooser_dialog_new ("Load a Preview Flat Field Image",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);
                                    
    prevmaster_file_chooser = GTK_FILE_CHOOSER (load_file_dialog);
    gtk_file_chooser_add_filter (GTK_FILE_CHOOSER (prevmaster_file_chooser),prevmaster_file_filter);
    gtk_file_chooser_set_filter (GTK_FILE_CHOOSER (prevmaster_file_chooser),prevmaster_file_filter);
    // Run the 'file load' dialogue
    res = gtk_dialog_run (GTK_DIALOG (load_file_dialog));
    if (res == GTK_RESPONSE_ACCEPT)
     {
      gchar *filename;
      filename = gtk_file_chooser_get_filename (prevmaster_file_chooser);
      lfval=read_preview_master((char *)filename,2);
      if(lfval==0) show_message(filename,"P.Flat: ",MT_INFO,0);
      g_free(filename);
     }
     // Remove the filter from the file load dialogue and close it:
     gtk_file_chooser_remove_filter(prevmaster_file_chooser,prevmaster_file_filter);
     gtk_widget_destroy(load_file_dialog);
    break;
    case PD_EJECT:
    // Ask if sure they really want to eject the current preview master
    // dark / flat.
    // If yes, eject it. In either case, cycle the button action
    gtk_window_set_title (GTK_WINDOW (dlg_choice), "Eject Preview Master Correction");
    gtk_message_dialog_set_markup(GTK_MESSAGE_DIALOG(dlg_choice),"Do you really want to eject the preview master dark/flat?");
    choice = gtk_dialog_run(GTK_DIALOG(dlg_choice));
    switch (choice)
      {
       case GTK_RESPONSE_YES:
       nullify_preview_darkfield();
       nullify_preview_flatfield();
       PrevCorr_BtnStatus--;
       break;
       default:
       break;
     }
    break;
   }

 
  // Set the correct colour and font for the multifinction button text
  // according to what corrective image(s) are loaded:
  // red bold -> Only the master dark is loaded
  // green bold -> Only the master flat is loaded
  // blue bold -> both master flat and dark are loaded
  // no colour (black) and no bold -> None are loaded
  if(PrevDark_Loaded && PrevFlat_Loaded) mfb_format=loaded_af_format;
  else if(PrevDark_Loaded) mfb_format=loaded_df_format;
  else if(PrevFlat_Loaded) mfb_format=loaded_ff_format;
  else mfb_format=loaded_nf_format;

  // Cycle the action of the preview correction button
   PrevCorr_BtnStatus++;
   if(PrevCorr_BtnStatus>2) PrevCorr_BtnStatus = 0;

recycle:
   switch(PrevCorr_BtnStatus){
    case PD_LOADD:
      btn_markup = g_markup_printf_escaped (mfb_format, "Load P.Dark");
      btnlabel = gtk_bin_get_child(GTK_BIN(widget));
      gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
      g_free (btn_markup);
    break;
    case PD_LOADF:
      btn_markup = g_markup_printf_escaped (mfb_format, "Load P.Flat");
      btnlabel = gtk_bin_get_child(GTK_BIN(widget));
      gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
      g_free (btn_markup);
    break;
    case PD_EJECT:
      if(PrevDark_Loaded || PrevFlat_Loaded) {
        btn_markup = g_markup_printf_escaped (mfb_format, "Eject All");
        btnlabel = gtk_bin_get_child(GTK_BIN(widget));
        gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
        g_free (btn_markup);
       } else { // Can't eject if there is nothing loaded
         PrevCorr_BtnStatus=PD_LOADD;
         goto recycle;
       }
    break;
   }

 return;
}

static void btn_cs_load_darkfield_click(GtkWidget *widget, gpointer data) 
// This just gets the file name and checks the format - it doesn't load
// the image.
{
 gint res;
 GtkFileChooserAction fca_open = GTK_FILE_CHOOSER_ACTION_OPEN;
 GtkFileChooser *dfimg_load_chooser; // Dark field reference image
 GtkFileFilter  *dfimg_load_filter;

 df_pending=0;
 
 // Set up the 'file load' dialogue with the dark field image choosing
 // settings:
// Set up the dark field correction reference image file selection
// filter to be used with the file chooser dialog - we only show this
// when the user clicks the [Select] button in the camera settings
// window. 
 dfimg_load_filter  = gtk_file_filter_new();
 gtk_file_filter_set_name (dfimg_load_filter,"Dark Field Images");                                
 gtk_file_filter_add_pattern (dfimg_load_filter, "*.dou"); 

 load_file_dialog = gtk_file_chooser_dialog_new ("Load a Dark Field Image",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);
                                    
 dfimg_load_chooser = GTK_FILE_CHOOSER (load_file_dialog);
 gtk_file_chooser_add_filter (GTK_FILE_CHOOSER (dfimg_load_chooser),dfimg_load_filter);
 gtk_file_chooser_set_filter (GTK_FILE_CHOOSER (dfimg_load_chooser),dfimg_load_filter);
 // gtk_file_chooser_set_current_folder (dfimg_load_chooser,DFFile);
 
 // Run the 'file load' dialogue
 res = gtk_dialog_run (GTK_DIALOG (load_file_dialog));
 if (res == GTK_RESPONSE_ACCEPT)
  {
   gchar *filename;
   filename = gtk_file_chooser_get_filename (dfimg_load_chooser);
   test_selected_df_filename((char *)filename);
   g_free(filename);
  }

 // Remove the filter from the file load dialogue and close it:
 gtk_file_chooser_remove_filter(dfimg_load_chooser,dfimg_load_filter);
 gtk_widget_destroy(load_file_dialog);
 
 return;
}

void nullify_darkfield(void)
// Test if already nullified so return without doing anything. This is
// not just for efficiency but also to prevent un-necessary pop-ups and
// error messages when loading settings from a saved settings file.
{
 if(!strcmp(DFFile,"[None]")) return;
 
  resize_memblk((void **)&DF_Image,1,sizeof(unsigned char), "the dark field image");
  DFht=DFwd=0;
  sprintf(DFFile,"[None]");
  dffile_loaded = DFIMG_NONE;
  dfcorr_status = DFCORR_OFF;
  // GUI stuff - don't do this if the camera settings window is not
  // visible because the dynamically allocated GUI controls will not be
  // addressable and GTK errors will result. This situation may arise
  // during the running of a script if the camera settings window was
  // closed prior to running it.
  if(gtk_widget_is_visible(win_cam_settings)==TRUE){
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usedfcor),FALSE);
      gtk_widget_set_sensitive (chk_usedfcor,FALSE);
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_ud]),"No");
      gtk_widget_set_sensitive (CamsetWidget[windex_ud],FALSE);
      gtk_widget_set_sensitive (CamsetWidget[windex_ud2],FALSE);
      // Set label to show nullified filename
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rdfi]), DFFile);  
    }
  show_message("Any pre-existing dark field image has been nullified. Dark field subtraction is disabled till a new dark field image is loaded.","FYI: ",MT_INFO,1);
  return;
}

static int init_darkfield_image(void)
// Load and prepare any user-supplied dark-field image.
// Return 0 on success and 1 on failure
{
 int lht,lwd,imfmt,width_stride;
 size_t imsz,rgbimsz;
 char msgtxt[320];
 imfmt=0; 
     
 // Read the image file header to get dimensions and file type
 if(read_qih_file(Selected_DF_filename,&lht,&lwd, &imfmt)){
    show_message("Selected dark field image(s) cannot be loaded. No dark field correction can be done. Try selecting another file.","FAILED: ",MT_ERR,1);
    nullify_darkfield(); return 1;
   }

 // Check dimensions are identical to current main image
 if(lht!=Selected_Ht){
    show_message("Selected flat dark image is not the same height as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_darkfield(); return 1;
   }     
 if(lwd!=Selected_Wd){
    show_message("Selected dark field image is not the same width as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_darkfield(); return 1;
   }     

 // OK, now see if we can allocate memory for the dark field image
  switch(imfmt){
    case DFIMG_Y:
     width_stride=Selected_Wd;
    break;
    case DFIMG_RGB:
     width_stride=Selected_Wd*3;
    break;
    default: // This should not happen
        show_message("Unrecognised image format for dark field image.","Program Error: ",MT_ERR,1);
        nullify_darkfield();
        return 1;
    }
 imsz=(size_t)Selected_Ht*(size_t)width_stride; 
 rgbimsz=imsz;
 
 if(resize_memblk((void **)&DF_Image,rgbimsz,sizeof(double), "the dark field image"))
  {
   nullify_darkfield();
   return 1;
  }

 // We got memory, so now try reading the image into it
 sprintf(msgtxt,"There was a problem reading the chosen dark field file. Cannot proceed.");
 if(read_raw_doubles(Selected_DF_filename, &DF_Image,lht,lwd,imfmt)){
    show_message(msgtxt,"FAILED: ",MT_ERR,1);
    nullify_darkfield(); return 1;
   }

    
 // OK. Image is loaded correctly. Now we set the dimensions and GUI:
 DFht=lht; DFwd=lwd;
 sprintf(DFFile,"%s",Selected_DF_filename);
 sprintf(msgtxt,"Dark field correction image loaded: %s",DFFile);
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     dffile_loaded=imfmt;
     //set label to show filename
     gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rdfi]), name_from_path(DFFile));  
     gtk_widget_set_sensitive (chk_usedfcor,TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_ud],TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_ud2],TRUE);
  }
 
 return 0;
}

int test_selected_ff_filename(char *filename)
// Attempt to read the file header and see if it is suitable for use as
// a master flat for flat field correction. If successful, copy the
// file name into the global Selected_FF_filename variable and set the 
// ff_pending flag and GUI widgets accordingly. 
// Return 0 on success, 1 on failure.
{
 char msgtxt[256];
 int imfmt,lht,lwd;
 int16_t bitcount;
     
 // Read the image file header to get dimensions and file type
 imfmt=0;
 if(!get_pgm_header(filename, &lht,&lwd))     imfmt=SAF_YP5;   // It's a pgm greyscale image
 else if(!get_ppm_header(filename, &lht,&lwd))imfmt=SAF_RGB;   // It's a ppm colour image
 else if(!get_bmp_header(filename, &lht,&lwd,&bitcount)){// It's a bmp image:
        switch(bitcount){
            case 8: imfmt=SAF_BM8; break;                                     // greyscale bmp
            case 24:imfmt=SAF_BMP; break;                                     // colour    bmp  
            default:
             show_message("Selected flat field bmp image is not 8 or 24 bit (other bit depths are not supported). Select a different file.","FAILED: ",MT_ERR,1);
            return 1;
        }   
    } else {// It is not one of the allowed unsigned char formats so see if
            // it is a raw doubles flat field image:
         
            // Read the external qih file header to get dimensions and file type
            if(read_qih_file(filename,&lht,&lwd, &imfmt)){
              show_message("Selected flat field image(s) is not of an acceptable format. Select a different file.","FAILED: ",MT_ERR,1);
              return 1;
             }
           }  
     
    sprintf(Selected_FF_filename,"%s",filename);
    ff_pending=1;
    
    gtk_widget_set_sensitive (chk_useffcor,TRUE);
    gtk_widget_set_sensitive (CamsetWidget[windex_uf],TRUE);
    gtk_widget_set_sensitive (CamsetWidget[windex_uf2],TRUE);

    sprintf(msgtxt,"You selected flat field image: %s (%d)\nWill attempt to load and process it when you click 'Apply',",name_from_path(Selected_FF_filename),imfmt);
    show_message(msgtxt,"FYI: ",MT_INFO,1);
   return 0;
}

static void btn_cs_load_flatfield_click(GtkWidget *widget, gpointer data) 
// This just gets the file name and checks the format - it doesn't load the image
{
 gint res;
 GtkFileChooserAction fca_open = GTK_FILE_CHOOSER_ACTION_OPEN;
 GtkFileChooser *ffimg_load_chooser;
 GtkFileFilter  *ffimg_load_filter;
 
 ff_pending=0;
 
 // Set up the 'file load' dialogue with the flat field image choosing
 // settings:
 // Set up the flat field correction reference image file selection
 // filter to be used with the file chooser dialog - we only show this
 // when the user clicks the [Select] button in the camera settings
 // window. 
 ffimg_load_filter  = gtk_file_filter_new();
 gtk_file_filter_set_name (ffimg_load_filter,"Flat Field Images");
 gtk_file_filter_add_pattern (ffimg_load_filter, "*.pgm");                                     
 gtk_file_filter_add_pattern (ffimg_load_filter, "*.ppm");                                     
 gtk_file_filter_add_pattern (ffimg_load_filter, "*.bmp");                                     
 gtk_file_filter_add_pattern (ffimg_load_filter, "*.dou");                                     

 load_file_dialog = gtk_file_chooser_dialog_new ("Load a Flat Field Image",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);

 ffimg_load_chooser = GTK_FILE_CHOOSER (load_file_dialog);

 gtk_file_chooser_add_filter (GTK_FILE_CHOOSER (ffimg_load_chooser),ffimg_load_filter);
 gtk_file_chooser_set_filter (GTK_FILE_CHOOSER (ffimg_load_chooser),ffimg_load_filter);
 // gtk_file_chooser_set_current_folder (ffimg_load_chooser,FFFile);
 
 // Run the 'file load' dialogue
 res = gtk_dialog_run (GTK_DIALOG (ffimg_load_chooser));
 if (res == GTK_RESPONSE_ACCEPT)
  {
   gchar *filename;
   filename = gtk_file_chooser_get_filename (ffimg_load_chooser);
   test_selected_ff_filename((char *)filename);
   g_free(filename);
  }

 // Remove the filter from the file load dialogue and close it
 gtk_file_chooser_remove_filter(ffimg_load_chooser,ffimg_load_filter);
 gtk_widget_destroy(load_file_dialog);
 
 return;
}

void nullify_flatfield(void)
 // Test if already nullified so return without doing anything. This is
 // not just for efficiency but also to prevent un-necessary pop-ups and
 // error messages when loading settings from a saved settings file.
{
 if(!strcmp(FFFile,"[None]")) return;
 
  resize_memblk((void **)&FF_Image,1,sizeof(unsigned char), "the flat field image");
  FFht=FFwd=0;
  sprintf(FFFile,"[None]");
  fffile_loaded = FFIMG_NONE;
  ffcorr_status = FFCORR_OFF;
  // GUI stuff - don't do this if the camera settings window is not visible
  // because the dynamically allocated GUI controls will not be addressable
  // and GTK errors will result. This situation may arise during the running
  // of a script if the camera settings window was closed prior to running it.
  if(gtk_widget_is_visible(win_cam_settings)==TRUE){
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_useffcor),FALSE);
      gtk_widget_set_sensitive (chk_useffcor,FALSE);
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_uf]),"No");
      gtk_widget_set_sensitive (CamsetWidget[windex_uf],FALSE);
      gtk_widget_set_sensitive (CamsetWidget[windex_uf2],FALSE);
      // Set label to show nullified filename
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rffi]), FFFile);  
    }
  show_message("Any pre-existing flat field image has been nullified. Flat field correction is disabled till a new flat field image is loaded.","FYI: ",MT_INFO,1);
  return;
}

static int init_flatfield_image(int normalise_ff)
// Read and initialise any user-supplied flat field file.
// normalise_ff is a flag that tells the function whether to normalise
// the flat field image within the support of the mask (1) or not (0).
// This function is called twice when applying settings. The first time
// is to load an image and check its format and dimensions only (so is
// called with normalise_ff=0).
// The second time is after the user makes their selection of whether
// to apply a custom mask or not (and whether that custom mask is
// successfully loaded or fails with a full mask being substituted) so
// is called with normalise_ff=1).
{
 int lht,lwd,imfmt,idx,rgbpos,width_stride,rawdou;
 double mr,mg,mb,meanr,meang,meanb,dtmp;
 size_t imsz,rgbimsz,stdx;
 char msgtxt[320];
 unsigned char *tmploc,cref[1024];    
 int16_t bitcount;
 
 imfmt=0; 
 rawdou=0; // Is the FF image raw doubles format?

 
 // Read the image file header to get dimensions and file type
 if(!get_pgm_header(Selected_FF_filename, &lht,&lwd))     imfmt=SAF_YP5;   // It's a pgm greyscale image
 else if(!get_ppm_header(Selected_FF_filename, &lht,&lwd))imfmt=SAF_RGB;   // It's a ppm colour image
 else if(!get_bmp_header(Selected_FF_filename, &lht,&lwd,&bitcount)){      // It's a bmp image:
    switch(bitcount){
        case 8: imfmt=SAF_BM8; break;                                     // greyscale bmp
        case 24:imfmt=SAF_BMP; break;                                     // colour    bmp  
        default:
             show_message("Selected flat field bmp image is not 8 or 24 bit (other bit depths are not supported). Cannot proceed.","FAILED: ",MT_ERR,1);
        nullify_flatfield(); return 1;
      }   
 } else{ // It is not one of the allowed unsigned char formats so see if
         // it is a raw doubles flat field image:
         
    // Read the external qih file header to get dimensions and file type
    if(read_qih_file(Selected_FF_filename,&lht,&lwd, &imfmt)){
       show_message("Selected flat field image(s) is not of an acceptable format. No flat field correction can be done. Try selecting another file.","FAILED: ",MT_ERR,1);
       nullify_flatfield(); return 1;
      }
    rawdou=1; // Header checks out OK so flag this as a raw doubles FF
  }  


 // Check dimensions are identical to current main image
 if(lht!=Selected_Ht){
    show_message("Selected flat field image is not the same height as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_flatfield(); return 1;
   }     
 if(lwd!=Selected_Wd){
    show_message("Selected flat field image is not the same width as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_flatfield(); return 1;
   }     


 // OK, now see if we can allocate memory for the flat field image
 if(rawdou==0){
     
  switch(imfmt){
    case SAF_YP5:
    case SAF_BM8:
     width_stride=Selected_Wd;
    break;
    case SAF_RGB:
    case SAF_BMP:
     width_stride=Selected_Wd*3;
    break;
    default: // This should not happen
        show_message("Unrecognised u.char image format for flat field image.","Program Error: ",MT_ERR,1);
        nullify_flatfield();
        return 1;
    }
    
 } else {
     
  switch(imfmt){
    case FFIMG_Y:
     width_stride=Selected_Wd;
    break;
    case FFIMG_RGB:
     width_stride=Selected_Wd*3;
    break;
    default: // This should not happen
        show_message("Unrecognised doubles image format for flat field image.","Program Error: ",MT_ERR,1);
        nullify_flatfield();
        return 1;
    }
  } 
  
 imsz=(size_t)Selected_Ht*(size_t)width_stride; 
 rgbimsz=imsz;
 
if(rawdou==0){
     
 tmploc=(unsigned char *)calloc(rgbimsz,sizeof(unsigned char));
 if(tmploc==NULL){
     show_message("Failed to allocate memory to store flat field image. Cannot proceed to load it.","FAILED: ",MT_ERR,1);
     nullify_flatfield(); return 1;
    }
        
 // We got memory, so now try reading the image into it
 sprintf(msgtxt,"There was a problem reading the chosen flat field file (u.char). Cannot proceed.");
 switch(imfmt){
        case SAF_YP5: 
            if(get_pgm(Selected_FF_filename, &tmploc, &lht,&lwd)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_flatfield(); free(tmploc); return 1;
            }
        break;
        case SAF_BM8: 
            if(get_bmp(Selected_FF_filename, &tmploc, &lht, &lwd, cref)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_flatfield(); free(tmploc); return 1;
            }
        break;
        case SAF_RGB:
            if(get_ppm(Selected_FF_filename, &tmploc)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_flatfield(); free(tmploc); return 1;
            }
        break;
        case SAF_BMP:
            if(get_bmp(Selected_FF_filename, &tmploc, &lht, &lwd, cref)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_flatfield(); free(tmploc); return 1;
            }
        break;
        default: // This should not happen
         show_message("Unrecognised image format for flat field image.","Program Error: ",MT_ERR,1);
         nullify_flatfield(); free(tmploc); return 1;
    }
    
  // The image is now loaded into an unsigned char array but corrections
  // are done in doubles. So make a raw doubles array and copy over the
  // values:
  if(resize_memblk((void **)&FF_Image,rgbimsz,sizeof(double), "the flat field image (doubles)"))
   {
    nullify_flatfield();
    free(tmploc);
    return 1;
   }
  for(stdx=0;stdx<rgbimsz;stdx++) FF_Image[stdx]=(double)tmploc[stdx];
  // Now we are done with tmploc. Also we need to change flags to the
  // raw doubles version for the rest of this function. So ...
  free(tmploc);
  rawdou=1;
  switch(imfmt){
        case SAF_YP5: 
        case SAF_BM8: 
            imfmt=DFIMG_Y;
        break;
        default:
            imfmt=DFIMG_RGB;
        break;
    }  
    
 } else { // If the user wants to load a raw doubles flat field ...
     
 if(resize_memblk((void **)&FF_Image,rgbimsz,sizeof(double), "the flat field image (doubles)"))
  {
   nullify_flatfield();
   return 1;
  }

 // We got memory, so now try reading the image into it
 sprintf(msgtxt,"There was a problem reading the chosen flat field file. Cannot proceed.");
 if(read_raw_doubles(Selected_FF_filename, &FF_Image,lht,lwd,imfmt)){
    show_message(msgtxt,"FAILED: ",MT_ERR,1);
    nullify_flatfield(); return 1;
   }
     
 }
  
 if(normalise_ff){

 // If we have got to here then the flat field correction image is
 // loaded as an array of raw doubles in FF_Image, the rawdou flag is
 // set to 1 and the imfmt flag is set to DFIMG_Y or DFIMG_RGB.
  
 // Test if image can be used for flat field correction and set the
 // fffile_loaded flag appropriately. All testing and normalising is
 // done within the confines of the support mask. This function could
 // not have been called unless an updated support mask was already
 // and successfully generated (either from a user-supplied file or
 // an internally generated full-support mask) so it is safe to use
 // MaskIm[] to the current image size indices.
 
 // Prior to this point imsz and rgbimsz were set equal to each other.
 // Now we will be using the mask which is only Selected_Ht*Selected_Wd
 // and not 3 times that size, even if we have loaded a colour flat
 // field file. So we must set imsz to only a single frame size and use
 // that as the primary index for loops otherwise we will overshoot the
 // mask array and get a segmentation fault:
 imsz=(size_t)Selected_Ht*(size_t)Selected_Wd; 

 
 switch(imfmt){
    case DFIMG_Y: // Greyscale formats (i.e. single channel)
    
         // Test for maximum pixel value. No realistic flat field
         // correction image taken with a camera down a microscope
         // should have a maximum pixel value of <1 assuming grey level
         // readout is on an integer scale starting at 0.
         mr=-1.0; meanr=0;
         for(idx=0;idx<imsz;idx++){
             if(MaskIm[idx]>0){
               dtmp=FF_Image[idx];
               if(dtmp>mr) mr=dtmp;
               meanr+=dtmp; // Accumulator to calc. the mean.
              }
            }
         if(mr<0.5){
           show_message("Background image is loadable but not useable for flat field correction (no pixel is greater than 0).","FAILED: ",MT_ERR,1);
           nullify_flatfield(); return 1;
         }
         // Now normalise by dividing all pixel values by the mean
         meanr/=Mask_supp_size; // Mean value
         sprintf(msgtxt,"Flat field image mean Y = %g",meanr);
         show_message(msgtxt,"FYI: ",MT_INFO,0);
         for(idx=0;idx<imsz;idx++){
             if(MaskIm[idx]>0){
                  FF_Image[idx]/=meanr;
              }}
         // Mark the master flat field as being successfully loaded
         fffile_loaded=FFIMG_Y;
        break;
    case DFIMG_RGB: // Colour formats
         mr=-1.0;mg=-1.0;mb=-1.0;meanr=0.0;meang=0.0;meanb=0.0;
         for(idx=0,rgbpos=0;idx<imsz;idx++){
             if(MaskIm[idx]>0){
               dtmp=FF_Image[rgbpos];
               if(dtmp>mr) mr=dtmp;
               meanr+=dtmp; // Accumulator to calc. the mean R value.
               rgbpos++;
               dtmp=FF_Image[rgbpos];
               if(dtmp>mg) mg=dtmp;
               meang+=dtmp; // Accumulator to calc. the mean G value.
               rgbpos++;
               dtmp=FF_Image[rgbpos];
               if(dtmp>mb) mb=dtmp;
               meanb+=dtmp; // Accumulator to calc. the mean B value.
               rgbpos++;
             } else rgbpos+=3;
           }
         if(mr<0.5){
           show_message("Background image is loadable but not useable for flat field correction (no red pixel is greater than 0).","FAILED: ",MT_ERR,1);
           nullify_flatfield(); return 1;
         }
         if(mg<0.5){
           show_message("Background image is loadable but not useable for flat field correction (no green pixel is greater than 0).","FAILED: ",MT_ERR,1);
           nullify_flatfield(); return 1;
         }
         if(mb<0.5){
           show_message("Background image is loadable but not useable for flat field correction (no blue pixel is greater than 0).","FAILED: ",MT_ERR,1);
           nullify_flatfield(); return 1;
         }
         
         // Now normalise by dividing all pixel values by the mean
         meanr/=Mask_supp_size; // Mean value
         meang/=Mask_supp_size; // Mean value
         meanb/=Mask_supp_size; // Mean value
         sprintf(msgtxt,"Flat field image mean RGB = %g, %g, %g",meanr,meang,meanb);
         show_message(msgtxt,"FYI: ",MT_INFO,0);
         
         for(idx=0,rgbpos=0;idx<imsz;idx++){
             if(MaskIm[idx]>0){
               FF_Image[rgbpos++]/=meanr;
               FF_Image[rgbpos++]/=meang;
               FF_Image[rgbpos++]/=meanb;
              } else rgbpos+=3;
            }
         
         // Mark the master flat field as being successfully loaded  
         fffile_loaded=FFIMG_RGB;
        break;
        default: // This should not happen
         show_message("Unrecognised image format for flat field image.","Program Error: ",MT_ERR,1);
         nullify_flatfield(); return 1;
    }

 } else fffile_loaded=FFIMG_NORM; // This indicates that a flat field
                                  // image was loaded successfully but
                                  // is yet to be normalised.

 // OK. Image is loaded correctly and is useable. Now we set the
 // associated global variables and GUI labels, etc.
 FFht=lht; FFwd=lwd;
 sprintf(FFFile,"%s",Selected_FF_filename);
 sprintf(msgtxt,"Flat field correction image loaded: %s",FFFile);
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     // fffile_loaded was set in the above switch block.
     // Set label to show filename and other GUI controls/labels:
     gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rffi]),name_from_path(FFFile));  
     gtk_widget_set_sensitive (chk_useffcor,TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_uf],TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_uf2],TRUE);
  }
 
 return 0;
}

int test_selected_msk_filename(char *filename)
// Attempt to read the file header and see if it is suitable for use as
// a corrections maek. If successful, copy the file name into the global
// Selected_Mask_filename variable and set the msk_pending flag and GUI
// widgets accordingly. 
// Return 0 on success, 1 on failure.
{
 char msgtxt[256];
 int lht,lwd,imfmt;
 int16_t bitcount;
 
 // No useable mask selected
 if(!strcmp(filename,"[None]") || !strcmp(filename,"[Full]") || !strcmp(filename,"[UNDF]") || !strcmp(filename,"None.bmp")){
   msk_pending=0;
   if(!strcmp(filename,"None.bmp")) sprintf(Selected_Mask_filename,"[None]");
   gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usemskcor),FALSE);
   gtk_widget_set_sensitive (chk_usemskcor,FALSE);
   gtk_widget_set_sensitive (CamsetWidget[windex_um],FALSE);
   gtk_widget_set_sensitive (CamsetWidget[windex_um2],FALSE);
   return 1;
 }

    
 // Read the image file header to get dimensions and file type
 // Read the mask image file header to get dimensions and file type
 imfmt=SAF_YUYV; // I use SAF_YUYV just as a reference value to see if
                 // it changes (if it doesn't then no acceptable format
                 // was found).
 if(!get_pgm_header(filename, &lht,&lwd))     imfmt=SAF_YP5;  // PGM greyscale 
 else if(!get_ppm_header(filename, &lht,&lwd))imfmt=SAF_RGB;  // PPM colour
 else if(!get_bmp_header(filename, &lht,&lwd,&bitcount)){     // BMP
    switch(bitcount){
        case 8: imfmt=SAF_BM8; break;     // greyscale bmp
        case 24:imfmt=SAF_BMP; break;     // colour    bmp  
        default:
             show_message("Selected mask bmp image is not 8 or 24 bit (other bit depths are not supported). Cannot proceed.","FAILED: ",MT_ERR,1);
             return 1;
      }   
 } 
 if(imfmt==SAF_YUYV) { // It is not one of the allowed unsigned char formats
         show_message("Selected mask image is not of an acceptable format. No custom masking can be done. Try selecting another file.","FAILED: ",MT_ERR,1);
         return 1;
        }                 

    sprintf(Selected_Mask_filename,"%s",filename);
    msk_pending=1;
    gtk_widget_set_sensitive (chk_usemskcor,TRUE);
    gtk_widget_set_sensitive (CamsetWidget[windex_um],TRUE);
    gtk_widget_set_sensitive (CamsetWidget[windex_um2],TRUE);

    sprintf(msgtxt,"You selected mask image: %s\nWill attempt to load and process it when you click 'Apply',",name_from_path(Selected_Mask_filename));
    show_message(msgtxt,"FYI: ",MT_INFO,1);
   return 0;
}

static void btn_cs_load_mask_click(GtkWidget *widget, gpointer data) 
// This just gets the file name and checks the format - it doesn't load
// the image. Masks can only be of 8bpp BMP format.
{
 gint res;
 GtkFileChooserAction fca_open = GTK_FILE_CHOOSER_ACTION_OPEN;
 GtkFileChooser *mask_load_chooser;  // Mask image
 GtkFileFilter  *mask_load_filter;
 
 msk_pending=0;
 
 // Set up the 'mask load' dialogue with the mask image choosing
 // settings:
 // Set up the corrections mask reference image file selection
 // filter to be used with the file chooser dialog - we only show this
 // when the user clicks the [Select] button in the camera settings
 // window. 
 mask_load_filter  = gtk_file_filter_new();
 gtk_file_filter_set_name (mask_load_filter,"Mask Images");                                   
 gtk_file_filter_add_pattern (mask_load_filter, "*.pgm");                                     
 gtk_file_filter_add_pattern (mask_load_filter, "*.ppm");                                     
 gtk_file_filter_add_pattern (mask_load_filter, "*.bmp");                                     

 load_file_dialog = gtk_file_chooser_dialog_new ("Load a Mask Image",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);

 mask_load_chooser = GTK_FILE_CHOOSER (load_file_dialog);
 gtk_file_chooser_add_filter(GTK_FILE_CHOOSER (mask_load_chooser),mask_load_filter);
 gtk_file_chooser_set_filter(GTK_FILE_CHOOSER (mask_load_chooser),mask_load_filter);
 // gtk_file_chooser_set_current_folder (mask_load_chooser,MaskFile);
 
 // Run the 'file load' dialogue
 res = gtk_dialog_run (GTK_DIALOG (load_file_dialog));
 if (res == GTK_RESPONSE_ACCEPT)
  {
   gchar *filename;
   filename = gtk_file_chooser_get_filename (mask_load_chooser);
   test_selected_msk_filename((char *)filename);
   g_free(filename);
  }

 // Remove the mask filter from the file load dialogue and close it.
 gtk_file_chooser_remove_filter(mask_load_chooser,mask_load_filter);
 gtk_widget_destroy(load_file_dialog);
 
 return;
}

void set_mask_full_support(int ht, int wd)
// Set the mask to a full support (non-custom) mask
{
 size_t msize;

  MKht=ht;
  MKwd=wd;
  msize=(size_t)MKht*(size_t)MKwd;
  Mask_supp_size=(double)msize;

  if(resize_memblk((void **)&MaskIm,msize,sizeof(unsigned char), "the mask image")){
   show_message("Generic mask could not be set. No dark field or flat field correction can be done.","MASK FAILED: ",MT_ERR,1);
   mask_alloced=MASK_NO;
   mskfile_loaded = MASK_NONE;
   sprintf(MaskFile,"[None]");
  } else mask_alloced=MASK_YES;   
 // If mem alloced OK we can use the all-permissive mask.
 memset(MaskIm, 255, msize*sizeof(unsigned char));

 sprintf(MaskFile,"[Full]");
 mskfile_loaded = MASK_FULL;
 mask_status = 0;

  return;
}

void nullify_mask(void)
// Test if already nullified so return without doing anything. This is
// not just for efficiency but also to prevent un-necessary pop-ups and
// error messages when loading settings from a saved settings file.
{
 // Re-initialise the mask to be all 255 across the whole field using
 // Selected_Ht and Selected_Wd as dimensions:
 set_mask_full_support(Selected_Ht,Selected_Wd);

  // GUI stuff - don't do this if the camera settings window is not
  // visible because the dynamically allocated GUI controls will not be
  // addressable and GTK errors will result. This situation may arise
  // during the running of a script if the camera settings window was
  // closed prior to running it.
  if(gtk_widget_is_visible(win_cam_settings)==TRUE){
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usemskcor),FALSE);
      gtk_widget_set_sensitive (chk_usemskcor,FALSE);
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_um]),"No");
      gtk_widget_set_sensitive (CamsetWidget[windex_um],FALSE);
      gtk_widget_set_sensitive (CamsetWidget[windex_um2],FALSE);
      // Set label to show nullified filename
      gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rmski]), MaskFile);  
    }
  if(msk_pending) show_message("Any pre-existing mask image has been nullified. Custom masking is disabled till a new mask image is loaded.","FYI: ",MT_INFO,1);
  return;
}

void set_mask_pending(void)
{
 if(!strcmp(Selected_Mask_filename,"[None]") || !strcmp(Selected_Mask_filename,"[Full]") || !strcmp(Selected_Mask_filename,"[UNDF]")){
   msk_pending=0;
 } else msk_pending=1;
}

static int init_mask_image(void)
// Load any user-specified mask file and get it ready for use. Check the
// format and dimensions, etc.
// Return 0 on success and the loaded mask is present in MaskIm.
// Return 1 on failure and MaskIm is set to full support.
{
 int lht,lwd,imfmt,idx,rgbpos,width_stride;
 unsigned char *tmploc;
 double d1,d2,d3;
 int msupp;
 size_t imsz,rgbimsz;
 char msgtxt[320];
 unsigned char cref[1024];    
 int16_t bitcount;

 if(!strcmp(Selected_Mask_filename,"[None]") || !strcmp(Selected_Mask_filename,"[Full]") || !strcmp(Selected_Mask_filename,"[UNDF]")){
   msk_pending=0;
   if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usemskcor),FALSE);
     gtk_widget_set_sensitive (chk_usemskcor,FALSE);
     gtk_widget_set_sensitive (CamsetWidget[windex_um],FALSE);
     gtk_widget_set_sensitive (CamsetWidget[windex_um2],FALSE);
    }
   return 0;
 }

 // Read the mask image file header to get dimensions and file type
 if(!get_pgm_header(Selected_Mask_filename, &lht,&lwd))     imfmt=SAF_YP5;  // PGM greyscale 
 else if(!get_ppm_header(Selected_Mask_filename, &lht,&lwd))imfmt=SAF_RGB;  // PPM colour
 else if(!get_bmp_header(Selected_Mask_filename, &lht,&lwd,&bitcount)){     // BMP
    switch(bitcount){
        case 8: imfmt=SAF_BM8; break;     // greyscale bmp
        case 24:imfmt=SAF_BMP; break;     // colour    bmp  
        default:
             show_message("Selected mask bmp image is not 8 or 24 bit (other bit depths are not supported). Cannot proceed.","FAILED: ",MT_ERR,1);
             nullify_mask(); return 1;
      }   
 } else { // It is not one of the allowed unsigned char formats
         show_message("Selected mask image is not of an acceptable format. No custom masking can be done. Try selecting another file.","FAILED: ",MT_ERR,1);
         nullify_mask(); return 1;
        }

  switch(imfmt){
    case SAF_YP5:
    case SAF_BM8:
     width_stride=Selected_Wd;
    break;
    case SAF_RGB:
    case SAF_BMP:
     width_stride=Selected_Wd*3;
    break;
    default: // This should not happen
        show_message("Unrecognised image format for mask image.","Program Error: ",MT_ERR,1);
        nullify_mask();
        return 1;
    }

 // Check dimensions are identical to current main image
 if(lht!=Selected_Ht){
    show_message("Selected mask image is not the same height as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_mask(); return 1;
   }     
 if(lwd!=Selected_Wd){
    show_message("Selected mask image is not the same width as main image. Cannot proceed.","FAILED: ",MT_ERR,1);
    nullify_mask(); return 1;
   }     

 // OK, now see if we can allocate memory for the mask image

 imsz=(size_t)Selected_Ht*(size_t)Selected_Wd; 
 if(resize_memblk((void **)&MaskIm,imsz,sizeof(unsigned char), "the mask image"))
  {
   nullify_mask();
   return 1;
  }

 // We got memory for the mask. Now get memory for a temporary array to
 // load the mask image and try reading the image into it
 rgbimsz=(size_t)Selected_Ht*(size_t)width_stride; 
 tmploc=(unsigned char *)calloc(rgbimsz,sizeof(unsigned char));
 if(tmploc==NULL){
     show_message("Failed to allocate memory to store mask image. Cannot proceed to load it.","FAILED: ",MT_ERR,1);
     nullify_mask(); return 1;
    }

 // We got memory, so now try reading the image into it
 sprintf(msgtxt,"There was a problem reading the chosen mask file. Cannot proceed.");
 switch(imfmt){
        case SAF_YP5: 
            if(get_pgm(Selected_Mask_filename, &tmploc, &lht,&lwd)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_mask(); free(tmploc); return 1;
            }
        break;
        case SAF_BM8: 
            if(get_bmp(Selected_Mask_filename, &tmploc, &lht, &lwd, cref)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_mask(); free(tmploc); return 1;
            }
        break;
        case SAF_RGB:
            if(get_ppm(Selected_Mask_filename, &tmploc)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_mask(); free(tmploc); return 1;
            }
        break;
        case SAF_BMP:
            if(get_bmp(Selected_Mask_filename, &tmploc, &lht, &lwd, cref)){
              show_message(msgtxt,"FAILED: ",MT_ERR,1);
              nullify_mask(); free(tmploc); return 1;
            }
        break;
        default: // This should not happen
         show_message("Unrecognised image format for mask image.","Program Error: ",MT_ERR,1);
         nullify_mask(); free(tmploc); return 1;
    }

 // Now convert the mask image into a binary unsigned char array
 // whatever the original file format was by thresholding at a pixel
 // value of 127.
 msupp=0;
 switch(imfmt){
    case SAF_YP5:
    case SAF_BM8:
      for(idx=0;idx<(int)imsz;idx++){
         if(tmploc[idx]>127){ MaskIm[idx]=255; msupp++;}
         else MaskIm[idx]=0;
       }
    break;
    case SAF_RGB:
    case SAF_BMP:
      for(idx=0,rgbpos=0;idx<(int)imsz;idx++,rgbpos+=3){
         d1=(double)tmploc[rgbpos];
         d2=(double)tmploc[rgbpos+1];
         d3=(double)tmploc[rgbpos+2];
         d1=(d1+d2+d3)/3.0; // Intensity         
         if(d1>127.0){ MaskIm[idx]=255; msupp++;}
         else MaskIm[idx]=0;
       }
    break;
    default: // This can't happen - we checked above
    break;
  }

 // we're done with the temporary array now do free it
 free(tmploc);

 // Check that the mask has at least 1 pixel that is non-zero:
 if(msupp==0){
   show_message("Chosen mask has no support so cannot be used.","FAILED: ",MT_ERR,1);
   nullify_mask(); return 1;
  }

 // Now we set the dimensions and GUI:
 MKht=lht; MKwd=lwd;
 Mask_supp_size=(double)msupp;
 sprintf(MaskFile,"%s",Selected_Mask_filename);
 sprintf(msgtxt,"Mask image loaded: %s",MaskFile);
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 mask_alloced=MASK_YES;
 mskfile_loaded=MASK_YRGB;
 
 if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     //set label to show filename
     gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_rmski]), name_from_path(MaskFile));  
     gtk_widget_set_sensitive (chk_usemskcor,TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_um],TRUE);
     gtk_widget_set_sensitive (CamsetWidget[windex_um2],TRUE);
  }
 
 return 0;
}

static void btn_cs_load_cset_click(GtkWidget *widget, gpointer data) 
// This gets the file name and checks the format - it also attempts to
// load the settings into the various widgets ready for the user to
// click the 'Apply' button (new settings won't take effect till then).
{
 gint res;
 GtkFileChooserAction fca_open = GTK_FILE_CHOOSER_ACTION_OPEN;
 GtkFileChooser *csf_file_chooser;   // Camera settings file load/save
 GtkFileFilter  *csf_file_filter;
 
 // Set up the 'settings load' dialogue with the settings file choosing
 // settings:
 // Set up the settings file selection filter to be used with the file
 // chooser dialog.
 csf_file_filter  = gtk_file_filter_new();
 gtk_file_filter_set_name (csf_file_filter,"Settings Files");                                   
 gtk_file_filter_add_pattern (csf_file_filter, "*.txt");                                     

 load_file_dialog = gtk_file_chooser_dialog_new ("Load a Settings File",
                                      GTK_WINDOW(Win_main),
                                      fca_open,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Open",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);

 csf_file_chooser = GTK_FILE_CHOOSER (load_file_dialog);
 gtk_file_chooser_add_filter (csf_file_chooser,csf_file_filter);
 // gtk_file_chooser_set_current_folder (csf_file_chooser,CSFile);
 
 // Run the 'file load' dialogue
 res = gtk_dialog_run (GTK_DIALOG (load_file_dialog));
 if(res == GTK_RESPONSE_ACCEPT)
   {
    FILE *fp;
    gchar *filename;
    int file_checker_says;
    unsigned int linenum;
    char msgtxt[320],errmsg[256];
    
    filename = gtk_file_chooser_get_filename (csf_file_chooser);
     
    // Check the settings file to see if it is actionable without errors  
    fp = fopen(filename, "rb");
    if(fp == NULL) {
      show_message(msgtxt, "FAILED to Load Settings: ", MT_ERR, 1);
      goto skip_rest;
     }
    sprintf(errmsg, "None provided.");
    file_checker_says = csetfile_check(fp, &linenum, errmsg);
    switch (file_checker_says) {
        case PCHK_ALL_GOOD: // No errors: attempt to load the settings
            sprintf(msgtxt, "Settings file checked OK: %s\n", filename);
            show_message(msgtxt, "FYI: ", MT_INFO, 0);
            break;
        case PCHK_TERMINUS: // Program not terminated correctly
            sprintf(msgtxt, "[%s][%d]: Failed to terminate correctly.\n", name_from_path(filename), linenum);
            show_message(msgtxt, "FAILED to verify settings file: ", MT_ERR, 1);
            goto skip_rest;
        case PCHK_E_SYNTAX: // Syntax error detected
            sprintf(msgtxt, "[%s][%d]: Syntax error. %s\n", name_from_path(filename), linenum, errmsg);
            show_message(msgtxt, "FAILED to verify settings file: ", MT_ERR, 1);
            goto skip_rest;
        case PCHK_E_COMMND: // Unrecognised command
            sprintf(msgtxt, "[%s][%d]: Unrecognised or non-functional command \'%s\'.\n", name_from_path(filename), linenum, errmsg);
            show_message(msgtxt, "FAILED to verify settings file: ", MT_ERR, 1);
            goto skip_rest;
        case PCHK_E_FORMAT: // Incorrect file format
            sprintf(msgtxt, "[%s][%d]: The settings file format is wrong for this camera (or it is not a PARDUS camera settings file).\n", name_from_path(filename), linenum);
            show_message(msgtxt, "FAILED to verify settings file: ", MT_ERR, 1);
            show_message(errmsg, "Reason for failure: ", MT_ERR, 0);
            goto skip_rest;
        default: // This should not happen unless the source code was altered wrongly
            show_message("Settings file checker returned an unreconised value", "Program Error: ", MT_ERR, 1);
            goto skip_rest;
        }
 
    // If we get here it means the settings file checked out OK.
    // Rewind the file pointer and attempt to load all the settings:
    rewind(fp);
    file_checker_says = csetfile_load(fp, &linenum, errmsg);
    switch (file_checker_says) {
        case PCHK_ALL_GOOD: // No errors: attempt to load the settings
            sprintf(msgtxt, "Settings file loaded OK: %s\n", filename);
            show_message(msgtxt, "FYI: ", MT_INFO, 0);
            sprintf(CSFile,"%s",filename); // Update CSFile 
            csetfile_loaded=CSET_CUST;
            sprintf(msgtxt,"Your loaded settings (%s) will not take effect until you click 'Apply All Settings',",name_from_path(CSFile));
            show_message(msgtxt,"FYI: ",MT_INFO,1);
            break;
        case PCHK_TERMINUS: // Program not terminated correctly
            sprintf(msgtxt, "[%s][%d]: Failed to terminate correctly.\n", name_from_path(filename), linenum);
            show_message(msgtxt, "FAILED to load settings file: ", MT_ERR, 0);
            goto skip_rest;
        case PCHK_E_SYNTAX: // Syntax error detected
            sprintf(msgtxt, "[%s][%d]: Syntax error. %s\n", name_from_path(filename), linenum, errmsg);
            show_message(msgtxt, "FAILED to load settings file: ", MT_ERR, 0);
            goto skip_rest;
        case PCHK_E_COMMND: // These can't happen because the load
        case PCHK_E_FORMAT: // doesn't return them and because the
        default: // checker function wouldn't allow us to get here.
         break;
       }
        
    skip_rest:
        g_free(filename);
        fclose(fp);
   }


 // Remove the mask filter from the file load dialogue and close it.
 gtk_file_chooser_remove_filter(csf_file_chooser,csf_file_filter);
 gtk_widget_destroy(load_file_dialog);

 // Set the file name of the loaded settings file
  if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     // csetfile_loaded was set above.
     // Set label to show filename and other GUI controls/labels:
     if(csetfile_loaded==CSET_CUST){
        gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_ldcs]), name_from_path(CSFile)); 
       } 
   }

 
 return;
}

static void btn_cs_save_cset_click(GtkWidget *widget, gpointer data) 
// This attempts to save all current settings to a text file.
{
 gint res;
 GtkFileChooserAction fca_save = GTK_FILE_CHOOSER_ACTION_SAVE;
 GtkFileChooser *csf_file_chooser;   // Camera settings file load/save
 GtkFileFilter  *csf_file_filter;
 
 // Set up the 'settings load' dialogue with the settings file choosing
 // settings:
 // Set up the settings file selection filter to be used with the file
 // chooser dialog.
 csf_file_filter  = gtk_file_filter_new();
 gtk_file_filter_set_name (csf_file_filter,"Settings Files");                                   
 gtk_file_filter_add_pattern (csf_file_filter, "*.txt");                                     

 save_file_dialog = gtk_file_chooser_dialog_new ("Save Current Settings",
                                      GTK_WINDOW(Win_main),
                                      fca_save,
                                      "_Cancel",
                                      GTK_RESPONSE_CANCEL,
                                      "_Save",
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);

 csf_file_chooser = GTK_FILE_CHOOSER (save_file_dialog);
 gtk_file_chooser_add_filter (csf_file_chooser,csf_file_filter);
 // gtk_file_chooser_set_current_folder (csf_file_chooser,CSFile);
 
 // Run the 'file save' dialogue
 res = gtk_dialog_run (GTK_DIALOG (save_file_dialog));
 if (res == GTK_RESPONSE_ACCEPT)
  {
    char msgtxt[256];
    gchar *filename;
    int enumresult;

    
    filename = gtk_file_chooser_get_filename (csf_file_chooser);
     
    // Save the settings file
    
    // Ensure the chosen file name has a .txt extension (or it will not
    // be shown in the file chooser dialog box for loading.
    if(check_extn(filename, "txt", 3, "Settings save FAILED: ")){
      if(check_extn(filename, "TXT", 3, "Settings save FAILED: ")) goto skip_rest;
     }
    // Now initialise the settings file by writing the settings
    // retrieved from the camera firmware to it:  
    // First update the firmware settings value in the linked list of
    // camera settings:
    enumresult=enumerate_camera_settings();
    if(enumresult!=CSE_SUCCESS){
      sprintf(msgtxt,"Failed to get %s settings from camera (VIDIOC_QUERYCTRL, %d)",NCSs?"some":"any",enumresult);
      show_message(msgtxt,"Error: ",MT_ERR,1);
      if(!NCSs) goto skip_rest; 
     } 
     // Now print them:   
    if(print_cs_file(filename)){
        show_message("Could not write the camera controls settings.","Settings save FAILED: ",MT_ERR,1);
        goto skip_rest;  
     }
    // Now append the custom settings provided by PARD Capture
    if(append_cs_file(filename)){
        show_message("Could not write the custom controls settings.","Settings save FAILED: ",MT_ERR,1);
        goto skip_rest;  
     }
     
    // Success. Update the CSFile and csetfile_loaded variables
    sprintf(CSFile,"%s",filename);
    csetfile_loaded=CSET_CUST;

  skip_rest:
    g_free(filename);
  }

 // Close the dialog
 gtk_file_chooser_remove_filter(csf_file_chooser,csf_file_filter);
 gtk_widget_destroy(save_file_dialog);
 
 // Set the file name of the loaded settings file
  if(gtk_widget_is_visible(win_cam_settings)==TRUE){
     // csetfile_loaded was set above.
     // Set label to show filename and other GUI controls/labels:
     if(csetfile_loaded==CSET_CUST){
        gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_sacs]), name_from_path(CSFile)); 
       } 
   }
 
 return;
}

int get_camera_control(unsigned int id, int *ival)
// Gets the value of the camera control with vd_control ID 'id'
// and puts it into 'ival'.
// Returns 0 on success and 1 on failure.
{ 

 memset(&vd_queryctrl, 0, sizeof(vd_queryctrl));
 memset(&vd_control, 0, sizeof (vd_control));

 vd_control.id = id;
 vd_queryctrl.id=vd_control.id;
 *ival=0;

 if(0 == ioctl(fd, VIDIOC_QUERYCTRL, &vd_queryctrl)) {
      if (!(vd_queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
         // Query the vd_control and get current value, name and ranges
         vd_control.id = vd_queryctrl.id;
         if (0 == ioctl(fd, VIDIOC_G_CTRL, &vd_control)) *ival=vd_control.value; else return 1;
     }
  } else return 1;

 return 0;
}

int set_camera_control(unsigned int id, int ival, char *cname)
// Sets the camera control with vd_control ID 'id' to the value 'ival'
// and prints the vd_control name in 'cname' if it can't.
// Returns 0 on success and 1 or 2 on failure.
{
 vd_control.id = id;
 vd_control.value = ival;
 vd_queryctrl.id=vd_control.id; 
 ioctl(fd, VIDIOC_QUERYCTRL, &vd_queryctrl);  
 sprintf(cname,"%s",vd_queryctrl.name);

 if(-1 == ioctl(fd, VIDIOC_S_CTRL, &vd_control)) {
   // Identify if this an error setting the  manual focus value by
   // returning the special value '2'. This will allow us to determine
   // if this is really an error or whether this call to set the value
   // has just come prior to switching off the Auto Focus flag. We will
   // make that determination at the end of setting all camera controle
   // because the auto focus control comes AFTER absolute focus in
   // the list of controls for OptArc cameras.
   if(!strcmp("Focus, Absolute",cname)) return 2;
   return 1;
  }
 return 0;
}

int update_framerate_resolutions(void)
// Query the imaging device to get a list of supported frame resolutions
// and maximum frame rates and put that info into the size combo box.
// If there is an entry that corresponds to Selected_Wd x Selected_Ht
// it sets the combo box cuurently active index to that position.
// If that selected resolution is not found it sets the current position
// of the combo box to VGA resolution.
// If even VGA resolution is not supported it returns 1 and outputs a
// popup error message.
// Otherwise it returns 0.
{
 char ctrl_name[64],ctrl_name2[64],msgtxt[128];
 int fdx,fintdx,comboidx;
 struct v4l2_frmsizeenum frmsizeenum;
 struct v4l2_frmivalenum frmivalenum;
 
 // Query the camera to list the device's frame sizes for the YUYV format
 // into the frame size drop-down list
  for(fdx=0;fdx<MAX_RESOLUTIONS;fdx++) maxframerate[fdx]=0;

   memset(&frmsizeenum, 0, sizeof(frmsizeenum));
   frmsizeenum.pixel_format=CamFormat; 
   fdx=0; frmsizeenum.index=fdx;
   show_message("Attempting to enumerate the supported frame sizes (W x H):","FYI: ",MT_INFO,0);
   
   // Clear the current entries from the size combo:
   gtk_combo_box_text_remove_all ( (GtkComboBoxText *) combo_sz);

   // Append the values to the size combo list and set the default item
   // as the current ImHeight and ImWidth (or 0 if none of the listed
   // formats match the current values).
   CurrDims_idx = VGA_idx = -1; comboidx=0;
   while(0 == ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum)){

         // PARD Capture does not yet support these frame heights:
         if(frmsizeenum.discrete.height==288) goto Next_resolution;
         if(frmsizeenum.discrete.height==144) goto Next_resolution;

         sprintf(ctrl_name,"%u x %u",frmsizeenum.discrete.width,frmsizeenum.discrete.height);
         sprintf(msgtxt,"\t[%d]-> %s",fdx,ctrl_name);
         if((Selected_Ht == (int)frmsizeenum.discrete.height) && (Selected_Wd == (int)frmsizeenum.discrete.width)) CurrDims_idx=comboidx;
         if(640 == frmsizeenum.discrete.width && 480 == frmsizeenum.discrete.height) VGA_idx = comboidx;
         show_message(msgtxt,"",MT_INFO,0);
         // Enumerate frame intervals for this resolution and format
         // (frame rate = 1 / frame interval)
         memset(&frmivalenum, 0, sizeof(frmivalenum));
         frmivalenum.pixel_format=CamFormat;
         fintdx=0; frmivalenum.index=fintdx;
         frmivalenum.width=frmsizeenum.discrete.width;
         frmivalenum.height=frmsizeenum.discrete.height;
         maxframerate[comboidx]=0; // The index is comboidx not fintdx 
                                   // because this is going to be the
                                   // maximum frame rate for the current
                                   // resolution entry in the combo list
         while(0 == ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum)){
              sprintf(ctrl_name2," at %u/%u fps",frmivalenum.discrete.denominator,frmivalenum.discrete.numerator);
              sprintf(msgtxt,"\t\tFrame rate [%d]%s",fintdx,ctrl_name2);
              show_message(msgtxt,"",MT_INFO,0);
              if(frmivalenum.discrete.denominator>maxframerate[comboidx]) maxframerate[comboidx]=frmivalenum.discrete.denominator / frmivalenum.discrete.numerator;
              fintdx++;
              frmivalenum.index=fintdx;
             }
         // Done enumerating frame intervals for this resolution.
         sprintf(msgtxt,"%s%s",ctrl_name,ctrl_name2);
         gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT (combo_sz),NULL,msgtxt);
         comboidx++;
         Next_resolution:
         fdx++; // index of next frame size to query  
         frmsizeenum.index=fdx;
        }
   // Update the number of resolutions in the drop down list:
   Nresolutions=comboidx;

   // This just prints a newline character (for non-GUI notices):
   show_message("","",MT_INFO,0); 
   // Show error message:
   if(errno != EINVAL) {
       show_message("Error when retrieving frame sizes.","Error: ",MT_ERR,1);
     }
   gtk_combo_box_set_active(GTK_COMBO_BOX (combo_sz), 1);

   if(CurrDims_idx>=0) gtk_combo_box_set_active(GTK_COMBO_BOX (combo_sz), CurrDims_idx);
   else if(VGA_idx>=0) gtk_combo_box_set_active(GTK_COMBO_BOX (combo_sz), VGA_idx);
   else {
       show_message("Could not get even a VGA frame size.\nYou should save your work now before clicking 'OK' - this may not end well.","Error: ",MT_ERR,1);
       return 1;
   }
   
 return 0;
}

int fps_index(int tdx)
// Gets the index in the preview frames per second list of strings given
// the desired numerical fps entry (as tdx). Defaults to the first
// index in the list if idx is not one of the entries. 
{
 int i;
  for(i=0;i<fpx_max;i++){
    if(!(atoi(fps_options[i])-tdx)) return i;
  }
 return 0;
}

int cs_int_range_check(int limlo, int limhi,const char *cname, int proposed_val, int verbose)
// Check if a proposed integer value, proposed_val, lies in the closed
// interval limlo to limhi for control called cname.
// If it isn't, return non-zero
// If verbose is non-zero show a pop-up error message.
{
 char emsg[320];
 
 if(proposed_val<=limlo || proposed_val>=limhi){
   if(verbose){
     sprintf(emsg,"The value for '%s' must be between %d and %d (inclusive).\nThe value you chose will not be applied. Reverting to previous value.",cname,limlo+1,limhi-1);
     show_message(emsg,"Invalid Setting: ",MT_ERR,1);
    }
   return 1;
  }
 return 0;
}

static void btn_cs_apply_click(GtkWidget *widget, gpointer data)
// Apply the user's chosen settings to the frame grabber and image
// processing behaviours
{
 int ctrlindex,idx,tdx,tmp_preview,tmp_avd,szidx,esdx;
 int cfchanged,requestedfmt;
 char msgtxt[320],cname[64];
 gchar *numstr,*markup;
 int MFidx,retval,manualfocus; // For correct manual focus warning
 const char *format = "<span foreground=\"red\">\%s</span>";
 int currval,MFval,cidx;       // Camera control values
 int AutoWB,AutoExp,AutoFocus; // Camera control flags

 if(Ser_active>0) return; // Don't allow changes during a series capture
 if(Av_limit>1) return; // Don't allow changes during an average capture

 // If none of the supported formats are provided by the camera, you
 // cannot allow settings to be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot apply settings because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }

 // If previewing is active, switch it off while we get the new settings
 // - else the preview timout could call the preview function in the
 // middle of settings changes and cause a crash (e.g. due to mixed
 // dimensions, old and new)
 if(Need_to_preview){
    tmp_preview=Need_to_preview; 
    Need_to_preview=PREVIEW_OFF;
   } else tmp_preview=0;
 
 cfchanged=0; // To record if the format has changed since last update.
              // Either the image dimensions and/or the camera output
              // stream format.
 Preview_changed=0;// To know if we need to update preview parameters.
                   // We will always need to update preview parameters
                   // if cfchanged is set but we don't need to update
                   // image dimensions if only the preview mode has
                   // changed, hence the need for this flag in addition
                   // to cfchanged.

  // Get the height and width selections for full size image capture 
  numstr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_sz));
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_sz]),numstr); 
  sscanf(numstr,"%d %*s %d",&Selected_Wd,&Selected_Ht);
  sprintf(msgtxt,"You chose: W = %d,  H = %d",Selected_Wd,Selected_Ht);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the 'Use crop from full-size image as preview?' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_preview_central))==TRUE){
     if(selected_Preview_fullsize==0) Preview_changed=1;
     selected_Preview_fullsize=1;
   } else {
     if(selected_Preview_fullsize) Preview_changed=1;
     selected_Preview_fullsize=0;
   }
  if(selected_Preview_fullsize) numstr = g_strdup_printf("Yes"); else numstr = g_strdup_printf("No");
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_pc]),numstr);
  sprintf(msgtxt,"You chose: Use crop from full-size image as preview? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the 'monochrome preview' selection. Changing this does NOT need
  // the preview parameters to be re-calculated so we do not set the 
  // Preview_changed flag.
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_cam_yonly))==TRUE){
       col_conv_type=CCOL_TO_Y;
       numstr = g_strdup_printf("Yes");
       gtk_widget_show(prev_int_label);
       gtk_widget_show(prev_bias_label);
       gtk_widget_show(preview_integration_sbutton);
       gtk_widget_show(preview_bias_sbutton);
       gtk_widget_show(preview_corr_button);
   } else {
       col_conv_type=CCOL_TO_RGB ;
       numstr = g_strdup_printf("No");
       gtk_widget_hide(prev_int_label);
       gtk_widget_hide(prev_bias_label);
       gtk_widget_hide(preview_integration_sbutton);
       gtk_widget_hide(preview_bias_sbutton);
       gtk_widget_hide(preview_corr_button);
   }
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_yo]),numstr);
  sprintf(msgtxt,"You chose: Preview in monochrome? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the camera stream format selection. If it has changed, update
  // the list of resolutions and maximum frame rates accordingly.
  // Also, set the updated resolutions combo box to the currently
  // selected HxW if that is included in the updated resolutions list or
  // set it to 640x480 as a fallback default with error warning if the
  // user's choice is not supported for the new camera stream format.
  numstr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_camfmt));
  requestedfmt=camfmt_from_string(numstr);
  if(FormatForbidden==requestedfmt){
    show_message("The camera stream format you requested is not supported by your camera.","FYI: ",MT_INFO,1);
    // Reset the camera format combo box.
    switch(CamFormat){
      case V4L2_PIX_FMT_YUYV:
        gtk_combo_box_set_active (GTK_COMBO_BOX (combo_camfmt), 0);
        numstr = g_strdup_printf("Raw YUYV");
      break;
      case V4L2_PIX_FMT_MJPEG:
        gtk_combo_box_set_active (GTK_COMBO_BOX (combo_camfmt), 1);
        numstr = g_strdup_printf("MJPEG");
      break;
      default: // Should never get here. Set YUYV by default if you do.
         show_message("Unrecognised camera format.","Program Error: ",MT_ERR,1);
      break;
    }
   } else {
    switch(requestedfmt){
      case CAF_YUYV:
       if(CamFormat != V4L2_PIX_FMT_YUYV){
          cfchanged=1;
          CamFormat = V4L2_PIX_FMT_YUYV;
         }
      break;
      case CAF_MJPEG:
       if(CamFormat != V4L2_PIX_FMT_MJPEG){
          cfchanged=1;
          CamFormat = V4L2_PIX_FMT_MJPEG;
         }
      break;
      default: // Should never get here. Set YUYV by default if you do.
         show_message("Unrecognised camera format. Using YUYV","Program Error: ",MT_ERR,1);
       if(CamFormat != V4L2_PIX_FMT_YUYV){
          cfchanged=1;
          CamFormat = V4L2_PIX_FMT_YUYV;
         }
      break;
    }
  }
    gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_camfmt]),numstr);
    sprintf(msgtxt,"You chose: camera stream format = %s",numstr);
    show_message(msgtxt,"FYI: ",MT_INFO,0);
    g_free(numstr);
 
  // If the camera stream format has changed since last update, 
  // update the resolutions and max framerate for this format
  if(cfchanged){
    Preview_changed = 1; // We must update the preview settins later.                  
    if(update_framerate_resolutions()){
         sprintf(msgtxt,"Failed to get a valid frame resolution for the new camera stream format");
         show_message(msgtxt,"Warning: ",MT_ERR,0);
        } 
    // Re-initialising will set the new camera stream format:
    if(re_init_device()){ 
      sprintf(msgtxt,"Failed to set the new camera stream format");
      show_message(msgtxt,"Warning: ",MT_ERR,0);
      //esdx++;
    }
   }

  // Get the preview fps selection 
  numstr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_fps));
  // Get the selected image dimensions index
  szidx=gtk_combo_box_get_active (GTK_COMBO_BOX(combo_sz)); 
  sscanf(numstr,"%d",&tdx); // Get the selected frame rate.
  // If the user-selected frame rate (tdx) is greater than the maximum
  // frame rate supported for the chosen resolution, advise the user and
  // don't select it. Use the maximum framerate instead.
  if(tdx>maxframerate[szidx]){  
   sprintf(msgtxt,"Your selected frame rate (%d) is greater than the maximum your camera can support at the chosen resolution (%u).\n"
                  "The frame rate will we set to %d fps for now until you change it.",tdx,maxframerate[szidx],maxframerate[szidx]);
   show_message(msgtxt,"Warning: ",MT_INFO,1);
   tdx=maxframerate[szidx];
   g_free(numstr);
   numstr = g_strdup_printf("%d",tdx);
   gtk_combo_box_set_active (GTK_COMBO_BOX (combo_fps), fps_index(tdx));
  }  
  preview_fps = 1000/tdx;
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_fps]),numstr); 
  sprintf(msgtxt,"You chose: frame rate %s fps (delay = %d)",numstr,preview_fps);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);
  // Notify the timeout that it needs to update its interval parameter
  change_preview_fps=1; 

  // Get the save-to-disk format selection 
  numstr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_safmt));
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_safmt]),numstr);
  saveas_fmt = saveas_from_string(numstr);
  sprintf(msgtxt,"You chose: to save images as format %s [%d]",numstr,saveas_fmt);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the 'Save as raw doubles?' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_sa_rawdoubles))==TRUE){
       Save_raw_doubles=1;
       numstr = g_strdup_printf("Yes");
   } else {
       Save_raw_doubles=0 ;
       numstr = g_strdup_printf("No");
   }
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_sad]),numstr);
  sprintf(msgtxt,"You chose: Save as raw doubles? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the 'Save as FITS?' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_sa_fits))==TRUE){
       Save_as_FITS=1;
       numstr = g_strdup_printf("Yes");
   } else {
       Save_as_FITS=0 ;
       numstr = g_strdup_printf("No");
   }
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_fit]),numstr);
  sprintf(msgtxt,"You chose: Save as FITS? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);

  // Get the 'Scale mean of each frame to first?' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_scale_means))==TRUE){
       Av_scalemean=1;
       numstr = g_strdup_printf("Yes");
   } else {
       Av_scalemean=0 ;
       numstr = g_strdup_printf("No");
   }
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_smf]),numstr);
  sprintf(msgtxt,"You chose: Scale mean of each frame to first? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);


 // If the user elects to use a custom mask, update the mask.
 if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usemskcor))==TRUE){
  // Update the mask - we need this before initialising any dark or flat
  // field image because the dark and flat field image pre-processing
  // must only be done in the region of support of the mask (the
  // normalisation pre-process may use wrong averages if the average is
  // calculated in a different regions of support, e.g. full frame,
  // when the actual flat field divisional correction is restricted to
  // limited masked-off support):
   set_mask_pending();
   if(msk_pending) msk_pending=init_mask_image();
   else { // If there is no user-specified mask pending we need some
          // kind of mask alloced, even if that is a full support mask.
          // 'mask_pending' may be 0 because we successfully loaded a
          // a mask or it  may be 0 because this is the very first time
          // we are applying settings since the program started and we
          // have not yet alloced any mask. So check this.
          // Perhaps a mask is alloced but to the wrong size for the
          // currently selected dimensions. So msk_pending is 0 and 
          // mask_alloced is MASK_YES but we can no longer use the 'old'
          // loaded mask - so check this and reset the mask accordingly.
         if(mask_alloced==MASK_NO || MKht!=Selected_Ht || MKwd!=Selected_Wd)
            set_mask_full_support(Selected_Ht,Selected_Wd);
        }
  // If the user did not elect to use a custom mask, generate a full 
  // support mask.
  } else set_mask_full_support(Selected_Ht,Selected_Wd);

  // If the user selected a new dark field image, load and assess it - 
  // but only if the user selected to use it - why bother them with
  // warnings about their dark field image being unusable if they don't
  // want to use it?
  if(df_pending && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usedfcor))==TRUE){
    if(mask_alloced==MASK_NO){
       show_message("Although a dark field image was selected it cannot be processed or used due to failure to load or generate a mask.","FYI: ",MT_INFO,1);
      } else if(init_darkfield_image()){
       show_message("Although a dark field image was selected it cannot be processed or used due to failure to load and initialise it","FYI: ",MT_INFO,1);
      }
   } else df_pending=0;

    
  // If the user selected a new flat field image, load and assess it - 
  // but only if they say they want to use it.
  // This is done here AFTER the mask initialisation above for the
  // reasons stated above.
  if(ff_pending && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_useffcor))==TRUE){
    if(mask_alloced==MASK_NO){
       show_message("Although a flat field image was selected it cannot be processed or used due to failure to load or generate a mask.","FYI: ",MT_INFO,1);
      } else if(init_flatfield_image(0)){
       show_message("Although a flat field image was selected it cannot be processed or used due to failure to load and initialise it","FYI: ",MT_INFO,1);
      }
    } else ff_pending=0;

  // If control widgets were created, look for those with editable
  // numerical camera settings values and try to set them to the user's
  // chosen value.
  
  // Initialise flags and variables for detecting exceptions to errors
  // in control settings
  AutoWB=-1;
  AutoExp=-1;
  AutoFocus=-1;
  MFval= -1;
  manualfocus = 0;
  
  // If there are any controls ...
  if(windex){ 
      
   esdx=0; // Number of controls that fail to set to the user's choice 
   
   // Loop through each control widget ...    
   for(ctrlindex=idx=tdx=0;ctrlindex<windex;ctrlindex++){
       // If this not an entry box, skip to the next control
       if(cswt_id[ctrlindex]!=CS_WTYPE_ENTRY) continue; 
       // If it is a camera control retrieved by ioctl from the camera,
       // attempt to set the control value to the user's choice using
       // ioctl (using the function 'set_camera_control')
       if(ctrl_id[ctrlindex]){ 
          sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
          currval=atoi(msgtxt);
          // Check if the wanted value is in the range for the setting
          retval=check_camera_setting(ctrl_id[ctrlindex], currval, &cidx);
          switch(retval){
             case CSC_OK:    // All good
             goto try_setting;
             case CSC_NOCS:  // No settings are loaded (N/A here)
                show_message("No camera settings are available!","Warning: ",MT_ERR,0);
             break;
             case CSC_NOID:  // The proferred control ID is not in the
                             // list of controls for the camera
                             // currently being used.
                show_message("Camera ID is not found!","Warning: ",MT_ERR,0);
             break;
             case CSC_RANGE: // The proferred control value is outside
                             // the range supported by that control
                             // for the currently active camera.
                show_message("Your choice of value for this setting is out of range!","Warning: ",MT_ERR,0);
             break;
             default: // Should not happen if program written corectly
                show_message("Unidentified check_camera_setting response!","Program Error: ",MT_ERR,1);
             break;
          }
          goto cset_failure;
          // If we get here, the chosen value is in the control's range:
          try_setting:
          retval=set_camera_control(ctrl_id[ctrlindex],currval,cname);
          if(retval){
          // This control could not be set even though the desired value
          // passed the acceptable range test. So check if this is
          // normal for the overall configuration of settings (e.g. you
          // normally can't set manual exposure if auto-exposure is
          // active - this is not an error):
            if(retval==2){ // Special case if MF could not be set:
               manualfocus=1;  // Need to retry setting MF at the end.
               MFval= currval; // This is the value to try to set. 
               MFidx=ctrlindex;// ID of the MF control.
              } else { // Get the current value of the control
               cset_failure:
               // I found that if setting attempt fails the cname
               // does not get set so let us retrieve the name of the 
               // failed setting first so we can notify the user:
               cidx=ncsidx_from_ctrl_id(ctrl_id[ctrlindex]);
               sprintf(cname,"%s",CSlist[cidx].name);

               if(get_camera_control(ctrl_id[ctrlindex], &currval))
                  sprintf(msgtxt,"FAIL");
               else sprintf(msgtxt,"%-7d",currval);
             
               // Update the GUI label with this current value
               gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);

               // Now update the CSList entry value
               CSlist[cidx].currval=currval;
               
               // Now check for exceptions to being an error:
               // Was it a manual control but we are in auto mode?
               if(!strcmp("White Balance Temperature",cname) && AutoWB==1) goto Setting_success;
               if(!strcmp("Exposure Time, Absolute",cname) && AutoExp>=2) goto Setting_success;
               
               // If we get here, this is an error, so mark it as such:
               markup = g_markup_printf_escaped (format, msgtxt);
               gtk_label_set_markup (GTK_LABEL(CamsetWidget[ctrlindex+1]), markup);
               g_free (markup);
               sprintf(msgtxt,"Failed to set control %s to value %d (VIDIOC_S_CTRL)",cname,atoi(gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex]))));
               show_message(msgtxt,"Warning: ",MT_ERR,0);
               esdx++;
              }
           } else {
             // User's choice was successfully set so update the current
             // value label, auto-mode flags and the success counter.
             gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             if(!strcmp("White Balance, Automatic",cname)) AutoWB=currval;
             if(!strcmp("Auto Exposure",cname)) AutoExp=currval;
             if(!strcmp("Focus, Automatic Continuous",cname)) AutoFocus=currval;
             // Now update the CSList entry value
             CSlist[ncsidx_from_ctrl_id(ctrl_id[ctrlindex])].currval=currval;

             Setting_success:
             idx++; // The number of values successfully changed.
           }
           
         } else {
        // It is an entry box but not for a value that comes from the
        // camera's firmware settings list (retrieved by ioctl) so it is
        // a custom setting. Handle it accordingly
             if(ctrlindex == windex_imroot){     // Image root name
                     sprintf(ImRoot,"%s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),ImRoot);
                     idx++;
             } else if(ctrlindex == windex_fno){ // Image frame number
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     if(is_not_integer(msgtxt)){
                        show_message("The value you supplied for 'File frame number start from' is not a valid integer and will not be set","Warning: ",MT_ERR,1);
                       } else {
                        frame_number = atoi(msgtxt);
                        idx++;
                       }
                     sprintf(msgtxt,"%-7d",frame_number);
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_gn){  // YUYV to RGB gain
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     Gain_conv = atof(msgtxt);
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
                     idx++;
             } else if(ctrlindex == windex_bs){  // YUYV to RGB bias
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     Bias_conv = atof(msgtxt);
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
                     idx++;
             } else if(ctrlindex == windex_del){ // Delay first capture
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 1 to 172800
                     if(cs_int_range_check(-1, 172801,"Delay first capture by (s)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",(int)Delayed_start_seconds);
                     } else {//  Set the new frame average denominator
                      Delayed_start_seconds = (double)tmp_avd;
                      idx++;
                     }
                     if(Delayed_start_seconds>=1.0) Delayed_start_on=1;
                       else Delayed_start_on=0;
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_avd){ // No. frames to average
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 1 to 4096
                     if(cs_int_range_check(0, 4097,"Frame averaging (number of frames)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",Av_denom);
                     } else {//  Set the new frame average denominator
                      Av_denom = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_to){  // Grabber timeout
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 4 to 360
                     if(cs_int_range_check(3, 361,"Grabber timeout (seconds)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",Gb_Timeout);
                     } else {//  Set the new grabber timeout
                      Gb_Timeout = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_rt){  // Number of re-tries
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 0 to 4096
                     if(cs_int_range_check(-1, 4097,"Frame capture (number of retries)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",Gb_Retry);
                     } else {//  Set the new number of retries
                      Gb_Retry = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_srn){ // Number of captures
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 1 to 604800 (arbitrary limit,
                     // it is enough to record continuously for 1 week 
                     // at a delay interval of 1 second between images).
                     if(cs_int_range_check(0, 604801,"Series (number of images)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",Ser_Number);
                     } else {//  Set the new number of retries
                      Ser_Number = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_srd){ // Delay between each capture
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 0 to 86400 (arbitrary limit,
                     // 86400 seconds is 24 hours).
                     if(cs_int_range_check(-1, 86401,"Min. interval for series (s)", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",Ser_Delay);
                     } else {//  Set the new number of retries
                      Ser_Delay = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else if(ctrlindex == windex_jpg){ // JPEG save quality
                     sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[ctrlindex])));
                     tmp_avd=atoi(msgtxt);
                     // Check this value is within acceptable limits
                     // hard coded here as 1 to 100.
                     if(cs_int_range_check(0, 101,"JPEG save quality", tmp_avd,1)){
                      sprintf(msgtxt,"%-7d",JPG_Quality);
                     } else {//  Set the new number of retries
                      JPG_Quality = tmp_avd;
                      idx++;
                     }
                     gtk_label_set_text(GTK_LABEL(CamsetWidget[ctrlindex+1]),msgtxt);
             } else { // This should not happen unless the programmer has made a mistake
                    show_message("Unidentified custom camera setting edit box!","Program Error: ",MT_ERR,1);
             }
          }
       tdx++; // tdx counts the total number of entry box settings present.
      }
    // Special case of manual focus setting handling. If there was an
    // error setting MF it could be because the AF was not disabled in
    // time (before trying to set the MF value) so, if AF was eventually
    // disabled, let us try once more to set the MF value:
    if(manualfocus) // Only bother trying if MF failed the first time:
     {              
      if(AutoFocus==0) { // AF is (now) disabled - so try setting MF:
        retval=set_camera_control(ctrl_id[MFidx],MFval,cname);
        if(retval){ // OK - no excuse, this really is an error:
          if(get_camera_control(ctrl_id[MFidx], &currval)) sprintf(msgtxt,"FAIL");
          else sprintf(msgtxt,"%-7d",currval);
          gtk_label_set_text(GTK_LABEL(CamsetWidget[MFidx+1]),msgtxt);
          markup = g_markup_printf_escaped (format, msgtxt);
          gtk_label_set_markup (GTK_LABEL(CamsetWidget[MFidx+1]), markup);
          g_free (markup);
          sprintf(msgtxt,"Failed to set control %s to value %d (VIDIOC_S_CTRL)","Focus, Absolute",atoi(gtk_entry_get_text(GTK_ENTRY(CamsetWidget[MFidx]))));
          show_message(msgtxt,"Warning: ",MT_ERR,0);
          esdx++;
       } else { 
        // MF was set without error, increment the success counter
        // and update the current MF value label.
        sprintf(msgtxt,"%-7s",gtk_entry_get_text(GTK_ENTRY(CamsetWidget[MFidx])));
        gtk_label_set_text(GTK_LABEL(CamsetWidget[MFidx+1]),msgtxt);
        idx++;
       }
     } // AF is enabled, so failure to set MF is not an error. In this
       // case I just do nothing - ignore the fact that MF setting
       // failed and don't bother updating the MF current value label.
    }
    
    // Alert the user if any of the controls could not be set to their
    // chosen value (after all legitimate exceptions were taken into 
    // account)
    if(esdx){
            sprintf(msgtxt,"Failed to set %d settings (details given in printout)",esdx);
            show_message(msgtxt,"Warning: ",MT_ERR,1);
     } 
   }
 
 // Update the colour conversion LUTs in case Gain_conv or Bias_conv were changed.
 calculate_yuyv_luts(); 
  
 sprintf(msgtxt,"Set %d (out of %d) camera settings",idx,tdx);
 show_message(msgtxt,"FYI: ",MT_INFO,0);    
 
 // If user selected a different image resolution or preview mode,
 // attempt to implement these:
 switch(change_image_dimensions())
  {
      case CID_OK: // All OK - nothing more to do. No need to set
                   // Preview_changed because change_image_dimensions()
                   // will do this if it succeeds in setting the new
                   // dimnensions and, if it fails, we set the
                   // Preview_changed flag in the cases below. The only
                   // other option is that change_image_dimensions()
                   // returns CID_OK because the new image dimensions
                   // have not changed at all from the previous ones -
                   // in which case the preview does not need to be
                   // updated due to a dimension change.
      break;
      case CID_NOCLOSE: // Failed - couldn't close the imaging device
       show_message("Failed to change image resolution (can't close the camera)","Error: ",MT_ERR,1);
      break;
      // User error notices for the following three cases will have been
      // given via change_image_dimensions() so no need to create new
      // error pop up dialogue boxes here:
      case CID_NOREVERT: // Failed and couldn't even revert to previous.
                         // The program could crash if not exited soon!
      case CID_REVERTED: // Failed to change resolution - reverted back
                         // to previous resolution.
      case CID_NOSTREAM: // Succeeded in changing dimensions but failed
                         // to reopen the imaging stream.
      Preview_changed=1; // Anything other than success means we should
                         // re-initialise the preview settings.
      break;
      default: // This should not happen unless the program was edited incorrectly.
       show_message("Unrecognised return value for change_image_dimensions().","Program Error: ",MT_ERR,1);
      break;
  }


 // If, after any dimension change etc., a mask image is loaded
 // and ready for use, check if the user wants to apply it and check if
 // the dimensions are still correct - change_image_dimensions may not
 // have been able to set the selected width and height and may have
 // reverted to preview dimensions or fallback VGA so any mask loaded
 // earlier using the orignal selected width and height might be of the
 // wrong dimensions now.
 if(MKht!=ImHeight || MKwd!=ImWidth){
   // Warn the user if they chose to use a custom mask
   if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usemskcor))==TRUE){
     if(mskfile_loaded!=MASK_NONE){
       sprintf(msgtxt,"Currently loaded mask is of the wrong dimensions for the image so cannot be used.");
       show_message(msgtxt,"FYI: ",MT_INFO,0);
      }
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usemskcor),FALSE);
     }
    // Discard any user-supplied mask data:
    mask_status=MASK_NO;
    set_mask_full_support(ImHeight,ImWidth);
    show_message("A custom corrections mask cannot be applied","FYI: ",MT_INFO,0);
  } else  if(mskfile_loaded!=MASK_NONE){
 // Image dimensions have not deviated from the user's choice so set
 // their preference:

  // Get the 'Use a corrections mask' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usemskcor))==TRUE){
     mask_status=MASK_YES;
     numstr = g_strdup_printf("Yes");
   } else {
       mask_status=MASK_NO;
       numstr = g_strdup_printf("No");
       // Discard any user-supplied mask data:
       set_mask_full_support(ImHeight,ImWidth);
   }

  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_um]),numstr);
  sprintf(msgtxt,"You chose: Use a corrections mask? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);
 }

    
 // If, after any dimension change etc., a dark field image is loaded
 // and ready for use, check if the user wants to apply it.
 // This only applies if a mask has been alloced - even if it is a full
 // mask.
 if(dffile_loaded!=DFIMG_NONE){
  // Get the 'Use dark field correction' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usedfcor))==TRUE){

    if(mask_alloced==MASK_NO){
       show_message("Although dark field subtraction was selected it cannot be done due to failure to load or generate a mask.","FYI: ",MT_INFO,1);
       dfcorr_status=DFCORR_OFF;
       numstr = g_strdup_printf("No");
       gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_usedfcor),FALSE);
    } else {
     dfcorr_status=DFCORR_ON;
     numstr = g_strdup_printf("Yes");
    }
   
   } else {
       dfcorr_status=DFCORR_OFF;
       numstr = g_strdup_printf("No");
   }
 
  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_ud]),numstr);
  sprintf(msgtxt,"Setting: Use dark field subtraction? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);
 }
    
 // If, after any dimension change etc., a flat field image is loaded
 // and ready for use, check if the user wants to apply it
 // This only applies if a mask has been alloced - even if it is a full
 // mask.
 if(fffile_loaded!=FFIMG_NONE){

  // Get the 'Use flat field correction' selection 
  if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_useffcor))==TRUE){

    if(mask_alloced==MASK_NO){
       show_message("Although flat field division was selected it cannot be done due to failure to load or generate a mask.","FYI: ",MT_INFO,1);
       ffcorr_status=FFCORR_OFF;
       numstr = g_strdup_printf("No");
       gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_useffcor),FALSE);
     } else {

      // We have to load and re-initialise the flat field image at this
      // stage - after the mask option is selected - because the flat
      // field image is pre-processed within the region of mask support
      // and this might have changed by now with user selection of
      // whether to use a custom mask or not (full mask).
      if(init_flatfield_image(1)){
        show_message("Although a flat field image was selected it cannot be processed or used due to failure to normalisee it","FYI: ",MT_INFO,1);
        ffcorr_status=FFCORR_OFF;
        numstr = g_strdup_printf("No");
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_useffcor),FALSE);
       } else {
         ffcorr_status=FFCORR_ON;
         numstr = g_strdup_printf("Yes");
       }

     }

   } else {
       ffcorr_status=FFCORR_OFF;
       numstr = g_strdup_printf("No");
   }

  gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_uf]),numstr);
  sprintf(msgtxt,"Setting: Use flat field division? - %s",numstr);
  show_message(msgtxt,"FYI: ",MT_INFO,0);
  g_free(numstr);
 }


 // Update the preview settings if that is indicated due to either a
 // main image dimension change, a camera stream format change or a
 // change in the user's choice of preview type (full frame or tile):
  if(Preview_changed){
     if(update_preview_settings(selected_Preview_fullsize)) tmp_preview = 0;
     if(Preview_fullsize && tmp_preview==PREVIEW_ON){
          gtk_label_set_text(GTK_LABEL(Label_preview)," Click to zoom ");
          gtk_widget_show(Ebox_lab_preview);
        }
     // Hide the 'Click to zoom' label if it was showing from the user's
     // previous settings
     if(Preview_fullsize==0){ 
        if(tmp_preview==PREVIEW_ON){
          gtk_widget_hide(Ebox_lab_preview);
        }
      }
    }


 // If we suspended previewing, restore the preview status
 if(tmp_preview) Need_to_preview=tmp_preview; 


 return;
}

static void is_change_click(GtkWidget *widget, gpointer data)
// This is called when a the camera settings slider changes its value
{
 char valstr[32],cname[128];
 double dval;
 int retval,currval;

 // If the slider is not assigned to a camera control, return.
 if(IScidx<0) return;
 if(ISwindex<0) return;

 // Get the current value of the slider
 dval=gtk_adjustment_get_value (ISlider_params);
 currval=(int)dval;

 // Attempt to set camera control
 retval=set_camera_control(CSlist[IScidx].ctrl_id,currval,cname);
 if(retval) return; // Failed - do nothing. This will probably be
                    // due to attempting to set a manual value when
                    // the 'auto' version is activated.

 // If succeeded in setting camera control, update the GUI settings:
 // Get the actual camera setting (in case it set differently to what
 // we told it to for some reason. This is so we are sure we display the
 // actual current setting of the camera:
 if(get_camera_control(CSlist[IScidx].ctrl_id, &currval)) sprintf(valstr,"FAIL");
   else sprintf(valstr,"%-7d",currval);

 // Put this text into the entry box
 put_entry_txt(valstr,CamsetWidget[ISwindex]);
       
 // Update the GUI label with this current value
 gtk_label_set_text(GTK_LABEL(CamsetWidget[ISwindex+1]),valstr);
 // Select-all text in the entry box
 gtk_editable_select_region(GTK_EDITABLE(CamsetWidget[ISwindex]), 0,-1);

 // Now update the CSList entry values
 CSlist[IScidx].currval=currval;

 return;
}

static void cs_edit_click(GtkWidget *widget,  GdkEventFocus *event, gpointer data)
// This is called when one of the camera edit boxes gains focus for the
// purpose of setting up the interactive slider control on that setting.
{
 int cidx,wdx;
 
 // If none of the supported formats are provided by the camera, you
 // cannot allow settings to be set:
 if(FormatForbidden==CAF_ALLBAD){
   show_message("You cannot apply settings because your camera does not support YUYV or MJPEG streaming. Restart the program with another camera.","FYI: ",MT_INFO,1);
   return;  
  }

 // Only do this when focus is gained - not lost:
 if(event->in != TRUE) return; 

 // Get the widget index from the widget that called this function. This
 // allows us to retrieve and set the values in the edit box via the
 // slider control and also to get the camera control structure ID so we
 // can get the range of values and the control ID needed to change the
 // the camera settings (as well as the control name to put on the
 // slider's label):
 wdx=windex_from_widget(widget);
 if(wdx<0) return; // Widget index not found. This shouldn't happen.

 // Get the camera control data structure ID from the widget index
 cidx=ncsidx_from_ctrl_id(ctrl_id[wdx]);
 if(cidx<0) return; // Control index not found. This shouldn't happen.

 // Now set the global flags that tell the interactive slider what to
 // contriol:
   IScidx=cidx; // Camera settings structure for interactive control
   ISwindex=wdx;// Edit box widget index for GUI interactive updates


 // Now set up the slider parameters to match that of the selected
 // camera sontrol:
 gtk_adjustment_set_lower (ISlider_params,(double)CSlist[cidx].minimum);
 gtk_adjustment_set_upper (ISlider_params,(double)CSlist[cidx].maximum+1);
 gtk_adjustment_set_value (ISlider_params,(double)CSlist[cidx].currval);
 gtk_adjustment_set_step_increment (ISlider_params,(double)CSlist[cidx].step);
 // Set the slider control label:
 gtk_label_set_text (GTK_LABEL(ISLabel),CSlist[cidx].name);

 // Select-all text in the entry box
 //gtk_editable_select_region(GTK_EDITABLE(CamsetWidget[ISwindex]), 0,-1);


 return;
}

static int add_settings_line_to_gui(const gchar *ctrl_value, const gchar *ctrl_name, GtkInputPurpose purpose)
{
  // I use this 'email' purpose as a marker that the 'control' is just
  // a title / heading and does not need an entry box
  if(purpose==GTK_INPUT_PURPOSE_EMAIL){ 
    gchar *markup;
    const char *format = "<span style=\"italic\" weight=\"bold\">\%s</span>";
    CamsetWidget[windex]=gtk_label_new (ctrl_name);  cswt_id[windex]= CS_WTYPE_LABEL;
    gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
    gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 0, rowdex, 1, 1);
    gtk_widget_show(CamsetWidget[windex]);
    markup = g_markup_printf_escaped (format, ctrl_name);
    gtk_label_set_markup (GTK_LABEL(CamsetWidget[windex]), markup);
    g_free (markup);
    if(next_windex()) return 1;
    return 0;
  }
  
  // If we got here then the control is not just a heading so make an
  // edit box, value labels, etc.
  
  // The actual edit box
  // Create this empty then use put_entry_txt as below otherwise
  // you get critical pango run time warnings:
  camgeb[windex]=gtk_entry_buffer_new (NULL,-1); 
  CamsetWidget[windex]=gtk_entry_new_with_buffer(camgeb[windex]);  cswt_id[windex]= CS_WTYPE_ENTRY;
  gtk_entry_set_input_purpose (GTK_ENTRY(CamsetWidget[windex]),purpose);
  gtk_entry_set_width_chars (GTK_ENTRY(CamsetWidget[windex]),8);
  put_entry_txt((char *)ctrl_value,CamsetWidget[windex]);
  gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
  gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 0, rowdex, 1, 1);
  gtk_widget_show(CamsetWidget[windex]);
  // For controls that are not custom ones supplied by PARD Capture
  // but are read from the camera device itself, it may be possible to
  // allow interactive real-time setting of these with the GUI slider.
  // So create a callback function on the edit box so that the user can
  // select that value to be the one linked to the interactive control
  // slider.
  if(IScompatible)
    g_signal_connect (CamsetWidget[windex], "focus-in-event", G_CALLBACK (cs_edit_click), &windex);

  

  // The label to store the currently applied value as opposed to the
  // the value entered in the edit box that may be waiting to be applied
  if(next_windex()) return 1;   
  CamsetWidget[windex]=gtk_label_new (ctrl_value);  cswt_id[windex]= CS_WTYPE_LABEL;
  gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
  gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 1, rowdex, 1, 1);
  gtk_widget_show(CamsetWidget[windex]);

  // Now create the label that shows the description of the control
  if(next_windex()) return 1;
  CamsetWidget[windex]=gtk_label_new (ctrl_name);  cswt_id[windex]= CS_WTYPE_LABEL;
  gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
  gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
  gtk_widget_show(CamsetWidget[windex]);

  // Get the next widget index ready for the next control.
  if(next_windex()) return 1;
  
  
 return 0;
}

static int add_settings_custom_widget(GtkWidget *cwidget, int *cwidx, const char *ctxt, const char *ltxt)
{
 gchar *numstr;
   *cwidx=0;
   gtk_grid_attach (GTK_GRID (grid_camset), cwidget, 0, rowdex, 1, 1);
   gtk_widget_show(cwidget);
   if(next_windex()) return 1;
   *cwidx = windex; // Make a note that this index is for this widget's current value label
   numstr = g_strdup_printf("%s",ctxt);
   CamsetWidget[*cwidx]=gtk_label_new (numstr);  cswt_id[*cwidx]= CS_WTYPE_LABEL;
   g_free(numstr);
   gtk_widget_set_halign (CamsetWidget[*cwidx], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[*cwidx], 1, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[*cwidx]);    // Show the current value label next to the widget
   if(next_windex()) return 1;
   CamsetWidget[windex]=gtk_label_new (ltxt);  cswt_id[windex]= CS_WTYPE_LABEL; // The widget's desription label
   gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex]);
   rowdex++;
  return 0;
}

static int enumerate_menu(int fdx)
{
 char ctrl_name[64];
 int idx;

 rowdex++;
 CamsetWidget[windex]=gtk_label_new ("    > Menu items:"); cswt_id[windex]= CS_WTYPE_LABEL;
 gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
 gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex++, 1, 1);
 gtk_widget_show(CamsetWidget[windex]);
 if(next_windex()) return 1;

 for(idx=0;idx<CSlist[fdx].num_menuitems;idx++){
    sprintf(ctrl_name,"          %s",CSlist[fdx].miname[idx]);
    CamsetWidget[windex]=gtk_label_new (ctrl_name);  cswt_id[windex]= CS_WTYPE_LABEL;
    gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
    gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex++, 1, 1);
    gtk_widget_show(CamsetWidget[windex]);
    if(next_windex()) return 1;
   } 
 return 0; 
}

static gboolean on_camset_show (GtkWidget *widget, GdkEvent  *event,  gpointer   data)
{
 char ctrl_name[64],ctrl_value[10];
 char fname[PATH_MAX];
 int fdx;
 gchar *numstr;
 GtkInputPurpose ipurpose;

 
 
 windex = windex_gn = windex_bs = windex_camfmt = windex_safmt = 0;
 windex_fps = windex_imroot = 0;
 windex_fno = windex_sz = windex_avd = windex_to = windex_rt = 0;
 windex_srn = windex_srd = windex_jpg = windex_del = 0;
 
 // Read the device's controls and their current values from CSlist
   
 rowdex = 0; // The row in which to display this control information
 IScompatible=1; // All controls retrieved from the camera driver may be 
                 // set with the slider mechanism.

 for(fdx=0;fdx<NCSs;fdx++){
     
    // If the range and step are all zero, this is a title 'control' so
    // I label the input 'purpose' as 'email' just to let me know so I
    // don't try making edit boxes and reading / writing values to it.
    if(CSlist[fdx].minimum == 0 && CSlist[fdx].maximum ==0 && CSlist[fdx].step == 0){
       ipurpose = GTK_INPUT_PURPOSE_EMAIL;
       sprintf(ctrl_name,"\n%s",CSlist[fdx].name);
      } else {
       ipurpose = GTK_INPUT_PURPOSE_NUMBER;
       sprintf(ctrl_name,"%s [Min=%d, Max=%d, Step=%d]",CSlist[fdx].name,CSlist[fdx].minimum,CSlist[fdx].maximum,CSlist[fdx].step);
      }
    sprintf(ctrl_value,"%-7d",CSlist[fdx].currval);

    ctrl_id[windex]= CSlist[fdx].ctrl_id;
    if(add_settings_line_to_gui((const gchar *)ctrl_value, (const gchar *)ctrl_name,ipurpose)) break;

    if(CSlist[fdx].num_menuitems) if(enumerate_menu(fdx)) break;
    rowdex++;   
   }
   
 // If all went well, show the 'Apply' button  
 gtk_widget_show(btn_cs_apply); 

 // Remaining controls are not to be adjusted with the GUI slider
 IScompatible=0;

 // Add the Custom Controls heading
 if(add_settings_line_to_gui((const gchar *)"0", (const gchar *)"\nCustom Controls",GTK_INPUT_PURPOSE_EMAIL)) return TRUE;

 rowdex++;

 // Query the camera to list the device's frame sizes for the chosen
 // camera stream format into the frame size drop-down list
 update_framerate_resolutions();

 // Attach the size combo to the list of settings
 gtk_grid_attach (GTK_GRID (grid_camset), combo_sz, 0, rowdex, 1, 1);

 // Create the frame size current value text label
 if(next_windex()) return TRUE;
 windex_sz = windex; // Make a note that this index is for the size
                     // combo value label
 // Set the dimensions label to the one corresponding to the current HxW
 // or to the VGA entry or 'Pending' if neither of those could be found.
 numstr=gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_sz));
 if(numstr==NULL)  CamsetWidget[windex]=gtk_label_new ("Pending");
   else  CamsetWidget[windex]=gtk_label_new (numstr);
 cswt_id[windex]= CS_WTYPE_LABEL;
 gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
 gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 1, rowdex, 1, 1);
 g_free(numstr);

   show_message("","",MT_INFO,0); // This just prints a newline
                                  // character (for non-GUI notices)
   if(errno != EINVAL) {
       show_message("Error when retrieving frame sizes.","Error: ",MT_ERR,1);
     }

   // Set the default combo entry and make that text the one to show in
   // the settings window as being the current value
   gtk_combo_box_set_active(GTK_COMBO_BOX (combo_sz), CurrDims_idx);
   numstr=gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(combo_sz));
   sprintf(ctrl_name,"%s",numstr);
   g_free(numstr);
   gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_sz]),ctrl_name); 

   gtk_widget_show(combo_sz);                // Show size combo box, set
   gtk_widget_show(CamsetWidget[windex_sz]); // the current WxH & FPS
                                             // value label next to it

   if(next_windex()) return TRUE;
   CamsetWidget[windex]=gtk_label_new ("Image size and FPS for full frame capture");  cswt_id[windex]= CS_WTYPE_LABEL;
   gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex]);
   if(next_windex()) return TRUE;

   rowdex++;

 // Add the Preview fps selection combo and make it visible and create
 // its current value and description labels
   gtk_grid_attach (GTK_GRID (grid_camset), combo_fps, 0, rowdex, 1, 1);
   gtk_widget_show(combo_fps);
   if(next_windex()) return TRUE;
   windex_fps = windex; // Make a note that this index is for the fps combo value label
   numstr = g_strdup_printf("%d",(int)(1000/preview_fps));
   CamsetWidget[windex_fps]=gtk_label_new (numstr);  cswt_id[windex_fps]= CS_WTYPE_LABEL;
   g_free(numstr);
   gtk_widget_set_halign (CamsetWidget[windex_fps], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex_fps], 1, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex_fps]);    // Show the current fps value label next to it
   if(next_windex()) return TRUE;
   CamsetWidget[windex]=gtk_label_new ("Frames per second for live preview");  cswt_id[windex]= CS_WTYPE_LABEL;
   gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex]);
   if(next_windex()) return TRUE;

   rowdex++;

 // Add the camera format selection combo and make it visible and create
 // its current value and description labels
   gtk_grid_attach (GTK_GRID (grid_camset), combo_camfmt, 0, rowdex, 1, 1);
   gtk_widget_show(combo_camfmt);
   if(next_windex()) return TRUE;
   windex_camfmt = windex; // Make a note that this index is for the camera format combo value label
   switch(CamFormat){
       case V4L2_PIX_FMT_YUYV: // Raw YUYV 16 bpp array
         numstr = g_strdup_printf("Raw YUYV");
       break;
       case V4L2_PIX_FMT_MJPEG: // MJPEG stream
         numstr = g_strdup_printf("MJPEG");
       break;
       default: // This should not happen unless the programmer has made a mistake
        show_message("Undefined camera format specified!","Program Error: ",MT_ERR,1);
        return TRUE;
       break;
   }
   CamsetWidget[windex_camfmt]=gtk_label_new (numstr);  cswt_id[windex_camfmt]= CS_WTYPE_LABEL;
   g_free(numstr);
   gtk_widget_set_halign (CamsetWidget[windex_camfmt], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex_camfmt], 1, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex_camfmt]);    // Show the current format value label next to it
   if(next_windex()) return TRUE;
   CamsetWidget[windex]=gtk_label_new ("Format of stream from the camera");  cswt_id[windex]= CS_WTYPE_LABEL;
   gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex]);
   if(next_windex()) return TRUE;

   rowdex++;


 // Add the 'save as' format selection combo and make it visible and
 // create its current value and description labels
   gtk_grid_attach (GTK_GRID (grid_camset), combo_safmt, 0, rowdex, 1, 1);
   gtk_widget_show(combo_safmt);
   if(next_windex()) return TRUE;
   windex_safmt = windex; // Make a note that this index is for the format combo value label
   switch(saveas_fmt){
       case SAF_YUYV: // Raw YUYV 16 bpp array
       case SAF_YP5: // Y-only (greyscale) 8bpp PGM p5
       case SAF_BM8: // Y-only (greyscale) 8bpp BMP
       case SAF_INT: // I-only (greyscale) doubles
       case SAF_RGB: // Full colour 24 bpp RGB PPM P6
       case SAF_BMP: // Full colour 24 bpp RGB BMP
       case SAF_PNG: // Full colour PNG file
       case SAF_JPG: // jpeg image
       break;
       default: // Shouldn't happen unless the programmer made a mistake
        show_message("Undefined save as format specified!","Program Error: ",MT_ERR,1);
        return TRUE;
       break;
   }
   numstr = g_strdup_printf(safmt_options[saveas_fmt]);
   CamsetWidget[windex_safmt]=gtk_label_new (numstr);  cswt_id[windex_safmt]= CS_WTYPE_LABEL;
   g_free(numstr);
   gtk_widget_set_halign (CamsetWidget[windex_safmt], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex_safmt], 1, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex_safmt]);    // Show the current format value label next to it
   if(next_windex()) return TRUE;
   CamsetWidget[windex]=gtk_label_new ("Format to save image files to disk as");  cswt_id[windex]= CS_WTYPE_LABEL;
   gtk_widget_set_halign (CamsetWidget[windex], GTK_ALIGN_START);
   gtk_grid_attach (GTK_GRID (grid_camset), CamsetWidget[windex], 2, rowdex, 1, 1);
   gtk_widget_show(CamsetWidget[windex]);
   if(next_windex()) return TRUE;

   rowdex++;

// Now add edit box for save as file name root.
   windex_imroot = windex;
   add_settings_line_to_gui((const gchar *)ImRoot, "File name root for saved images",GTK_INPUT_PURPOSE_FREE_FORM); rowdex++; 

// Now add edit box for save as file name frame number
   sprintf(ctrl_value,"%-7d",frame_number); windex_fno = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "File name frame number start from",GTK_INPUT_PURPOSE_DIGITS); rowdex++; 

// Now add multiframe averaging setting
   sprintf(ctrl_value,"%-7d",Av_denom);   windex_avd = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Frame averaging (number of frames) [1-4096]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add grabber timeout setting
   sprintf(ctrl_value,"%-7d",Gb_Timeout);   windex_to = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Grabber timeout (seconds) [4-360]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add frame grab retry setting
   sprintf(ctrl_value,"%-7d",Gb_Retry);   windex_rt = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Frame capture (number of retries) [0-4096]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add series number of images setting
   sprintf(ctrl_value,"%-7d",Ser_Number);   windex_srn = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Series (number of images) [1-604800]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add series interval time setting
   sprintf(ctrl_value,"%-7d",Ser_Delay);   windex_srd = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Min. interval for series (s) [0-86400]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add JPEG quality setting
   sprintf(ctrl_value,"%-7d",JPG_Quality);   windex_jpg = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "JPEG save quality [1-100]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add local controls over YUYV conversion
   sprintf(ctrl_value,"%-7f",Gain_conv); windex_gn = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "YUYV conversion gain",GTK_INPUT_PURPOSE_NUMBER); rowdex++; 
   sprintf(ctrl_value,"%-7f",Bias_conv); windex_bs = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "YUYV conversion bias",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 

// Now add the 'Delay first capture by (s)' setting
   sprintf(ctrl_value,"%.0f",Delayed_start_seconds);   windex_del = windex;
   add_settings_line_to_gui((const gchar *)ctrl_value, "Delay first capture by (s) [0-172800]",GTK_INPUT_PURPOSE_NUMBER);rowdex++; 
   
// Now add the 'Use crop from full-size image as preview?' check box,
// make it visible and create its current value and description labels:
   if(add_settings_custom_widget(chk_preview_central, &windex_pc, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_preview_central)))?"Yes":"No",
   "Use crop from full-size image as preview?")) return TRUE;
   
// Now add the 'Preview in monochrome' check box and make it visible and create its current value and description labels
   if(add_settings_custom_widget(chk_cam_yonly, &windex_yo, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_cam_yonly)))?"Yes":"No",
   "Preview in monochrome?")) return TRUE;
       
// Now add the 'Save as raw doubles?' check box and make it
// visible and create its current value and description labels
   if(add_settings_custom_widget(chk_sa_rawdoubles, &windex_sad, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_sa_rawdoubles)))?"Yes":"No",
   "Save as raw doubles?")) return TRUE;
       
// Now add the 'Save as FITS?' check box and make it
// visible and create its current value and description labels
   if(add_settings_custom_widget(chk_sa_fits, &windex_fit, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_sa_fits)))?"Yes":"No",
   "Save as FITS?")) return TRUE;
       
// Now add the 'Scale mean of each frame to first?' check box and make it
// visible and create its current value and description labels
   if(add_settings_custom_widget(chk_scale_means, &windex_smf, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_scale_means)))?"Yes":"No",
   "Scale mean of each frame to first?")) return TRUE;
       
 // Add a separator
   if(add_settings_line_to_gui((const gchar *)"0", (const gchar *)"_________________________\n",GTK_INPUT_PURPOSE_EMAIL)) return TRUE;
   rowdex++;
       
// Now add the 'Apply dark field subtraction?' check box and make it
// visible and create its current value and description labels
   windex_ud2=0;
   if(add_settings_custom_widget(chk_usedfcor, &windex_ud, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usedfcor)))?"Yes":"No",
   "Apply dark field subtraction?")) return TRUE;
   windex_ud2=windex; // The description text - need this so I can make
                      // it 'insensitive' or 'sensitive'
                    
// Now add the 'Load dark field subtraction image' button and create its
// current value and description labels
   if(dffile_loaded==DFIMG_NONE) sprintf(fname,"[None]");
    else sprintf(fname,"%s",name_from_path(DFFile));
   if(add_settings_custom_widget(btn_cs_load_dfri, &windex_rdfi, fname,"Dark field subtraction image")) return TRUE;
   if(dffile_loaded==DFIMG_NONE){
    gtk_widget_set_sensitive (chk_usedfcor,FALSE);
    gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_ud]),"No");
    gtk_widget_set_sensitive (CamsetWidget[windex_ud],FALSE);
    gtk_widget_set_sensitive (CamsetWidget[windex_ud2],FALSE);
   }

     
 // Add a separator
   if(add_settings_line_to_gui((const gchar *)"0", (const gchar *)"_________________________\n",GTK_INPUT_PURPOSE_EMAIL)) return TRUE;
   rowdex++; 
 
        
// Now add the 'Apply flat field division?' check box and make it
// visible and create its current value and description labels
   windex_uf2=0;
   if(add_settings_custom_widget(chk_useffcor, &windex_uf, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_useffcor)))?"Yes":"No",
   "Apply flat field division?")) return TRUE;
   windex_uf2=windex; // The description text - need this so I can make
                      // it 'insensitive' or 'sensitive'
                   
// Now add the 'Load flat field correction image' button and create its
// current value and description labels
   if(fffile_loaded==FFIMG_NONE) sprintf(fname,"[None]");
    else sprintf(fname,"%s",name_from_path(FFFile));
   if(add_settings_custom_widget(btn_cs_load_ffri, &windex_rffi, fname,"Flat field division image")) return TRUE;
   if(fffile_loaded==FFIMG_NONE){
    gtk_widget_set_sensitive (chk_useffcor,FALSE);
    gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_uf]),"No");
    gtk_widget_set_sensitive (CamsetWidget[windex_uf],FALSE);
    gtk_widget_set_sensitive (CamsetWidget[windex_uf2],FALSE);
   }


 // Add a separator
   if(add_settings_line_to_gui((const gchar *)"0", (const gchar *)"_________________________\n",GTK_INPUT_PURPOSE_EMAIL)) return TRUE;
   rowdex++; 
       
// Now add the 'Use correction mask?' check box and make it
// visible and create its current value and description labels
   windex_um2=0;
   if(add_settings_custom_widget(chk_usemskcor, &windex_um, 
   (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_usemskcor)))?"Yes":"No",
   "Use corrections mask?")) return TRUE;
   windex_um2=windex; // The description text - need this so I can make
                      // it 'insensitive' or 'sensitive'
                      
// Now add the 'Corrections mask image' button and create its
// current value and description labels
   if(mskfile_loaded==MASK_NONE) sprintf(fname,"[None]");
    else sprintf(fname,"%s",name_from_path(MaskFile));
   if(add_settings_custom_widget(btn_cs_load_mskri, &windex_rmski, fname,"Corrections mask image")) return TRUE;
   if(mskfile_loaded==MASK_NONE){
    gtk_widget_set_sensitive (chk_usemskcor,FALSE);
    gtk_label_set_text(GTK_LABEL(CamsetWidget[windex_um]),"No");
    gtk_widget_set_sensitive (CamsetWidget[windex_um],FALSE);
    gtk_widget_set_sensitive (CamsetWidget[windex_um2],FALSE);
   }


 // Add the Settings Files heading
   if(add_settings_line_to_gui((const gchar *)"0", (const gchar *)"\nSettings Files",GTK_INPUT_PURPOSE_EMAIL)) return TRUE;

   rowdex++;

// Now add the 'Load settings file' button and create its
// current value and description labels
   if(csetfile_loaded==CSET_NONE) sprintf(fname,"[None]");
    else sprintf(fname,"%s",name_from_path(CSFile));
   if(add_settings_custom_widget(btn_cs_load_cset, &windex_ldcs, fname,"Load a settings file")) return TRUE;

// Now add the 'Save current settings' button and create its
// current value and description labels
   if(csetfile_loaded==CSET_NONE) sprintf(fname,"[None]");
    else sprintf(fname,"%s",name_from_path(CSFile));
   if(add_settings_custom_widget(btn_cs_save_cset, &windex_sacs, fname,"Save current settings")) return TRUE;



 return TRUE;
}

static void hide_remove_from_container(GtkWidget *widget, GtkContainer *container)
{
 gpointer combo_ptr;
  gtk_widget_hide(widget);          // Hide the widget
  combo_ptr = g_object_ref(widget); // Get the reference of the widget because gtk_container_remove will loose the reference of the argument passed to it (and it will effectively be destroyed so can't be used again.)
  gtk_container_remove (container,GTK_WIDGET(combo_ptr)); // Remove the widget from the container (may need to be placed in a different cell if a different set of settings is constructed (e.g. from another camera).
 return;
}

static gboolean on_camset_delete_event (GtkWidget *widget, GdkEvent  *event,  gpointer   data)
{
 int ctrlindex;

 // Ensure the interactive slider is not assigned to a control:
   IScidx=-1; // Camera settings structure for interactive control
   ISwindex=-1;// Edit box widget index for GUI interactive updates
   gtk_label_set_text (GTK_LABEL(ISLabel),"Real-time control of: [None - select a control edit box below]");
   gtk_adjustment_set_value (ISlider_params,0);
   gtk_adjustment_set_step_increment (ISlider_params,1.0);


 // Clear the entries from the size combo (or you will get duplicate
 // entries next time you open the settings window (we use 'append' to
 // add entries). We do this before destroying the text widgets to avoid
 // errors of calling out to non-existant text label widgets
 gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(combo_sz)); 
 
 // First delete text in edit boxes - needed otherwise there are "pango"
 // run-time critical warnings of the type: Pango-CRITICAL **:
 // pango_layout_get_cursor_pos: assertion 'index >= 0 && index <=
 // layout->length' failed
 // If control widgets were created, look for those with edit boxes and
 // delete the edit box text
  if(windex){ 
   for(ctrlindex=0;ctrlindex<windex;ctrlindex++){
       switch(cswt_id[ctrlindex]){
           case CS_WTYPE_ENTRY:
            gtk_editable_delete_text(GTK_EDITABLE(CamsetWidget[ctrlindex]), 0, -1);
           case CS_WTYPE_LABEL:
            gtk_widget_destroy(GTK_WIDGET(CamsetWidget[ctrlindex]));
           break;
           default: break;
        }
        ctrl_id[ctrlindex]=0; // means it does not contain valid entries
        cswt_id[ctrlindex]=CS_WTYPE_UNDEF;
      }
  }
  // Unfade settings button on main window (if we are not in a delayed
  // start countdown):
  if(!Delayed_start_in_progress)
  gtk_widget_set_sensitive (GTK_WIDGET(data),TRUE); 
  // Hide the apply button. No need to remove from grid because it
  // always stays in the same cell:
  gtk_widget_hide(btn_cs_apply); 
  
 // Hide and remove the size selector combo
  hide_remove_from_container(combo_sz,GTK_CONTAINER(grid_camset));
  // Hide the preview fps selector combo
  hide_remove_from_container(combo_fps,GTK_CONTAINER(grid_camset)); 
  // Hide the save as format selector combo
  hide_remove_from_container(combo_camfmt,GTK_CONTAINER(grid_camset));
  // Hide the save as format selector combo 
  hide_remove_from_container(combo_safmt,GTK_CONTAINER(grid_camset)); 
  // Hide the preview crop window selector check box
  hide_remove_from_container(chk_preview_central,GTK_CONTAINER(grid_camset)); 
  // Hide the preview monochrome window selector check box
  hide_remove_from_container(chk_cam_yonly,GTK_CONTAINER(grid_camset)); 
  // Hide the use save as raw doubles selector check box
  hide_remove_from_container(chk_sa_rawdoubles,GTK_CONTAINER(grid_camset)); 
  // Hide the use save as FITS selector check box
  hide_remove_from_container(chk_sa_fits,GTK_CONTAINER(grid_camset)); 
  // Hide the Scale mean of each frame to first? selector check box
  hide_remove_from_container(chk_scale_means,GTK_CONTAINER(grid_camset)); 
  // Hide the use dark field correction selector check box
  hide_remove_from_container(chk_usedfcor,GTK_CONTAINER(grid_camset)); 
  // Hide the dark field correction reference file selector button
  hide_remove_from_container(btn_cs_load_dfri,GTK_CONTAINER(grid_camset));
  // Hide the use flat field correction selector check box 
  hide_remove_from_container(chk_useffcor,GTK_CONTAINER(grid_camset)); 
  // Hide the flat field correction reference file selector button
  hide_remove_from_container(btn_cs_load_ffri,GTK_CONTAINER(grid_camset)); 
  // Hide the use correction mask selector check box 
  hide_remove_from_container(chk_usemskcor,GTK_CONTAINER(grid_camset)); 
  // Hide the correction mask file selector button
  hide_remove_from_container(btn_cs_load_mskri,GTK_CONTAINER(grid_camset)); 
  // Hide the load settings file selector button
  hide_remove_from_container(btn_cs_load_cset,GTK_CONTAINER(grid_camset)); 
  // Hide the save settings file selector button
  hide_remove_from_container(btn_cs_save_cset,GTK_CONTAINER(grid_camset)); 

  gtk_widget_hide(widget);  // Finally, hide the settings window itself.
  return TRUE;
}

static void grab_prev_adjust_value (GtkSpinButton *button, gpointer padjtype)
{
 char msgtxt[256];
 gint ival,adjtype;
 int idx;

 ival = gtk_spin_button_get_value_as_int (button);
 adjtype=*(gint *)padjtype;
 switch(adjtype){
  case PADJUST_INTEGRAL:
   sprintf(msgtxt,"Integrating %d frames for preview",ival);
   show_message(msgtxt,"FYI: ",MT_INFO,0);
   if(ival>1){
   for(idx=1;idx<ival;idx++){
     if(resize_memblk((void **)&PreviewBuff[idx],(size_t)PreviewImg_size,sizeof(int), "the integration buffer")){
         show_message("No RAM available for prime preview buffer.","Error: ",MT_ERR,0);
         Preview_integral=1;
         return;
      }
    }
   if(idx<PREVINTMAX){ // De-alloc memory from unused preview buffers
     for(;idx<PREVINTMAX;idx++){
       if(resize_memblk((void **)&PreviewBuff[idx],(size_t)1,sizeof(int), "the integration buffer")){
         show_message("No RAM available to de-alloc prime preview buffer.","Error: ",MT_ERR,0);
         Preview_integral=1;
         return;
        }
      }
     }
    }

   Preview_integral=ival;
  break;
  case PADJUST_BIAS:
   sprintf(msgtxt,"Biasing preview by %d greyscale units",ival);
   show_message(msgtxt,"FYI: ",MT_INFO,0);
   Preview_bias=ival;
  break;
  default: // Shouldn't get here - programming error
   show_message("Unrecognised preview adjust constant.","Program Error: ",MT_ERR,0);
  break;
 }

 return;
}

static gboolean key_press_event_wm (GtkWidget * widget, GdkEvent * event, gpointer data)
{
 GdkEventKey key = event->key;

  // Take action according to what key is pressed
  switch (key.keyval) {
        case GDK_KEY_g: // Grab ('Save Image') button activate
            if(TRUE==gtk_widget_get_sensitive (btn_cam_save)){
              show_message("g key pressed - activating 'Save image' button.","FYI: ",MT_INFO,0);
              btn_cam_save_click(btn_cam_save,btn_cam_save);
             }
        return TRUE;
        case GDK_KEY_a:
            if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(chk_audio))){
                gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(chk_audio),FALSE);
                Use_audio=AU_NO;
             } else {
                gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON(chk_audio),TRUE);
                Use_audio=AU_YES;
             }
        return TRUE;
        case GDK_KEY_h:
             show_message("Press the 'g' key to activate the 'Save Image' button","Help: ",MT_INFO,0);
             show_message("Press the 'a' key to toggle GUI audio","Help: ",MT_INFO,0);
        return TRUE;
        default:
        break;
    }

    return FALSE;
}

static gboolean on_mainwin_show (GtkWidget *widget, GdkEvent  *event,  gpointer   data)
{
 if(Log_wanted == -1){
   // We must use show_popup directly - not show_message - because the
   // gui_up flag will not have been set at this point (so you won't see
   // a popup window even though you request it) and also this
   // Log_wanted flag needs to be 0 or 1 for proper log handling in the
   // show_message function.
   show_popup("Failed to initialise a log file for this session.\n","ERROR: ");
   Log_wanted=0;
  }
 return TRUE;
}

static gboolean on_delete_event (GtkWidget *widget, GdkEvent  *event,  gpointer   data)
{
  gint choice;
  gboolean returnval;
  
  show_message("Delete event occurred.","Notice!: ",MT_INFO,0);
  // Ask if sure they want to quit. If 'Yes' then return FALSE otherwise (to stay running) return TRUE
  gtk_window_set_title (GTK_WINDOW (dlg_choice), "Quit PARDUS?");
  gtk_message_dialog_set_markup(GTK_MESSAGE_DIALOG(dlg_choice),"Do you really want to quit?");
  choice = gtk_dialog_run(GTK_DIALOG(dlg_choice));
  switch (choice)
  {
    case GTK_RESPONSE_YES:
       returnval = FALSE;
       break;
    default:
       returnval = TRUE;
       break;
  }
  return returnval;
}

static void add_button(GtkWidget **btn,const gchar *label,GtkAlign align)
// Create a new button widget using the supplied parameters
{
    *btn = gtk_button_new_with_label (label);
    gtk_widget_set_size_request(*btn,bt_def_wd,32);
    gtk_widget_set_hexpand (*btn, FALSE);
    gtk_widget_set_halign (*btn, align);
}

static void add_win(GtkWidget **win,const gchar *title,gint wd, gint ht,gboolean sizeable, gboolean modal)
// Create a new window widget using the supplied parameters
{
   *win = gtk_window_new (GTK_WINDOW_TOPLEVEL);
   gtk_window_set_title (GTK_WINDOW (*win), title);
   gtk_window_set_default_size(GTK_WINDOW(*win), wd, ht);
   gtk_container_set_border_width(GTK_CONTAINER (*win), 5);
   gtk_window_set_resizable (GTK_WINDOW(*win),sizeable);
   gtk_window_set_modal (GTK_WINDOW(*win),modal);
   gtk_window_set_position (GTK_WINDOW(*win),GTK_WIN_POS_CENTER);
}

static void add_grid(GtkWidget **grd,GtkWidget **ctn)
// Create a new grid using the supplied parameters
{
    *grd = gtk_grid_new();
    gtk_container_add (GTK_CONTAINER (*ctn), *grd);
    gtk_grid_set_row_spacing (GTK_GRID(*grd),2);
    gtk_grid_set_column_spacing (GTK_GRID(*grd),2);
}

int main(int argc,char *argv[])
{
    GtkWidget *grid_main,*grid_camset_main;
    GtkWidget *vsep1,*vsep2,*vsep3,*vsep4;
    GtkWidget *btn_help_about;
    GtkWidget *scrolwin_camset,*scrolwin_camset_child;
    // For a CSS Provider in memory to colour the preview label eventbox
    // background:
    GtkCssProvider *cssmemprovider_lab_prev; 
    // Screen needed to apply the css properties of the preview label
    // to make them visible.
    GdkScreen *screen;                       
    // This markup stuff is just to make the text red and bold on the
    // Cancel averaging button:
    gchar *btn_markup;
    const char *ca_format = "<span foreground=\"red\" weight=\"bold\">\%s</span>";
    const char *strm_format = "%s <span foreground=\"green\" weight=\"bold\">\%s</span>\n%s";
    GtkWidget *btnlabel;
    GtkAdjustment *preview_integration_adjustment;
    GtkAdjustment *preview_bias_adjustment;
    int prev_int,prev_bias;
    int       idx,gridrow;
    char msgtxt[128];
    time_t ts;


// Check command like options
// Error, wrong number of command arguments
if(argc>3){
args_fail:
  fprintf(stderr,"\nUsage: %s [option] [argument]\n",argv[0]);
  fprintf(stderr,"\n[option] can be: -h for help or -l followed by a file name for logging\n");
  fprintf(stderr,"\nSee the GitHub site for links to a full user manual:\n");
  fprintf(stderr,"\nhttps://github.com/TadPath/PARDUS\n\n");

  exit(1);
 }

if(argc>1){
// User wants help
if(!strcmp(argv[1],"-h")){
  // Print intro and licence info
  printf("\nPARD Capture Stand Alone (%s)\nCopyright (c) 2020-2022 by Dr Paul J. Tadrous\n\n%s\n\n",argv[0],License_note);
  // Print command line options
  printf("\nUsage: %s [option] [argument]\n",argv[0]);
  printf("\n[option] can be: -h for help or -l followed by a file name for logging\n");
  printf("\nSee the GitHub site for links to a full user manual.\n");
  printf("\nhttps://github.com/TadPath/PARDUS\n\n");
  exit(0);
 }

// User wants version info
if(!strcmp(argv[1],"-v")){
  printf("\nPARD Capture (Stand Alone) version "PARDCAP_VERN"\n\n");
  exit(0);
 }

// Check if the user wants log entries and check the usability of their
// chosen log file name
if(!strcmp(argv[1],"-l")){
 if(argc==3){
   sprintf(LogFilename, "%s", argv[2]);
   fplog = fopen(LogFilename, "wb");
   if (fplog) {
                fprintf(fplog, "Log file for PARD Capture session\n");
                fprintf(fplog, "Starting at: %s\n\n", ((time(&ts)) == -1) ? "[Time not available]" : ctime(&ts));
                fflush(fplog); fclose(fplog);
                Log_wanted=1; // User want's logging and they got it
             } else {
                fprintf(stderr,"\nChosen log file could not be accessed for writing (%s)\n",argv[2]);
                Log_wanted = -1; // User wanted logging but the file
                                 // name they selected is unwritable.
                                 // This flag lets us know to send them
                                 // them a pop-up message (once the GUI
                                 // is up) to that effect.
             }
   fplog = NULL; // This is important. Log file open/close functions
                 // check if fplog is NULL.
  } else goto args_fail;
 }
}

 // Print intro and licence info
 printf("\nPARD Capture Stand Alone (%s)\nCopyright (c) 2020-2022 by Dr Paul J. Tadrous\n\n%s\n\n",argv[0],License_note);

 if(Log_wanted==1) printf("\nWriting session info to log file: %s\n", LogFilename);
 else printf("\nNo log file will be written\n");                

// Generate the program's icon
 create_icon();

// Set up the GUI

 gui_up=0;
 if(gui_up==0){ 
    gtk_init(&argc,&argv);

    screen = gdk_screen_get_default();                                   


//====================================================================//
//                                                                    //
//      MAIN WINDOW AND GLOBAL DIALOGUE BOXES                         //
//                                                                    //
//====================================================================//


// Create main window and connect its callback functions:   
   add_win(&Win_main,"TadPath PARD Capture (Stand Alone)",500, 200,FALSE,FALSE);   
   if(PardIcon_ready) gtk_window_set_icon(GTK_WINDOW(Win_main),PardIcon_pixbuf);
   g_signal_connect (Win_main, "delete-event", G_CALLBACK (on_delete_event), NULL);
   g_signal_connect (Win_main, "key-press-event", G_CALLBACK (key_press_event_wm), NULL);
   g_signal_connect (Win_main, "show", G_CALLBACK (on_mainwin_show), NULL); 

   // Next connect the "destroy" event to the gtk_main_quit() function.
   // This signal is emitted when we call gtk_widget_destroy() on the window,
   // or if we return FALSE in the "delete_event" callback.
   g_signal_connect (Win_main, "destroy", G_CALLBACK (gtk_main_quit), NULL);

// Create a global dialog box used to request a binary choice:      
    dlg_choice = gtk_message_dialog_new (GTK_WINDOW(Win_main),
                                          GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                          GTK_MESSAGE_QUESTION,
                                          GTK_BUTTONS_YES_NO,
                                          "%s","Please choose");
    gtk_window_set_title (GTK_WINDOW (dlg_choice), "What do you choose?");
    g_signal_connect_swapped (dlg_choice,"response",G_CALLBACK (gtk_widget_hide),dlg_choice);
    
// Create a global dialog box used to provide information only:     
    dlg_info = gtk_message_dialog_new (GTK_WINDOW(Win_main),
                                          GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                          GTK_MESSAGE_INFO,
                                          GTK_BUTTONS_OK,
                                          "%s","Just to let you know.");
    gtk_window_set_title (GTK_WINDOW (dlg_info), "F.Y.I.");
    g_signal_connect_swapped (dlg_info,"response",G_CALLBACK (gtk_widget_hide),dlg_info);
 
// Create the interactive slider control for the settings window:
   ISlider_params = gtk_adjustment_new (0.0,0.0,11,1.0,1.0,1.0);
   ISlider = gtk_scale_new (GTK_ORIENTATION_HORIZONTAL,ISlider_params);
   gtk_scale_set_digits (GTK_SCALE(ISlider),0);
   ISLabel=gtk_label_new ("Real-time control of: [None - select a control edit box below]");
   IScidx=-1;  // Camera settings structure for interactive control
   ISwindex=-1;// Edit box widget index for GUI interactive updates
   g_signal_connect (ISlider_params, "value-changed", G_CALLBACK (is_change_click), ISlider_params);


//====================================================================//
//                                                                    //
//                          CAMERA TASKS                              //
//                                                                    //
//====================================================================//

   // The camera status label
   lab_cam_status=gtk_label_new ("Closed");
   gtk_widget_set_halign (lab_cam_status, GTK_ALIGN_START);


// Create the buttons for basic operations connect their callback functions:
    // Camera tasks buttons
    bt_def_wd=110;
    add_button(&btn_cam_settings,"Settings",GTK_ALIGN_CENTER);
    g_signal_connect (btn_cam_settings, "clicked", G_CALLBACK (btn_cam_settings_click), btn_cam_settings);

    add_button(&btn_cam_stream," Turn ON\nStreaming",GTK_ALIGN_CENTER);
    g_signal_connect (btn_cam_stream, "clicked", G_CALLBACK (btn_cam_stream_click), btn_cam_stream);
    btn_markup = g_markup_printf_escaped (strm_format, "Turn","ON","Streaming");
    btnlabel = gtk_bin_get_child(GTK_BIN(btn_cam_stream));
    gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
    g_free (btn_markup);

    add_button(&btn_av_interrupt,"CANCEL\nAveraging",GTK_ALIGN_CENTER);
    g_signal_connect (btn_av_interrupt, "clicked", G_CALLBACK (btn_av_interrupt_click), btn_av_interrupt);
    btn_markup = g_markup_printf_escaped (ca_format, "CANCEL\nAveraging");
    btnlabel = gtk_bin_get_child(GTK_BIN(btn_av_interrupt));
    gtk_label_set_markup(GTK_LABEL(btnlabel), btn_markup);
    g_free (btn_markup);

    add_button(&btn_cam_save,"Save Image",GTK_ALIGN_CENTER);
    g_signal_connect (btn_cam_save, "clicked", G_CALLBACK (btn_cam_save_click), btn_cam_save);
    add_button(&btn_help_about,"Help/About",GTK_ALIGN_END);
    g_signal_connect (btn_help_about, "clicked", G_CALLBACK (btn_help_about_click), btn_help_about);
    
   // Preview frame integration control
   preview_integration_adjustment = gtk_adjustment_new (1.0, 1.0, PREVINTMAX, 1.0, 1.0, 0.0);
   preview_integration_sbutton = gtk_spin_button_new (preview_integration_adjustment, 1.0, 0);
   prev_int=PADJUST_INTEGRAL;
   g_signal_connect (GTK_SPIN_BUTTON (preview_integration_sbutton), "value-changed", G_CALLBACK (grab_prev_adjust_value), &prev_int);
   gtk_widget_set_valign (preview_integration_sbutton, GTK_ALIGN_START);
   prev_int_label=gtk_label_new ("Preview Integral");
   gtk_widget_set_valign (prev_int_label, GTK_ALIGN_END);
    
   // Preview frame bias control
   // The range is from -(PREVINTMAX-1)*255 because the maximum preview
   // integrtion is PREVINTMAX frames. This means the maximum grey scale
   // value achievable with maximum integration is 255xPREVINTMAX.
   // Assuming you want this maximum to be displayed as white (e.g. 255)
   // you need to bias it with -(255xPREVINTMAX)+255 i.e.
   // PREVINTMAX x 255 - 255 i.e. -(PREVINTMAX-1)*255. 
   preview_bias_adjustment = gtk_adjustment_new (0.0,(double)(-(PREVINTMAX-1)*255), 512.0, 1.0, 10.0, 0.0);
   preview_bias_sbutton = gtk_spin_button_new (preview_bias_adjustment, 1.0, 0);
   prev_bias=PADJUST_BIAS;
   g_signal_connect (GTK_SPIN_BUTTON (preview_bias_sbutton), "value-changed", G_CALLBACK (grab_prev_adjust_value), &prev_bias);
   gtk_widget_set_valign (preview_bias_sbutton, GTK_ALIGN_START);
   prev_bias_label=gtk_label_new ("Preview Bias");
   gtk_widget_set_valign (prev_bias_label, GTK_ALIGN_END);

   // Preview master dark button
   add_button(&preview_corr_button,"Load P.Dark",GTK_ALIGN_CENTER);
   g_signal_connect (preview_corr_button, "clicked", G_CALLBACK (btn_io_prev_corrfield_click), preview_corr_button);


   // Preview on/off toggle control
   chk_cam_preview = gtk_check_button_new_with_label("Preview");
   gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_cam_preview), FALSE);
   g_signal_connect (GTK_TOGGLE_BUTTON (chk_cam_preview), "toggled", G_CALLBACK (toggled_cam_preview), chk_cam_preview);
   gtk_widget_set_halign (chk_cam_preview, GTK_ALIGN_START);
    
   // Use beeps on/off toggle control
   chk_audio = gtk_check_button_new_with_label("Use beeps");
   gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_audio), FALSE);
   g_signal_connect (GTK_TOGGLE_BUTTON (chk_audio), "toggled", G_CALLBACK (toggled_audio), chk_audio);
   gtk_widget_set_halign (chk_audio, GTK_ALIGN_START);
    
// Create the image preview box widgets and timeout and allocate memory
// to the preview image pixel array and sub-sampler arrays:
   PreviewWd = 640;
   PreviewHt = 480;
   PreviewImg_size = PreviewHt*PreviewWd;
   PreviewWd_stride = 3*PreviewWd;
   PreviewImg_rgb_size = PreviewHt*PreviewWd_stride;
   PreviewImg = (unsigned char *)calloc(PreviewImg_rgb_size,sizeof(unsigned char));
   if(PreviewImg==NULL){
         show_message("No RAM available for preview image pixels.","Error: ",MT_ERR,0);
         return 1;
    }

   Preview_dark = (double *)calloc(PreviewImg_size,sizeof(double));
   if(Preview_dark==NULL){
         show_message("No RAM available for preview image master dark.","Error: ",MT_ERR,0);
         return 1;
    }
   for(idx=0;idx<PreviewImg_size;idx++) Preview_dark[idx]=0.0;

   Preview_flat = (double *)calloc(PreviewImg_size,sizeof(double));
   if(Preview_flat==NULL){
         show_message("No RAM available for preview image master dark.","Error: ",MT_ERR,0);
         return 1;
    }
   for(idx=0;idx<PreviewImg_size;idx++) Preview_flat[idx]=1.0;

   PreviewRow = (unsigned char *)calloc(PreviewWd_stride,sizeof(unsigned char));
   if(PreviewRow==NULL){
         show_message("No RAM available for preview row buffer.","Error: ",MT_ERR,0);
         return 1;
    }

   SSrow = (int *)calloc(PreviewHt,sizeof(int));
   if(SSrow==NULL){
         show_message("No RAM available for preview row sampler.","Error: ",MT_ERR,0);
         return 1;
    }
   SScol = (int *)calloc(PreviewWd,sizeof(int));
   if(SScol==NULL){
         show_message("No RAM available for preview col sampler.","Error: ",MT_ERR,0);
         return 1;
    }
   for(idx=0;idx<PREVINTMAX;idx++){
    PreviewBuff[idx] = (int *)calloc(1,sizeof(int));
     if(PreviewBuff[idx]==NULL){
         show_message("No RAM available for preview buffers.","Error: ",MT_ERR,0);
         return 1;
      }
   }
  // We must have at least one preview frame buffer alloced
  if(resize_memblk((void **)&PreviewBuff[0],(size_t)PreviewImg_size,sizeof(int), "the integration buffer")){
       show_message("No RAM available for prime preview buffer.","Error: ",MT_ERR,0);
       return 1;
      }
   Preview_integral=1;
   PreviewIDX=0;
   Preview_bias=0; 

   gdkpb_preview = gdk_pixbuf_new_from_data (PreviewImg,GDK_COLORSPACE_RGB,FALSE,8,PreviewWd,PreviewHt,PreviewWd_stride,NULL,NULL);
                          
   Img_preview  = gtk_image_new_from_pixbuf (gdkpb_preview);
   Ebox_preview = gtk_event_box_new ();
   gtk_container_add (GTK_CONTAINER (Ebox_preview), Img_preview);
   // Make an overlay container for the preview so we can overlay text
   // on it.
   Overlay_preview = gtk_overlay_new ();
   gtk_container_add (GTK_CONTAINER (Overlay_preview), Ebox_preview);
   // Make a label to overlay text on the preview
   Label_preview = gtk_label_new (" Preview is OFF ");
   // Make an event box for the preview label so we can set a background
   // colour
   Ebox_lab_preview = gtk_event_box_new (); 
   gtk_container_add (GTK_CONTAINER (Ebox_lab_preview), Label_preview);
   // Set the colour and style of the label's font and ebox background
   gtk_widget_set_name (Ebox_lab_preview, "preview-label-ebox");
   cssmemprovider_lab_prev = gtk_css_provider_new ();
   gtk_css_provider_load_from_data (cssmemprovider_lab_prev,
                                   "#preview-label-ebox {\n"
                                   "  color: rgba(100%,100%,10%,1.0);\n"
                                   "  background-color: rgba(10%,10%,100%,0.5);\n"
                                   "  font-family: Sans;\n"
                                   "  font-size: 18px;\n"
                                   "}",
                                   -1,NULL);
   gtk_style_context_add_provider_for_screen(screen, GTK_STYLE_PROVIDER (cssmemprovider_lab_prev),GTK_STYLE_PROVIDER_PRIORITY_APPLICATION + 1);
   // End of code to set colour/style ,etc.                                           
  
   gtk_widget_set_halign (Ebox_lab_preview, GTK_ALIGN_CENTER);
   gtk_widget_set_valign (Ebox_lab_preview, GTK_ALIGN_CENTER);
   gtk_overlay_add_overlay (GTK_OVERLAY(Overlay_preview),Ebox_lab_preview);
   
   g_signal_connect (G_OBJECT (Ebox_preview),"button_press_event", G_CALLBACK (img_preview_click),Img_preview);
   g_signal_connect (G_OBJECT (Ebox_lab_preview),"button_press_event", G_CALLBACK (elp_preview_click),NULL);
   gtk_widget_set_halign(Ebox_preview,GTK_ALIGN_CENTER);
   gtk_widget_set_halign(Img_preview,GTK_ALIGN_CENTER);
   gtk_widget_set_valign(Img_preview,GTK_ALIGN_CENTER);
   gtk_widget_set_margin_start (Img_preview,0);
   gtk_widget_set_margin_top (Img_preview,0);
   preview_fps=100;
   if(g_timeout_add(preview_fps, G_SOURCE_FUNC(update_cam_preview),NULL)){
        sprintf(msgtxt,"Preview timeout created at %dms interrvals (10 fps).",preview_fps);
        show_message(msgtxt,"FYI: ",MT_INFO,0);
    }


//====================================================================//
//                                                                    //
//     GRID AND PLACEMENT FOR MAIN WINDOW                             //
//                                                                    //
//====================================================================//

// Create vertical line separator widgets:      
    vsep1 = gtk_separator_new (GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_hexpand (vsep1, FALSE);
    gtk_widget_set_halign (vsep1, GTK_ALIGN_CENTER);
    vsep2 = gtk_separator_new (GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_hexpand (vsep2, FALSE);
    gtk_widget_set_halign (vsep2, GTK_ALIGN_CENTER);
    vsep3 = gtk_separator_new (GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_hexpand (vsep3, FALSE);
    gtk_widget_set_halign (vsep3, GTK_ALIGN_CENTER);
    vsep4 = gtk_separator_new (GTK_ORIENTATION_VERTICAL);
    gtk_widget_set_hexpand (vsep4, FALSE);
    gtk_widget_set_halign (vsep4, GTK_ALIGN_CENTER);

// Construct the grid container used to pack (arrange) our widgets in the main window
  add_grid(&grid_main,&Win_main);

  gridrow=0;
  gtk_grid_attach (GTK_GRID (grid_main), gtk_label_new ("CAMERA TASKS"),   0, gridrow++, 12,1);
  gtk_grid_attach (GTK_GRID (grid_main), gtk_label_new ("Camera Ops"),     0, gridrow,   2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), gtk_label_new ("Camera status:"), 6, gridrow,   1, 1);
  gtk_grid_attach (GTK_GRID (grid_main), lab_cam_status,                   7, gridrow,   5, 1);
  gtk_grid_attach (GTK_GRID (grid_main), btn_help_about,                   8, gridrow++, 5, 1);
  gtk_grid_attach (GTK_GRID (grid_main), btn_cam_settings,                 0, gridrow,   2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), btn_cam_stream,                   0, gridrow+1, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), prev_int_label,                   0, gridrow+2, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), preview_integration_sbutton,      0, gridrow+3, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), prev_bias_label,                  0, gridrow+4, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), preview_bias_sbutton,             0, gridrow+5, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), preview_corr_button,              0, gridrow+6, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), chk_cam_preview,                  0, gridrow+7, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), chk_audio,                        0, gridrow+8, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), btn_av_interrupt,                 0, gridrow+9, 2, 1);
  gtk_grid_attach (GTK_GRID (grid_main), btn_cam_save,                     0, gridrow+10, 2, 1);
  // Increment / decrement the last figure below when adding / removing
  // components from the main left control panel.  
  gtk_grid_attach (GTK_GRID (grid_main), Overlay_preview, 6, gridrow, 7, 11);

// Show the widgets in the main window (and hide exceptions):
    gtk_widget_show_all(Win_main);
    gtk_widget_hide(btn_av_interrupt);
    gtk_widget_hide(prev_int_label);
    gtk_widget_hide(prev_bias_label);
    gtk_widget_hide(preview_integration_sbutton);
    gtk_widget_hide(preview_bias_sbutton);
    gtk_widget_hide(preview_corr_button);
    
        

//====================================================================//
//                                                                    //
//   CREATE THE COMMON COMPONENTS OF THE CAMERA SETTINGS WINDOW       //
//                                                                    //
//====================================================================//
    


// Create camera settings window and connect its callback functions:    
   add_win(&win_cam_settings,"PARDUS Camera Settings",640, 320,TRUE,FALSE); 
   if(PardIcon_ready) gtk_window_set_icon(GTK_WINDOW(win_cam_settings),PardIcon_pixbuf);
   g_signal_connect (win_cam_settings, "delete-event", G_CALLBACK (on_camset_delete_event), btn_cam_settings);
   g_signal_connect (win_cam_settings, "show", G_CALLBACK (on_camset_show), NULL); 

// Construct the grid container used to pack (arrange) our widgets in the camera settings window
   add_grid(&grid_camset_main,&win_cam_settings);
   scrolwin_camset_child=gtk_frame_new ("Available Settings for this Camera");
   gtk_widget_set_size_request (scrolwin_camset_child, 640, 320);
   gtk_widget_set_hexpand (scrolwin_camset_child, TRUE);
   gtk_widget_set_vexpand (scrolwin_camset_child, TRUE);

   // Create the button widgets for camera settings:        
   bt_def_wd=64;
   add_button(&btn_cs_apply,"Apply All Settings",GTK_ALIGN_END); // Apply settings
   g_signal_connect (btn_cs_apply, "clicked", G_CALLBACK (btn_cs_apply_click), NULL);
   gtk_widget_set_halign (btn_cs_apply, GTK_ALIGN_CENTER);
    
   // Select the image to use as dark field correction reference image
   add_button(&btn_cs_load_dfri,"Select",GTK_ALIGN_START);
   g_signal_connect (btn_cs_load_dfri, "clicked", G_CALLBACK (btn_cs_load_darkfield_click), NULL);
    
   // Select the image to use as flat field correction reference image
   add_button(&btn_cs_load_ffri,"Select",GTK_ALIGN_START);
   g_signal_connect (btn_cs_load_ffri, "clicked", G_CALLBACK (btn_cs_load_flatfield_click), NULL);
    
   // Select the image to use as corrections mask
   add_button(&btn_cs_load_mskri,"Select",GTK_ALIGN_START);
   g_signal_connect (btn_cs_load_mskri, "clicked", G_CALLBACK (btn_cs_load_mask_click), NULL);
   
   // Load settings file button
   add_button(&btn_cs_load_cset,"Load...",GTK_ALIGN_START);
   g_signal_connect (btn_cs_load_cset, "clicked", G_CALLBACK (btn_cs_load_cset_click), NULL);
   
   // Save settings file button
   add_button(&btn_cs_save_cset,"Save...",GTK_ALIGN_START);
   g_signal_connect (btn_cs_save_cset, "clicked", G_CALLBACK (btn_cs_save_cset_click), NULL);
    
    // Create the frame size selection combo 
    combo_sz = gtk_combo_box_text_new(); 
     
    // Create the preview frames per second selection combo
    // Create the combo box and append your string values to it.
    combo_fps = gtk_combo_box_text_new ();
    for(idx = 0; idx < G_N_ELEMENTS (fps_options); idx++){
        gtk_combo_box_text_append_text (GTK_COMBO_BOX_TEXT (combo_fps), fps_options[idx]);
       }
    // Choose the default item index to display from the beginning
    gtk_combo_box_set_active (GTK_COMBO_BOX (combo_fps), 6);
     
    // Create the camera format selection combo
    // Create the combo box and append your string values to it.
    combo_camfmt = gtk_combo_box_text_new ();
    for(idx = 0; idx < Ncamfs; idx++){
        gtk_combo_box_text_append_text (GTK_COMBO_BOX_TEXT (combo_camfmt), camfmt_options[idx]);
       }
    // Choose the default item index to display from the beginning
    gtk_combo_box_set_active (GTK_COMBO_BOX (combo_camfmt), 0);

    // Create the save as format selection combo
    // Create the combo box and append your string values to it.
    combo_safmt = gtk_combo_box_text_new ();
    for(idx = 0; idx < Nsafs; idx++){
        gtk_combo_box_text_append_text (GTK_COMBO_BOX_TEXT (combo_safmt), safmt_options[idx]);
       }
    // Choose the default item index to display from the beginning
    gtk_combo_box_set_active (GTK_COMBO_BOX (combo_safmt), SAF_BMP);

    // Create the 'use central crop as preview' check box
    chk_preview_central = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_preview_central), FALSE);
    gtk_widget_set_halign (chk_preview_central, GTK_ALIGN_CENTER);

    // Create the monochrome preview option check box
    chk_cam_yonly = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_cam_yonly), FALSE);
    gtk_widget_set_halign (chk_cam_yonly, GTK_ALIGN_CENTER);

    // Create the save as raw doubles option check box
    chk_sa_rawdoubles = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_rawdoubles), FALSE);
    gtk_widget_set_halign (chk_sa_rawdoubles, GTK_ALIGN_CENTER);

    // Create the save as FITS option check box
    chk_sa_fits = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_sa_fits), FALSE);
    gtk_widget_set_halign (chk_sa_fits, GTK_ALIGN_CENTER);

    // Create the Scale mean of each frame to first? option check box
    chk_scale_means = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_scale_means), FALSE);
    gtk_widget_set_halign (chk_scale_means, GTK_ALIGN_CENTER);

    // Create the dark field correction option check box
    chk_usedfcor = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usedfcor), FALSE);
    gtk_widget_set_halign (chk_usedfcor, GTK_ALIGN_CENTER);
    gtk_widget_set_sensitive (chk_usedfcor,FALSE);

    // Create the flat field correction option check box
    chk_useffcor = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_useffcor), FALSE);
    gtk_widget_set_halign (chk_useffcor, GTK_ALIGN_CENTER);
    gtk_widget_set_sensitive (chk_useffcor,FALSE);
 
    // Create the corrections mask option check box
    chk_usemskcor = gtk_check_button_new();
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (chk_usemskcor), FALSE);
    gtk_widget_set_halign (chk_usemskcor, GTK_ALIGN_CENTER);
    gtk_widget_set_sensitive (chk_usemskcor,FALSE);
 

    grid_camset = gtk_grid_new();
    scrolwin_camset = gtk_scrolled_window_new  (NULL,NULL);
    gtk_container_add (GTK_CONTAINER (scrolwin_camset), grid_camset);
    gtk_container_add (GTK_CONTAINER (scrolwin_camset_child), scrolwin_camset);
    //  gtk_container_add (GTK_CONTAINER (scrolwin_camset_child), grid_camset);
    gtk_grid_set_row_spacing (GTK_GRID(grid_camset),2);
    gtk_grid_set_column_spacing (GTK_GRID(grid_camset),5);
    // Place the 'Apply' button right at the top 
    gtk_grid_attach (GTK_GRID (grid_camset_main), btn_cs_apply, 0, 0, 1, 1);
    // Place the interactive slider under this 
    gtk_grid_attach (GTK_GRID (grid_camset_main), ISLabel, 0, 1, 1, 1);
    gtk_grid_attach (GTK_GRID (grid_camset_main), ISlider, 0, 2, 1, 1);
    // Place the scroll window for the generated settings under that 
    gtk_grid_attach (GTK_GRID (grid_camset_main), scrolwin_camset_child, 0, 3, 1, 1);
    // Clear these variables associated with housekeeping for the camera settings window GUI
    for(idx=0;idx<MAX_CAM_SETTINGS;idx++){ 
      ctrl_id[idx]=0; // means it does not contain valid entries
      cswt_id[idx]=CS_WTYPE_UNDEF;
    }  
 
}


// Set the default values:

  // Camera-related defaults
  camera_status.cs_opened = 0;
  camera_status.cs_initialised = 0;
  camera_status.cs_streaming = 0;
  // Set size paramters and allocate memory for a basic VGA-sized image:
  Selected_Ht=480;
  Selected_Wd=640;
  RGBimg=(unsigned char *)calloc(1,sizeof(unsigned char));
  Frmr=(double *)calloc(1,sizeof(double));
  Frmg=(double *)calloc(1,sizeof(double));
  Frmb=(double *)calloc(1,sizeof(double));
  Frame_status=FRM_FREED;
  if(RGBimg==NULL){
         show_message("No RAM available for main image.","Error: ",MT_ERR,0);
         return 1;
   }
  if(Frmr==NULL || Frmg==NULL || Frmb==NULL){
         show_message("No RAM available for frame buffers.","Error: ",MT_ERR,0);
         return 1;
   }

  // If we can't get enough RAM even for this basic small image there is
  // no point in continuing:
  if(set_dims_as_per_selected()) return 1; 

  Need_to_save = 0; // Set to 1 when the save image button is clicked
                    // and reset to 0 after the image save attampt is
                    // made (whether successful or not).
                    
  // Initialise memory for the frame averaging stores. We set the FREED
  // flag because that only changes to ALLOCED when they have enough
  // storage alloced to be used for accumulation of full frame images.
  Accumulator_status=ACC_FREED;
  Avr=(double *)calloc(1,sizeof(double));
  Avg=(double *)calloc(1,sizeof(double));
  Avb=(double *)calloc(1,sizeof(double));
  if(Avr==NULL || Avg==NULL || Avb==NULL){
         show_message("No RAM available for averaging accumulators.","Error: ",MT_ERR,0);
         return 1;
   }

  
  // Set default image save file name string
  ImRoot = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(ImRoot==NULL){
         show_message("No RAM available for image root name.","Error: ",MT_ERR,0);
         return 1;
   }
  sprintf(ImRoot,"frame");
  
  // Set default flat field reference image file name string
  FFFile = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(FFFile==NULL){
         show_message("No RAM available for flat field correction image file name.","Error: ",MT_ERR,0);
         return 1;
   }
  sprintf(FFFile,"/"); // Initialising to root - will need to change if
                       // porting to non-*ix OS

  // Set default dark field reference image file name string
  DFFile = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(DFFile==NULL){
         show_message("No RAM available for dark field correction image file name.","Error: ",MT_ERR,0);
         return 1;
   }
  sprintf(DFFile,"/"); // Initialising to root - will need to change if
                       // porting to non-*ix OS
  
  // Set default camera settings file name string
  CSFile = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(CSFile==NULL){
         show_message("No RAM available for camera settings file name.","Error: ",MT_ERR,0);
         return 1;
   }
  sprintf(CSFile,"/"); // Initialising to root - will need to change if
                       // porting to non-*ix OS
  csetfile_loaded = CSET_NONE;
  
  // Set default mask image file name string
  MaskFile = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(MaskFile==NULL){
         show_message("No RAM available for mask image file name.","Error: ",MT_ERR,0);
         return 1;
   }
  sprintf(MaskFile,"/"); // Initialising to root - will need to change
                         // if porting to non-*ix OS
  
  // Initialise memory for the flat field correction image
  FF_Image = (double *)calloc(2,sizeof(double));
  if(FF_Image==NULL){
    show_message("No RAM available for flat field correction image.","Error: ",MT_ERR,0);
    return 1;
   }
  fffile_loaded = FFIMG_NONE;
  ffcorr_status = FFCORR_OFF;
  ff_pending=0;
  FFht=FFwd=0;

  // Initialise memory for the dark field correction image
  DF_Image = (double *)calloc(2,sizeof(double));
  if(DF_Image==NULL){
    show_message("No RAM available for dark field correction image.","Error: ",MT_ERR,0);
    return 1;
   }
  dffile_loaded = DFIMG_NONE;
  dfcorr_status = DFCORR_OFF;
  df_pending=0;
  DFht=DFwd=0;
  
  
  // Initialise memory for the corrections mask image
  MaskIm = (unsigned char *)calloc(2,sizeof(unsigned char));
  if(MaskIm==NULL){
    show_message("No RAM available for corrections mask image.","Error: ",MT_ERR,0);
    return 1;
   }
  mskfile_loaded = MASK_NONE;
  mask_status = 0;
  msk_pending=0;
  MKht=MKwd=0;
  Mask_supp_size=1.0;
  mask_alloced=MASK_NO;
  sprintf(Selected_Mask_filename,"[None]");
  
  // Set the default image capture device to /dev/video0
  dev_name = (char *)calloc(FILENAME_MAX,sizeof(char));
  if(dev_name==NULL){
         show_message("No RAM available for device name.","Error: ",MT_ERR,1);
         return 1;
   }
  sprintf(dev_name,"/dev/video0");

  // Set image colour format defaults   
  CamFormat = V4L2_PIX_FMT_YUYV;
  FormatForbidden=CAF_ALLOK;
  col_conv_type=CCOL_TO_RGB ;
  saveas_fmt = SAF_BMP;
  
  Need_to_preview=PREVIEW_OFF; // This is changed by the preview
                               // checkbox and some other functions.
                               // There are different preview modes -
                               // see #defines for details.
  Preview_fullsize = 0; // If set to 1 (via a Camera settings window
                        // check box) then the preview image is a crop
                        // from the full size captured image (rather
                        // than a downsampled version of the whole
                        // image). If the full-sized image is <= preview
                        // resolution then the 'crop' is the whole image
                        // otherwise the user can select which preview-
                        // sized 'tile' of the full-size image to
                        // display in the preview area using the GUI.
  selected_Preview_fullsize = 0; // This lets us know when the user
                                 // changes the Preview_fullsize option
                                 // so we can make the required
                                 // dimension adjustments.
  Preview_tile_selection_made = 0; // Set when the user clicks in the 
                                   // preview window to select a tile
                                   // when in Preview_fullsize mode.
  Preview_changed = 1;  // We need to set parameters at the start.

  Preview_impossible=calculate_preview_params();


  
 // Allocate memory for the 7 YUYV to RGB conversion LUTs
 lut_yR = (double *)calloc(256,sizeof(double));
  if(lut_yR==NULL){show_message("No RAM for lut_yR.","Error: ",MT_ERR,0);return 1;}
 lut_yG = (double *)calloc(256,sizeof(double));
  if(lut_yG==NULL){show_message("%s: No RAM for lut_yG.","Error: ",MT_ERR,0);return 1;}
 lut_yB = (double *)calloc(256,sizeof(double));
  if(lut_yB==NULL){show_message("%s: No RAM for lut_yB.","Error: ",MT_ERR,0);return 1;}
 lut_crR = (double *)calloc(256,sizeof(double));
  if(lut_crR==NULL){show_message("%s: No RAM for lut_crR.","Error: ",MT_ERR,0);return 1;}
 lut_crG = (double *)calloc(256,sizeof(double));
  if(lut_crG==NULL){show_message("%s: No RAM for lut_crG.","Error: ",MT_ERR,0);return 1;}
 lut_cbB = (double *)calloc(256,sizeof(double));
  if(lut_cbB==NULL){show_message("%s: No RAM for lut_cbB.","Error: ",MT_ERR,0);return 1;}
 lut_cbG = (double *)calloc(256,sizeof(double));
  if(lut_cbG==NULL){show_message("%s: No RAM for lut_cbG.","Error: ",MT_ERR,0);return 1;}
 luts_alloced = 1;

// Default values for the gain and bias for the YUYV to RGB transform.
// If these change, the LUTs must be recalculated.
 Gain_conv = 1.0; //0.5;    
 Bias_conv = 0.0; //192.0;
                           
// Prepare for image colourspace conversion.
 calculate_yuyv_luts(); 
 
// Delayed capture start variables
 Delayed_start_on=0;
 Delayed_start_in_progress=0;
 Delayed_start_seconds=0.0; 


 // Initialise Audio
#ifndef M_PI
#define M_PI  (3.14159265)
#endif
 Audio_status=AS_NULL;
 Use_audio=AU_NO; // Until the user selects it (if it is possible)
 Audio_sounding=0;
 // Set the basic frequency
 for( idx=0; idx<PABUFSIZE; idx++ ){
      sine_buffer[idx] =  (uint8_t)(250.0 * sin(((double)idx/DPABUFSIZE) * M_PI * 2.0) );
     }

// Start the GUI main loop - no code beyond this point will be executed
// until the main window is closed.
 gui_up=1;    // Let all functions know the GUI is up
 gtk_main (); // Start the GUI main loop.
    
// The main GUI has been quit.
// Now run any tidy up code prior to exiting completely.
 gui_up=0; // Let any function know not to try to alter any widgets at
           // this point.
 tidy_up();
    
 return 0;
}

    
    
    
