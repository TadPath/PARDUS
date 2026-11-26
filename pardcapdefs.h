/******************************************************************************/
/* pardcapdefs.h v1.0                                                         */
/*                                                                            */
/* Common definitions for C programs that communicate with the PARDcap        */
/* server over socket API comms.                                              */
/*                                                  P. J. Tadrous 04.08.2025  */
/******************************************************************************/

#ifndef pardcapdefs_h__
#define pardcapdefs_h__

////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                          ERROR CODES                           /////////
//                                                                            //

// Return values from ps_check_blob
#define PS_CB_OK 0     // No error, all OK. This must be defined as value 0
// Values 0 to 63 are reserved for PARDU_ERR codes (errors from the Arduino).
// Values 64 to 128 are reserved for PARD_ERR codes (PARD server/client).
// So do not use them for PS_CB_E_ constants.
#define PS_CB_E_BSZ  129 // Blob too small for correct format requirements
#define PS_CB_E_VER  130 // Version error
#define PS_CB_E_IID  131 // Core instruction ID error (invalid CIID)
#define PS_CB_E_SID  132 // Sub-instruction ID error (invalid siid)
#define PS_CB_E_SIF  133 // Invalid image format
#define PS_CB_E_BIN  134 // Failure to provide a 1 or 0 binary choice value
#define PS_CB_E_MES  135 // Failure to get memory for the instruction string
#define PS_CB_E_MEB  136 // Failure to get memory for the instruction blob
#define PS_CB_E_MEE  137 // Failure to get memory for the encapsulated blob
#define PS_CB_E_PFC  138 // Preview failure - colour convert failed
#define PS_CB_E_PFZ  139 // Preview failure - zoom settings failed
#define PS_CB_E_DIM  140 // Wrong image dimensions supplied for intended use
#define PS_CB_E_NOM  141 // Wrong image name string format for intended use
#define PS_CB_E_SAV  142 // Failed to save image to local disc
#define PS_CB_E_LDL  143 // Failed to load image from local disc
#define PS_CB_E_FSR  144 // Failed to apply a setting due to out of range
#define PS_CB_E_CSE  145 // Checksum error
#define PS_CB_E_NTC  146 // No tissue content
#define PS_CB_E_NBC  147 // No background correction
#define PS_CB_E_GRB  148 // Image grab error
#define PS_CB_E_CAM  149 // Problem running or accessing the camera
#define PS_CB_E_CST  150 // Couldn't set the control to the value provided
#define PS_CB_E_FSC  151 // Failed to apply setting due to lack of criteria
#define PS_CB_E_CEM  152 // Failed to load corrections image for lack of a mask
#define PS_CB_E_CEI  153 // Failed to load corrections image due to failed initialisation
#define PS_CB_E_CMM  154 // Failed set 'use corrections mask' due to lack of loadable / usable mask
#define PS_CB_E_CID1 155 // FAILED to close down the imaging device. Cannot change resolution.
#define PS_CB_E_CID2 156 // FAILED to change resolution and there may be memory errors - please re-start the server.
#define PS_CB_E_CID3 157 // FAILED to change resolution (reverted to previous).
#define PS_CB_E_CID4 158 // FAILED to re-open the imaging device after resize.
#define PS_CB_E_LNA  159 // Selected preview LUT is not available.
#define PS_CB_E_SZNA 160 // Selected preview size is not available.
#define PS_CB_E_SZB  161 // Preview re-size buffer failed.
#define PS_CB_E_SZU  162 // Failed to update preview settings post re-size.
#define PS_CB_E_CGT  163 // Couldn't get the camera control value.
#define PS_CB_E_NOS  164 // Error NOS

// Values that the image grabber function returns
#define GRAB_ERR_NONE       1 // Success!
#define GRAB_ERR_SELECT     2 // Error selecting a frame from the stream
#define GRAB_ERR_TIMEOUT    3 // Too long getting a frame from the stream
#define GRAB_ERR_READIO     4 // Error getting a frame by read io method
#define GRAB_ERR_MMAPD      5 // MMAP DQBUFF Error
#define GRAB_ERR_MMAPQ      6 // MMAP QBUFF Error
#define GRAB_ERR_USERPD     7 // User pointer DQBUFF Error
#define GRAB_ERR_USERPQ     8 // User pointer QBUFF Error
#define GRAB_ERR_NOSTREAM   9 // The camera is not streaming images
#define GRAB_ERR_BUSY      10 // The grabber is being used by another
                              // program or function - try again later.
#define GRAB_ERR_FATAL     PS_CB_E_NOS

// Return values for grab_n_save() function:
#define GNS_OKIS   20 // All went well. A picture was saved.
#define GNS_OKNB   21 // All went well. A picture was saved but was NOT
                      // flat field corrected even though the user asked
                      // for flat field correction.
#define GNS_OKNS   22 // All went well. A picture was NOT saved because
                      // tissue content thresholding was requested and
                      // the tissue content was below the trheshold.
#define GNS_ECAM   23 // Failed due to problem running the camera.
#define GNS_EGRB   24 // Failed due to an error returned by the grab
                      // routines (the value of the global 'grab_report'
                      // variable will contain the error details as one
                      // of the GRAB_ERR_... #defines, which see).
#define GNS_ETCV   25 // Error in the tissue content value getting function.

//                                                                            //
///////                       END OF: ERROR CODES                      /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                    GENERAL DEFS & GLOBALS                      /////////
//                                                                            //


// The PARD Port - the default port the client and server will be connecting to.
// This may be altered by the user but this is the port the program's ps_Port
// variable is initialised to at startup so if the user does not want to change
// anything it should work on this for PARDcap and PARDserver.
#define PS_PORT 16118  

// Values for ps_Instruction field IDs:
#define PS_INS_CIID   1  // Core instruction ID
#define PS_INS_SIID   2  // Sub instruction ID
#define PS_INS_SAFMT  3  // Save as format (when dealing with image data)
#define PS_INS_IVAL1  4  // Integer value 1
#define PS_INS_IVAL2  5  // Integer value 2
#define PS_INS_IVAL3  6  // Integer value 3
#define PS_INS_DVAL1  7  // Double precision floating point value 1
#define PS_INS_DVAL2  8  // Double precision floating point value 2
#define PS_INS_DVAL3  9  // Double precision floating point value 3
#define PS_INS_ERR0   10 // Error values array element err[0]
#define PS_INS_ERR1   11 // Error values array element err[1]
#define PS_INS_ERR2   12 // Error values array element err[2]
#define PS_INS_ERR3   13 // Error values array element err[3]
#define PS_INS_STR    14 // String
#define PS_INS_BLOB   15 // Blob

// Values for core instruction IDs:
#define PS_CIID_IMGR  1  // Retrieve an image
#define PS_CIID_IMGS  2  // Set an image
#define PS_CIID_LRES  3  // List resolutions (and frame rates) camera can do
#define PS_CIID_LSET  4  // List available camera settings
#define PS_CIID_CAMS  5  // Set a camera driver setting
#define PS_CIID_CUSS  6  // Set a custom (non-driver) frame grabber setting
#define PS_CIID_ERRC  7  // Conveying an error code +/- message string
#define PS_CIID_SRES  8  // Set the resolution for main image capture
#define PS_CIID_CAMG  9  // Get a camera driver setting

// IMGT (image type) SIID options for IMGR and IMGS instructions.
#define PS_IMGT_MAINF 1  // Main image full size (i.e. capture an image / series) 
#define PS_IMGT_FFIMF 2  // Flat field image full size 
#define PS_IMGT_FFIMP 3  // Flat field image for mono preview
#define PS_IMGT_DFIMF 4  // Darkfield image full size
#define PS_IMGT_DFIMP 5  // Darkfield image for mono preview
#define PS_IMGT_DFCDP 6  // Darkfield image for colour preview
#define PS_IMGT_MASKF 7  // Corrections mask full size
#define PS_IMGT_MASKP 8  // Mask for preview image
#define PS_IMGT_PREVW 9  // Preview image at 640x480

// CUSS (custom frame grabber setting) SIID options
#define PS_CUSS_FTOSK 1  // Frames-to-skip 
#define PS_CUSS_SKFRM 2  // Skip frames 
#define PS_CUSS_GAIYU 3  // YUYV Gain 
#define PS_CUSS_BIAYU 4  // YUYV Bias 
#define PS_CUSS_FRMNO 5  // Frame number 
#define PS_CUSS_FTOAV 6  // Frames to average 
#define PS_CUSS_SAFMT 7  // Save-as format
#define PS_CUSS_FFCOR 8  // flat field correction on/off (prev. bg corr)
#define PS_CUSS_DFCOR 9  // dark field correction on/off - new 2025
#define PS_CUSS_MACOR 10 // mask for corrections on/off - new 2025
#define PS_CUSS_PCDFC 11 // preview colour dark field corr. on/off  - new 2025
#define PS_CUSS_PMAST 12 // preview mask for stats. on/off  - new 2025
#define PS_CUSS_PMASH 13 // preview mask show on/off  - new 2025
#define PS_CUSS_PHSSH 14 // preview histogram show on/off  - new 2025
#define PS_CUSS_PHSCR 15 // preview histogram cumulative Red/Y on/off - new 2025
#define PS_CUSS_PHSCG 16 // preview histogram cumulative Green on/off - new 2025
#define PS_CUSS_PHSCB 17 // preview histogram cumulative blue on/off - new 2025
#define PS_CUSS_PHSDF 18 // preview histogram don't auto-fit on/off - new 2025
#define PS_CUSS_PHSVM 19 // preview histogram restrict var/mn on/off - new 2025
#define PS_CUSS_PHSSF 20 // preview histogram scale factors - new 2025
#define PS_CUSS_PHSSL 21 // preview histogram saturation lower limits - new 2025
#define PS_CUSS_PHSSU 22 // preview histogram saturation upper limits - new 2025
#define PS_CUSS_PFMAL 23 // Focus assist - mean abs. Laplacian on/off - new 2025
#define PS_CUSS_PFVAR 24 // Focus assist - var x other(s) on/off - new 2025
#define PS_CUSS_PMONO 25 // Preview in monochrome on/off - new 2025
#define PS_CUSS_PINVT 26 // Preview invert colours on/off - new 2025
#define PS_CUSS_PFLPH 27 // Preview flip horizontal on/off - new 2025
#define PS_CUSS_PFLPV 28 // Preview flip vertical on/off - new 2025
#define PS_CUSS_PLAPL 29 // Preview display Laplacian on/off - new 2025
#define PS_CUSS_PVLUT 30 // Preview LUT - new 2025
#define PS_CUSS_PHSXY 31 // Preview histogram position XY - new 2025
#define PS_CUSS_PFCPY 32 // Preview focus assist position Y - new 2025
#define PS_CUSS_PMCEJ 33 // Preview mono correction images eject - new 2025
#define PS_CUSS_PMINT 34 // Preview mono integral number - new 2025
#define PS_CUSS_PMBIA 35 // Preview mono bias number - new 2025
#define PS_CUSS_AVSCA 36 // Scale each image to first (averaging)on/off-new 2025
#define PS_CUSS_GBTMO 37 // Grabber timeout in seconds - new 2025
#define PS_CUSS_GBRTY 38 // Grabber retries before failure - new 2025
#define PS_CUSS_GBDLY 39 // Grabber pre-delay for first image (s) - new 2025
#define PS_CUSS_SADOU 40 // Save as raw doubles off/on(raw)/fits - new 2025
#define PS_CUSS_PFCSH 41 // Preview focus assist show mode - new 2025
#define PS_CUSS_GPVST 42 // Get prieview stats off/on - new 2025

#define PS_CUSS_PFDCT 43 // Focus parameter DCT on/off - new 2025
#define PS_CUSS_PREMDN  44 // Preview pre-Laplacian median on/off - new 2025
#define PS_CUSS_POSTMDN 45 // Preview post-Laplacian median on/off - new 2025
#define PS_CUSS_DCTAL 46 // Calc. DCT after LUT on/off - new 2025
#define PS_CUSS_PVSZ  47 // Preview size - new 2025

#define PS_CUSS_GTCVL 48 // Get tissue content value off/on - updated 2026
#define PS_CUSS_TCTHR 49 // Set tissue content thresholds off/on - updated 2026
#define PS_CUSS_GPFOC 50 // Get focus values from a new preview frame - new 2026


// The only thing missing here are the options for taking a series, i.e. number
// of images in a series and delay between captures. This will be difficult to
// implemented with the ZMQ req-rep mode I am using (but not impossible).
// However, the series function is essentially an in-built loop in pardcap that
// calls the various grabber functions already controllable by the preceding
// controls and so this same functionality can be implemented locally in the
// client by a simple PCS script loop. So no functionality is actually lost or
// unavailable to the client with this omission.

 


// Values for saveas_fmt
#define SAF_YUYV 0  // Raw YUYV 16 bpp array
#define SAF_YP5  1  // Y-only (greyscale) 8bpp pgm p5 format
#define SAF_BM8  2  // Y-only (greyscale) 8bpp bmp format
#define SAF_INT  3  // Insensity from RGB/YUYV raw doubles monochrome
#define SAF_RGB  4  // Full colour RGB array saved in PPM p6 format
#define SAF_BMP  5  // Full colour RGB array saved in 24bpp BMP format
#define SAF_PNG  6  // Full colour PNG file
#define SAF_JPG  7  // Full colour JPEG file
#define SAF_DOU  8  // Raw doubles array (1 channel)   } These two are only used
#define SAF_DO3  9  // Raw doubles arrays (3 channels) } by the server.
#define SAF_PZO  10 // For a preview image only - zoomed
#define SAF_PUZ  11 // For a preview image only - unzoomed

// Values for colour channel for raw doubles IO (and preview overlay)
#define CCHAN_Y 1
#define CCHAN_R 2
#define CCHAN_G 3
#define CCHAN_B 4

// Return values for switch_string 
#define SWS_ON   1 // "ON"
#define SWS_OFF  2 // "OFF"
#define SWS_PPM  3 // String ends in .ppm
#define SWS_PGM  4 // String ends in .pgm
#define SWS_BMP  5 // String ends in .bmp
#define SWS_DOU  6 // String ends in Y.dou
#define SWS_DO3  7 // String ends in R.dou
#define SWS_FITS 8 // "FITS"
#define SWS_NOS  9 // String is none of the above


// Values for ffcorr_status are
#define FFCORR_OFF  0 // Don't (or can't) do flat field correction 
#define FFCORR_ON   1 // Do flat field correction.

// Values for fffile_loaded 
#define FFIMG_NONE  0  // No flat field (ff) image is currently loaded
#define FFIMG_RGB   1  // A colour image is loaded for ff correction
#define FFIMG_Y     2  // A monochrome image is loaded for ff correction
#define FFIMG_NORM  3  // It loaded fine but is awaiting normalisation.
#define FFIMG_BM8   4  // A monochrome image of bmp 8bpp format is loaded 
#define FFIMG_BMP   5  // A colour image of bmp 24bpp format is loaded 
#define FFIMG_DOU   6  // A monochrome image in doubles format is loaded 
#define FFIMG_DO3   7  // A colour image in doubles format is loaded 

// Values for dfcorr_status are
#define DFCORR_OFF  0 // Don't (or can't) do dark field correction
#define DFCORR_ON   1 // Do dark field correction 

// 'Do Field Correction' Values for do_ff_correction and do_df_correction
#define DOFC_NO     0  // Don't do it (usually because the currently
                       // selected 'save as' format is incompatible with
                       // the colour format of the currently loaded
                       // flat field image.
#define DOFC_RGB    1  // Yes, correct a colour image with the currently
                       // loaded colour flat field image.
#define DOFC_Y      2  // Yes, correct a greyscale image with the
                       // currently loaded greyscale flat field image.


// Values for dffile_loaded (also used for dffile_loaded so flat field
// when reading raw doubles from disc so these MUST have the same
// #defined numerical values as the corresponding FFIMG_... constants). 
#define DFIMG_NONE  0  // No dark field (df) image is currently loaded
#define DFIMG_RGB   1  // A colour image is loaded for df correction
#define DFIMG_Y     2  // A monochrome image is loaded for df correction

// Values for mask_alloced
#define MASK_NO     0  // Don't use a custom corrections mask. (This must be 0)
#define MASK_YES    1  // Yes, use a custom corrections mask.

// Values for mskfile_loaded 
#define MASK_NONE  0  // This is set when mask_alloced fails.
#define MASK_YRGB  1  // A custom corrections mask is currently loaded
#define MASK_FULL  2  // A non-custom full support mask is present

#define PREVINTMAX 20 // Maximum No. of frames for integration.

#define STATSBYTES 206 // Number of bytes in a preview stats blob
#define TCBYTES 8      // Number of bytes in a tissue content blob
#define FOCUSBYTES 24  // Number of bytes in a focus blob
#define CAMGETBYTES 4  // Number of bytes in a camera_get blob

// Preview resolution numbers
#define PV_VGA   0 // 640x480
#define PV_QVGA  1 // 320x240
#define PV_QQVGA 2 // 160x120
#define PV_UNSET 3 // Unknownd / unset

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

// The PS Blob
unsigned char *ps_Blob;  // The blob storage for server comms
int ps_BlobSize;         // The current size of blob storage in bytes

// The socket file descriptor for commuications with client / server
int ps_Comms_fd;  
// Flag to let us know if there is a connection (client to server or vice versa)  
int ps_CliSrv_connected;  
// Number (1, 2, 3, etc.) and corresponding array index (0, 1, 2, etc.) of the
// currently connected PARDcap server in the PSlist. This is used on the
// pardclient and pardserver but not in parcapture.
uint32_t CurrPS,CamIdx; 


// The instruction structure
 typedef struct {
   uint32_t ciid;   // Core instruction ID
   uint8_t  siid;   // Sub instruction ID
   uint8_t  safmt;  // Image format when dealing with image data
   int32_t  ival1;  // Image height or camera control ID
   int32_t  ival2;  // Image width or camera control value (if it is an integer)
   int32_t  ival3;  // third integer (e.g. for XYZ coord triplets with ival1/2)
   double   dval1;  // For 3 float values e.g. of a control, focus measure, etc.
   double   dval2;  // For 3 float values e.g. of a control, focus measure, etc.
   double   dval3;  // For 3 float values e.g. of a control, focus measure, etc.
   int32_t  err[4]; // To store upto 4 error codes (or success codes).
   int32_t  slen;   // Number of bytes in the following string
   uint8_t  *str;   // String of chars (image file name, list of settings, etc.)
   int32_t  blen;   // Number of bytes in the following blob)
   void     *blob;  // Blob for image or other general binary transmision
   struct {
    unsigned int siid  : 1;
    unsigned int safmt : 1;
    unsigned int ival1 : 1;
    unsigned int ival2 : 1;
    unsigned int ival3 : 1;
    unsigned int dval1 : 1;
    unsigned int dval2 : 1;
    unsigned int dval3 : 1;
    unsigned int err   : 1;
    unsigned int str   : 1;
    unsigned int blob  : 1;
   } used;
 } ps_InstructData;


//                                                                            //
///////                 END OF: GENERAL DEFS & GLOBALS                 /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                       FUNCTION PROTOTYPES                      /////////
//                                                                            //

int      ps_recv_dc(int socketfd, unsigned char *databuff, int *datalen, int timeout_ms);
void     ps_print_recv_error(int rv, char *infostr);
int32_t  ps_ins_set(int imember,void *ivalptr,void *ibuff);
void     ps_initialise_instruction(void);
void     ps_ins_reset(void);
int32_t  ps_alloc_instr(void);
int32_t  ps_alloc_insblob(void);
int      ps_ins_send(void);
int      ps_ins_to_blob(unsigned char **blobuff, int *bloblen);
void     ps_binval_to_uc(unsigned char *buf,void *bvptr, int binvalen);
int      ps_resize_blob(int32_t bsz);
int      ps_initialise_blob(void);
int      ps_receive_blob (int timeout_ms);
int32_t  ps_increment_blobindex(int32_t *idx,int32_t incr);
int32_t  ps_check_img_type(int8_t);
int32_t  ps_check_img_fmt(int8_t);
int32_t  ps_uc_to_int32(unsigned char *ucbuffer);
uint32_t ps_uc_to_uint32(unsigned char *ucbuffer);
double   ps_uc_to_dou(unsigned char *ucbuffer);


//                                                                            //
///////                   END OF: FUNCTION PROTOTYPES                  /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


#endif  // pardcapdefs_h__
