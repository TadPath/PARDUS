# Sample Images taken with PARD Capture and the AF51 Camera
Scientific image acquisition software, FLOSS GPL v3

                       
Description
------------
Below are links to some images taken with the [AF51 camera from OptArc](https://www.optarc.co.uk/products/cameras-2/) using the PARD Capture software. These are large full resolution (5 megapixel) uncompressed files so may take some time to load depending on your connection speed.

1. [Colour_1_png](Images/x40yuyvOA_avg_rgb.png) 3.4 MB PNG image. This is an image of a histological section of human skin stained with H&E taken through a PUMA microscope using an OptArc x40 objective and full Köhler transillumination. The image is a 64 frame average using the YUYV raw output of the camera. No dark field of flat field corrections was used. No corrections mask were used. The camera 'Sharpening' setting was set to 1 (out of a possible range of 0 to 100). The scale bar was added after acquisition in a separate software package.

2. [Grey_1_png](Images/x40OA_TestY_1269F_0000_Y.png) 1.0 MB PNG image. This is an image of pollen grains and surrounding structures in a thick histological section of a Soncus flower bud stained with fluorescein and taken through a PUMA microscope using an OptArc x40 objective. This was using blue light co-axial epi-illumination. This is an example of epi-fluorescence microscopy. The acquisition was of 64 frames (a multi-frame average) where each frame was dark field corrected and flat field corrected before being accumulated into the average. This was also masked hence the unusual sharp boundary just inside the circular field of view, this being the boundary of the mask. Everything outside this boundary was not subject to the corrections described above. This image uses only the Y component from the raw YUYV output of the camera. The camera 'Sharpening' setting was set to 1 (out of a possible range of 0 to 100). The exposure was set at 1269 (out of a maximum of 10000). The scale bar was added after acquisition in a separate software package. This image, being saved in an 8bpp format looses a lot of the quantisation benefits of the multi-frame averaging and applied corrections. See the raw doubles version below to get the most information from this image.

3. [Grey_1_raw_doubles](Images/grey1_cropped.dou) 22.0 MB Raw doubles file size 1700 x 1700 pixels. This is a cropped version of **Grey_1_png** but it was saved directly in raw doubles format from PARD Capture to preserve the advantages of multi-frame averaging and field corrections in reducing quantisation and increasing bit depth. You will need to use some software capable of displaying raw doubles (perhaps ImageJ or BiaQIm). Explore different windows to see the enhanced dynamic range, reduced quantisation effects and examine the noise profile. I had to crop the image because the original 5 MP frame was 38 MB and GitHub would not allow upload of any file over 25 MB. If you are going to use BiaQIm to view this image then the accompanying external header file [is here](Images/grey1_cropped.qih).



PJT

First Written: 12.11.2022

Last Edit: 13.11.2022
