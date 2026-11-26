# PCS Language Specification (PARDUS)
> High-density reference for AI coding agents. Optimized for version 1.

## Core Language Constraints
- **Atomic Statements Only:** No nested expressions (e.g., no `x = (y + z)`). One operation per line.
- **No String Delimiters:** Quotes are literal data. Use `set_str msg Hello World` (no quotes around Hello World).
- **Global Scope:** No local variables. Use unique prefixes (e.g., `func_varname`) to prevent collisions.
- **Universal Arrays:** Every variable is an array. Use `var[index]` for size > 1.
- **Nested dereferencing:** Array indices may be variables, including other arrays.
- **Left-Hand Master Casting:** The target variable's type determines how the source data is converted.
- **Whitespace:** Spaces between arguments are collapsed. Use system variable `Space` or `cat_char 32` for explicit formatting.

## Control Flow & Logic
- **First-Class Control Flow:** `goto` and `call_func` accept variables.
- **Non-Local Jumps:** `goto` can jump across function boundaries for "Panic Exits" or recovery points.
- **Recursive Functions:** `call_func` supports deep recursion.
- **Branching:** `set_condexp <varval1> <op> <varval2>` takes exactly 3 arguments (no inline math) and writes `0` or `1` to `Condexp`. Use `do_if Condexp` or `loop_while Condexp` to branch.
- **Left-Hand Comparison Casting:** `<varval2>` is temporarily cast to `<varval1>`'s type before comparison. Strings compare literally (`"5.0" != "5"`) and only allow `==` and `!=`.
- **Dynamic Operators:** `<op>` is evaluated as a string (`==`, `>=`, `>`, `<=`, `<`, `!=`, `&&`, `||`) and can be supplied by a string variable for runtime meta-programming.
- **Logic Gates:** Combine two existing flags directly using `&&` or `||` (`set_condexp A && B`), or accumulate `Condexp` results into a sum variable `S` via `add_var` for multi-condition gates:
  - **AND (all of N):** `set_condexp S == N`
  - **OR (any of N):** `set_condexp S > 0`
  - **XOR (2 flags):** `set_condexp A != B` (or `set_condexp S == 1`)
  - **NOT (1 flag):** `set_condexp A == 0`


## AI Design Patterns

### 1. The Panic Jump (Error Handling)
Jumping from deep recursion to a main-block label:
```pcs
def_func Worker
  set_condexp ErrorState == 1
  do_if Condexp
    goto recovery_point
  enddo
  call_func Worker
return
```

### 2. Jump Tables (Dynamic State)
```pcs
set_var States[0] IDLE_STATE
set_var States[1] BUSY_STATE
goto States[CurrentIndex]
```

### 3. The Scratchpad (Persistent Memory)
Everything below the `exit` command is ignored by the interpreter. Use this for:
- JSON metadata
- Future task planning
- Reasoning/Chain-of-Thought logs

## System Prompt for PCS Code Generation
"Act as an expert PCS programmer for PARDUS. Maintain a flat logic structure with no nested expressions. Use non-local goto jumps for error recovery. Perform all math in atomic steps via math_var/add_var. Every variable is an array. Place a detailed execution plan in the scratchpad zone below the exit command."

## Extended Documentation
- Full Language Manual: [./docs/PCS/pcs_ref_ai.md]
- Working Code Samples: [./PCS_examples/]

## Full Command Dictionary
Format: `command <required_args> [optional_args]` - Description
Note: All commands are case-sensitive and parameters are space-separated.
- `act_accel <act_type> [args based on <act_type>]` - Set acceleration parameters for actuator of type <act_type>.
- `act_get <act_id> <member_ID> <VARVAL> [<VARVAL_y> <VARVAL_z>]` - Get information about an actuator.
- `act_getid <actid>` - Get the actuator type and index number from its name.
- `act_getname <act_type> <act_index> <string_variable>` - Get the name of an actuator from its type and index number.
- `act_halt <act_id>` - Causes an actuator to stop what it is doing.
- `act_home <act_id> <home_type>` - Home an actuator.
- `act_moveby <act_id> <duration> <distance> [<dist_y> [dist_z]]` - Move an actuator from its current location and in its current direction by a specific distance over a specific time.
- `act_movereport <int_result> <act_type> <act_index>` - Poll the server (no wait, non-blocking) for a motion report.
- `act_moveto <act_id> [arguments as per actuator type]` - Move an actuator from its current location to a specific destination over a specific time.
- `act_set <act_id> <member_ID> <VARVAL> [<VARVAL_y> <VARVAL_z>]` - Set some property of an actuator.
- `act_unit <act_type> <unit>` - Set whether distance and position values are interpreted as steps or calibrated units.
- `add_lib <file_path>` - include a .pcl library into the script.
- `add_var <varname> <varval>` - Add a value to a variable.
- `af_ccrit <max_iterations> <max_difference> <min_true_peak> <timeout>` - Set the convergence criteria for the server-side autofocus algorithm.
- `af_setup <initial_focus_z> <initial_step_size> <max_retries> <dirn_up> <col_channel> <failure_action>` - Set common and initial values for the server-side autofocus algorithm.
- `autofocus` - Activate the server-side autofocus algorithm.
- `average_scale_means <ON|OFF>` - Whether to scale the mean of any image captured as part of a multiframe average to the mean of the first in the series.
- `beep <duration_ms> <freq_Hz> [amplitude_percent]` - This emits a non-blocking sine wave beep on the local PC.
- `call_func <function_label> [if <VARVAL>]` - This calls a function. The <function_label> may be a string variable.
- `camera_get <Control_ID> <varval> [info_item]` - Get the value of a control on the current camera.
- `camera_set <Control_ID> <VARVAL>` - Set the value of a control on the current camera.
- `cat_char <STR_VAR> <char_as_int>` - Add any UTF-8 Unicode character to the end of a string variable.
- `cat_str <STR_VAR> [[str1] [[str2] ...]]` - Add a list of up to 24 strings to a string variable, each separated by a single space character.
- `cli_path <root_full_path_for_preview_image>` - Allow the pcs-cli headless mode to emit updates of the current preview scene as a ppm file to disc.
- `cmath_array <rec_re_array> <rec_im_array> <cx_op> [<mod_re_array> <mod_im_array>]` -  Do complex maths with arrays (of type float).
- `cmath_var <rec_re> <rec_im> <cx_op> [<mod_re> <mod_im>]` - Do complex maths with single variables (of type float). 
- `corrections_mask <OFF|ON|image_file>` - whether to load and use a corrections mask for flat field and dark field correction during full frame image capture.
- `def_actuator <act_type> <act_index> <name_string>` - Create a nascent actuator structure on the server.
- `def_array <type> <name> <size> <initial_VARVAL>` - Define an array of variables.
- `def_func <function_label>` - Opens a function definition block. <function_label> must be a string constant (not a variable).
- `def_tracker <tracker_type> <tracker_name> <template_dim> <search_dim> <cell_dim>` - Create a new tracker
- `def_var <vartype> <varname> <VARVAL>` - Define a variable (equivalent to an array of size 1).
- `df_correction <OFF|ON|image_file>` - whether to load and use a dark field correction image for full frame image capture.
- `div_var <varname> <varval>` - Divide a variable by some value.
- `do_if <VARVAL>` - Opens a do_if code block.
- `else` - Terminates a do_if code block and starts and else code block.
- `enddo` - Terminates a code block opened by do_if or else.
- `endloop` - Terminates a code block opened by do_while.
- `eval_char <string> <position> <receptacle>` - extract the integer byte value of any character in a character string and put it into the receptacle variable.
- `execute_daemon <progname> [<arg1> ...]` - Spawn a process to run on the client host computer (non-blocking execution).
- `execute_system <progname> [<arg1> ...]` - Initiate a process to run on the client host computer (blocking execution)
- `exit` - Terminates a script. Mandatory for any script or header file.
- `fbyte_mode <ON/OFF>` - Sets whether file IO with integer variables is done 1 byte at a time (ON) or 4 bytes at a time (OFF).
- `ff_correction <OFF|ON|image_file>` - whether to load and use a flat field correction image in full frame image capture.
- `fourier <action> <forward?> <height> <width> <in_re> <in_im> [<out_re> <out_im>]` - Perform Fourier-related processes on arrays in 1D and 2D
- `fput_var <varname> <append?> <file_path>` - Writes the values in a variable (or array) to a file.
- `frame_number <number>` - Sets the value of the frame counter number for the file name the next captured image.
- `frames_to_average <number>` - Sets the number of frames to average for image capture with grab_image.
- `frames_to_skip <skip>` - Sets the number of frames to initially acquire and discard for each grab_image operation.
- `fset_var <varname> <skip> <file_path>` - Sets the values in a variable (or array) from data in a file.
- `fskip_size <number_of_bytes>` - Sets the size of a 'data item' used in the initial skip phase of an fset_var process.
- `get_actuators` - Get the name and index number of all extant actuators on the server.
- `get_camsettings` - Ensures the client GUI is synchronised to the settings of the currently selected camera.
- `get_preview <ON|OFF> <centre_x> <centre_y> [Display?]` - Capture a preview image. It will be immediately shown unless [Display?] is false.
- `get_preview_focus` - Capture a new preview image and retrieve only the RGB focus stats (but not the image).
- `get_preview_stats <ON|OFF>` - Retrieve the all stats, including the RGB focus stats, for a preview image (does not retrieve the image).
- `get_preview_tc <ON|OFF>` - Capture a new preview image for calculating the tissue content value (does not retrieve the image).
- `get_resolutions` - Update the client GUI pages and combo lists with the resolutions information of the current camera.
- `get_tracker <tracker_name> <tracker_item> <receptacle>` - Read a tracker setting or parameter value. 
- `goto <label> [if <VARVAL>]` - Jump to some part of a script. <label> may be contained in a string variable.
- `grabber_predelay <seconds>` - Tell the server how long to wait before a grab_image command attempts image capture.
- `grabber_retries <number>` - Number of time-out failed image grab attempts to ignore before failing with a fatal error.
- `grabber_timeout <seconds>` - Number of seconds to wait for a frame from the grabber failing registering a time-out fail.
- `grab_image` - Request a full size image to be taken and sent to the client.
- `gui_buffer <varval>` - allows drawing to the script console to be performed in an 'off-screen' fast text buffer.
- `gui_update` - immediately refresh the GUI.
- `gui_update_period <varval>` - set how often the GUI is refreshed.
- `init_tracker <tracker_name> <temp_r> <temp_c> <colchan>` - Initialise or re-initialise a tracker for tracking. 
- `jpeg_quality <number>` - Set the quality used to save a jpeg image to disc (1 = lowest quality, highest compression).
- `loop_while <VARVAL>` - Opens a loop_while block.
- `make_lib` - convert the current script into a library (archiver).
- `match_tracker <tracker_name> <row> <col> <receptacle>` - Get the tracker's matching score for a user-defined crop from the current preview image.
- `math_array <array_receptor> <math_func> [array_modifier]` - Perform mathematical operations with arrays
- `math_var <var_receptor> <math_func> [varval]` - Perform a mathematical operation with up to 2 values and put the result into <var_receptor>
- `max_daemons <number>` - Set the number of simultaneous background processes to operate simultaneously.
- `milliduration <start_millis> <receptor>` - Calculates the time in milliseconds elapsed from the monotonic count in <start_millis>
- `mul_var <varname> <varval>` - Multiply a variable by some value.
- `null_command` - do nothing except skip to the next command.
- `poll_user <wait> <return> <VARVAL>` - Look for input in the GUI text box
- `preview_clear` - Clears the full VGA preview display area.
- `preview_copy <height> <width> <receptacle> <colchan>` - Copies a rectangular portion of the current preview image into an integer array. 
- `preview_draw_angles <angle1> <angle2>` - set the angles that determine how an ellipse is drawn on the preview overlay buffer.
- `preview_draw_array <int_array>` - draw a preview overlay buffer based on the instructions in the input array.
- `preview_draw_blend <truefalse>` - set if preview overlay buffer drawing should blend with or overwrite existing graphics.
- `preview_draw_buffer <mode>` - set the preview overlay buffer to off, immediate or buffered modes.
- `preview_draw_chamfer <pixels>` - set the chamfer radius for rectangles drawn in preview overlay buffer.
- `preview_draw_clear` - clear the preview overlay buffer.
- `preview_draw_colour <R> <G> <B>` - set the colour for drawings on the preview overlay buffer.
- `preview_draw_cursor <X> <Y>` - set the position for drawings in the preview overlay buffer.
- `preview_draw_desttype <truefalse>` - Whether to use absolute coordinates (false - the default) or relative dimensions (true) for the destination arguments in drawing lines or rectangles.
- `preview_draw_file <file_path>` - draw the p6 image in a file onto the preview overlay buffer.
- `preview_draw_lwidth <pixels>` - set the line width for drawings in the preview overlay buffer.
- `preview_draw_pcg <char_number> <int_array>` - specify a custom character bitmap for the preview overlay buffer.
- `preview_draw_save <file_path>` - save the preview overlay buffer to a p6 file on disc.
- `preview_draw_shape <type> [<filled?> <X_dest/radius> <Y_dest/radius>]` - draw a pixel, line, rectangle or ellipse in the preview overlay buffer.
- `preview_draw_show` - display the preview overlay buffer overlain on the preview image.
- `preview_draw_string <size> <varval> [<bg_R> <bg_G> <bg_B> <invert?>]` - Draw text in the preview overlay buffer.
- `preview_file <file_path>` - Sets the file path and file name which will be used to save preview images.
- `preview_load [Display?]` - Load a p6 file to the current preview image. It will be immediately shown unless [Display?] is false.
- `preview_save` - Saves the last retrieved preview image to disc as a p6 ppm image.
- `preview_show` - Emits the current preview image buffer and any overlay buffer to the display.
- `preview_tile <number>` - Specify where a preview image is displayed in the VGA preview display area.
- `print_clear` - This clears the text buffer in the script console
- `print_colour <R/P> [<G> <B>]` - This sets the foreground colour for print_value operations.
- `print_cursor <X> <Y>` - This sets the position of the cursor in the script console. Resets to 0,0 after every print_value command.
- `print_file <exclusive?> <file_path>` - Allows the print_value output to be diverted to a file.
- `print_graphics <varval>` - This sets whether (ON) or not (OFF) printing is done in 1:1 aspect ratio graphics mode.
- `print_size <point_size>` - This sets the font size for print_value operations.
- `print_value <VARVAL> [format]` - Prints the value of <varval> to the current print cursor position in the script console (and optionally to a file)
- `prv_cdf_use <OFF|ON|image_file>` - Whether or not to apply subtraction of a custom preview colour dark field image.
- `prv_fc_posn <y>` - Set the Y position for the preview focus bar(s) overlay.
- `prv_fc_show <show_mode>` - Select whether and how to show the focus bar(s) overlay on the preview image.
- `prv_hgm_posn <x> <y>` - Set the position to display the histogram overlay on the preview image.
- `prv_hgm_satlim_l <r/y> <g> <b>` - Sets the preview image lower histogram saturation limits.
- `prv_hgm_satlim_u <r/y> <g> <b>` - Sets the preview image upper histogram saturation limits.
- `prv_hgm_scales <r/y> <g> <b>` - Sets the preview image histogram manual scaling factors
- `prv_lut <LUT_number>` - Sets the preview image look-up-table (LUT) for display.
- `prv_mask_use <OFF|ON|image_file>` - Whether or not to apply a custom preview mask to the calculation of preview stats.
- `prv_m_bias <number_of_grey_levels>` - Sets the number of grey levels to add (or subtract) from the monochrome preview image
- `prv_mcor_eject` - Ejects any loaded custom preview mono dark and flat field correction images.
- `prv_mdf_set <image_file>` - loads the supplied dark field corrections image and sets it as active for use with monochrome previewing.
- `prv_mff_set <image_file>` - loads the supplied flat field corrections file and sets it as active for use with monochrome previewing.
- `prv_m_integral <number_of_frames>` - Sets the number of frames to integrate for monochrome previews.
- `prv_size <size_number>` - Sets the size of the preview image as displayed in the VGA-sized preview area.
- `prv_toggle <option> <ON|OFF>` - This sets the binary state of a number of preview options.
- `randpcg <init?> <receptacle> [pos?]` - This generates a pseudorandom number and puts it into the receptacle variable.
- `return` - This closes a function definition block (opened with def_func)
- `save_coords <ON|OFF>` - Whether to save a coordinates file
- `save_doubles <OFF|ON|FITS>` - whether to also save the main captured image in raw doubles or FITS format.
- `save_format <format>` - Selects the preferred format for saving a main image.
- `save_path <image_full_save_path>` - Sets the directory path used for saving a main image.
- `save_resolution <width> <height>` - Set the pixel resolution for grabbing a full sized image.
- `save_root <image_name_root>` - Set the file name root for saving a main image.
- `select_camera <number>` - Set the current camera for all imaging operations.
- `set_condexp <varval1> <condition> <varval2>` - Performs a conditional expression and save the result to the system variable Condexp
- `set_previewpos <pos> <inc>` - set the value of the system variables PreviewPos and PreviewInc to the values specified in the arguments.
- `set_str <STR_VAR> [[str1] [[str2] ...]]` - Set a string variable to contain up to 24 strings, each separated by a single space character
- `set_tracker <tracker_name> <tracker_setting> <varval>` - Set a tracker parameter.
- `set_var <varname> <varval> [charpos]` - Set the value of a variable.
- `skip_frames <skip>` - Immediately capture and discard <skip> frames.
- `sleep <value>` - Pause script execution for <value> seconds (which may be a fractional float number).
- `stats_array <array_in> <stats_func> <threshold> <arguments as per <stats_func>>` - Calculates various summary statistics on an array of floats or ints (1D or 2D).
- `sub_var <varname> <varval>` - Subtract a value from a variable.
- `tc_threshold <lower> <upper>` - Sets lower and upper tissue content thresholds.
- `tmc_chconf_parse <chconf_info> <receptor>` - Extract a piece of information from the locally stored TMC stepper chop conf register.
- `tmc_status_parse <status_info> <receptor>` - Extract a piece of information from the locally stored TMC stepper status register.
- `track <tracker_name> <receptor_r> <receptor_c> <receptor_score>` - Activate a tracker to track its target.
- `undef_actuator <act_id>` - Free an actuator definition.
- `undef_tracker <tracker_name>` - Free a tracker (delete it and take it off the extant tracker list).
- `undef_var <varname>` - Deletes a user-defined variable and all its data. May be used with arrays or single variables.
- `update_gui_coords <server_sync?>` - Update the GUI coordinate display with the most recent locally held values.
- `verbosity <level>` - Sets how much output to show in the server console, script console and log file.
- `xyzs_af_period <period>` - Set the frequency of autofocus activation during an XY scan.
- `xyzs_finish <coord_x> <coord_y>` - Set the end-point coordinates of an XY scan.
- `xyzs_process <progname> [<arg1> ...]` - Spawn an external non-blocking command during an XY scan.
- `xyzs_scan` - Begin an XY scan.
- `xyzs_semino_z <slices>` - Set the number of focus plane slices in an XYZ scan.
- `xyzs_start <coord_x> <coord_y> <coord_z>` - Set the start point coordinates for an XY scan.
- `xyzs_stepsize <size_x> <size_y> <size_z>` - Set the size of each X, Y and Z movement step in an XYZ scan.
- `yuyv_bias <value>` - Set the value of the bias constant used in the YUYV-to-RGB conversion.
- `yuyv_gain <value>` - Set the value of the gain constant used in the YUYV-to-RGB conversion.

