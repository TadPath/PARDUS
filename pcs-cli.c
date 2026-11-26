/******************************************************************************/
/* pcs-cli                                                                    */
/*                                                                            */
/* This is the headless remote client application for a PARDUS robotics user. */
/* Copyright (c) Dr Paul J. Tadrous 2000-2026   All rights reserved.          */
 /*                                                                          */
  /*                                                                        */
   /*                                                                      */
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
   /*                                                                      */
  /*                                                                        */
 /*                                                                          */
// ============================================================================
//  pcs-cli.c  --  a headless driver for PCS lib tests, runing, compiling, etc. 
// ============================================================================
//
//  WHAT IT IS
//  ----------
//  pardclient is a GTK4 application, and a PCS script normally reaches the
//  interpreter / compiler / linker / run-engine through buttons and file
//  dialogs or dragn-n-drop.  pcs-cli allows these operations, and more, via the
//  terminal CLI without the need for a GUI.
//  It may be used to to run .dove programs directly by double-click on the
//  .dove executable file (if .dove files are associated with pcs-cli via the
//  desktop system on your PC. However it may also be used to run source code
//  pcs files from the command line as well as for compiling, decompiling,
//  library archiving and PCD code analyss as well. To do all these things
//  rigorously, the tests must run pardclient's own code - the same
//  script_preprocess(), compile_pcs_to_memory() (with the new linker at its
//  end), script_check(), link_aot_flow_control(), script_run(),
//  save_compiled_program_to_disk() and load_compiled_program_from_disk() -
//  not a re-implementation of them.  This file does that by #including
//  pardclient.c itself, with the client's main() renamed out of the way, and
//  supplying a small command-line main() of its own.
//
//  No GUI is started: gui_up stays 0, so show_message() writes to the
//  terminal instead of to widgets, and compile_to_lib() writes the library to
//  its default name (the script's name with the extension changed to .pcl)
//  instead of showing the save dialog (see compile_to_lib, "Rule 7").
//  If a script uses any command that needs the server, the user will be
//  prompted to enter their password at the command line before continuing (all
//  other server-related connection information being read from a config file).
//
//  HOW TO BUILD (from the folder that holds this file and parclient.c
//  --------------------------------------------------------------------------
//    gcc -O2 -march=x86-64-v2 -mavx -w `pkg-config --cflags gtk4` -o pcs-cli resources.c pcs-cli.c `pkg-config --libs gtk4` -lm -lzmq -lpng -ljpeg -lfontconfig -lpulse-simple -lpulse -lX11

// On MSYS2 UCRT64
// gcc -O2 -march=native -w `pkg-config --cflags gtk4` -o pcs-cli.exe resources.c pcs-cli.c `pkg-config --libs gtk4` -lws2_32 -lm -lzmq -lpng -ljpeg -lfontconfig -lwinmm
//  -w silences the client's pre-existing warnings
//  
// See the user manual for the macOS compile command.
//
//  COMMANDS
//  --------
//    pcs-cli [run]  <script.pcs|.dove> [interrupt_script.pcs|dove]
//        Pre-process, compile (and link), check, and run a script, or load,
//        AOT-link and run a .dove.  Prints one "pcs-cli:" line per stage if
//        the default verbosity is > 1. This is also how you would create a
//        library (archive a library) if the pcs script is of the correct
//        format for being the source of a library. The resulting library will
//        be saved with the name <path_root>.pcl where <path_root> is the input
//        program name with its extension changed from .pcs or .dove to .pcl.
//        If you supply an interrupt script this will get checked and run
//        (if there are no errors in it) automatically immediately after the
//        main script is interrupted with Ctrl+c (but the interrupt script will
//        not get run if the main script terminates normally).
//    pcs-cli passwd
//        Allows the user to log in to a pardserver and change their password.
//    pcs-cli dump <script.pcs>
//        Pre-process, compile and link a script, and print every row of the
//        resulting program: row number, source line, op code name, words.
//    pcs-cli compile <script.pcs> <out.dove>
//        Pre-process, compile and link a script, and save it as a .dove (a
//        "static build", per the last paragraph of Rules_for_libs.txt).
//    pcs-cli decompile <in.dove|in.pcl> <out.pcs>
//        Create a runnable PCS source code file from a compiled script or lib.
//        The line numbers of the original source are preserved where possible
//        but original comments and white space elements before or after lines
//        or between arguments will be generic and not reflect the original
//        white space of the source the program or lib was compiled from.
//        Because compiled files are statically linked against all libraries and
//        headers that they use, all that library source code will also be
//        exported into the resulting PCS file. Likewise and pre-processor
//        macros in the original PCS files that the compiled files were made
//        from will not be preserved in the resulting decompiled file but their
//        numerical or string substituted values (i.e. post pre-processor code)
//        will be output and so will not contain and #include or #define
//        statements. This also means that the resulting PCS source code will be
//        just as 'static' as the compiled program - not being dependent on the
//        present of external libs or header files.
//    pcs-cli libinfo  <file.pcl>
//        Load a library with the linker's own loader (lib_load) and print its
//        header values and rows.
//    pcs-cli bindings <script.pcs|.dove>
//        Proves that the positional BINDINGS are made by script_check() (or,
//        for a .dove, by link_aot_flow_control()) and by nothing else.
//        First, straight after compiling and linking (or loading the .dove),
//        it counts the rows that already carry a binding -- the answer must
//        be 0.  Then it runs the checker (or the AOT linker) and verifies
//        every binding it made: each goto bound to the row of its label,
//        each call_func and def_func bound to the right function, each
//        loop_while/endloop and do_if/else/enddo bound to the right node.
//        Prints "BINDINGS OK <n> verified" or each failure.
//    pcs-cli pclname <name>
//        Prints the name that make_lib would save a library under when the
//        save dialog returns <name>: the extension is always .pcl.
//    pcs-cli locators <script.pcs>
//        Compile (and link) and check a script, then ask every row-index
//        locator -- func_id_from_idx, loop_index_open/close, if_index_open,
//        else_index, if_index_close -- for every function, loop and if-block
//        that script_check() recorded, and verify that each answers with the
//        right node.  Prints "LOCATORS OK <n> queries" or each failure.
//
//  The exit status is 0 if every stage succeeded and 1 otherwise, so the
//  test scripts can test it directly.
// ============================================================================

#define main pardclient_gui_main      // keep the client's own main() out of the way
#include "pardclient.c"
#undef main

#if IS_WINDOWS_NATIVE
static UINT orig_console_out_cp = 0;
static UINT orig_console_in_cp  = 0;

static void init_windows_utf8_console(void)
{
    orig_console_out_cp = GetConsoleOutputCP();
    orig_console_in_cp  = GetConsoleCP();

    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
}

static void restore_windows_console_cp(void)
{
    if (orig_console_out_cp != 0) SetConsoleOutputCP(orig_console_out_cp);
    if (orig_console_in_cp  != 0) SetConsoleCP(orig_console_in_cp);
}
#endif


static void cli_sigint_handler(int sig)
{
    (void)sig;
    if (graphics_mode_active) set_script_console_graphics_mode(FALSE);
    cli_reset_print_colour(); 
    _exit(130);
}


static int cli_change_pwd_flow(void)
// Handle change password feature via the CLI
{
 char currpwd[MAX_PASS_LEN] = {0};
 char newpwd1[MAX_PASS_LEN] = {0};
 char newpwd2[MAX_PASS_LEN] = {0};
 int rc = 1;

 while (1) {
     printf("Enter current (one-time) password: ");
     get_keyboard_input(currpwd, sizeof(currpwd), true);

     printf("Enter new password: ");
     get_keyboard_input(newpwd1, sizeof(newpwd1), true);

     printf("Confirm new password: ");
     get_keyboard_input(newpwd2, sizeof(newpwd2), true);

     rc = perform_change_pwd(currpwd, newpwd1, newpwd2);

     // Scrub CLI buffers after each attempt
     memset(currpwd, 0, sizeof(currpwd));
     memset(newpwd1, 0, sizeof(newpwd1));
     memset(newpwd2, 0, sizeof(newpwd2));

     if (rc == 0) {
         // Mirror GUI behaviour: log out so user can re-authenticate with new password
         perform_logout();
         break;
     }
     printf("Would you like to try again? (y/n): ");
     char retry[8] = {0};
     get_keyboard_input(retry, sizeof(retry), false);
     if (retry[0] != 'y' && retry[0] != 'Y') break;
 }

 return rc;
}

static void print_cli_check_error(int code, unsigned int linenum, const char *errmsg)
{
    switch (code) {
        case PCHK_TERMINUS:
            fprintf(stderr, "pcs-cli: FAILED to verify script [line %u]: Failed to terminate correctly.\n",
                    linenum);
            break;
        case PCHK_E_SYNTAX:
            fprintf(stderr, "pcs-cli: FAILED to verify script [line %u]: Syntax error. %s\n",
                    linenum, errmsg);
            break;
        case PCHK_E_COMMND:
            fprintf(stderr, "pcs-cli: FAILED to verify script [line %u]: Unrecognised or non-functional command '%s'.\n",
                    linenum, errmsg);
            break;
        case PCHK_E_FORMAT:
            fprintf(stderr, "pcs-cli: FAILED to verify script [line %u]: Not recognised as a valid Pardus command script.\n",
                    linenum);
            break;
        default:
            fprintf(stderr, "pcs-cli: Program Error: Script checker returned an unrecognised value (%d).\n",
                    code);
            break;
    }
}

static const char *cli_cfg_file = "pcscli.cfg";

static int cli_login(const char *cfgfile)
// To solicit login to the server where required using a config file and getting
// the password from the keyboard. The format of the config file must be this:
// [server]
// key=rq:rM>}U?@Lns47E1%kR.o@n%FcmmsL/@{H8]yf7
// address=192.168.1.50
// port=5555
// server_name=Pardus1
// username=alice
{
 ServerConfig cfg;
 char password[MAX_PASS_LEN] = {0};
 char newpwd1[MAX_PASS_LEN]  = {0};
 char newpwd2[MAX_PASS_LEN]  = {0};
 int rc = 1;

 // Load config and enforce that 'username' is present for CLI sessions
 if (load_server_config(cfgfile, &cfg, true) != 0) {
     return 1;
 }

 if (cfg.verbosity >= 0) {
     Default_verbosity = cfg.verbosity;
     Verbosity = Default_verbosity;
 }

 // Prompt for password interactively from the keyboard
 printf("Password for %s@%s: ", cfg.username, cfg.server_name);
 get_keyboard_input(password, sizeof(password), true);

 LoginParams params = {
     .server_key  = cfg.key,
     .address     = cfg.address,
     .port        = cfg.port,
     .server_name = cfg.server_name,
     .username    = cfg.username,
     .password    = password,
 };

 rc = perform_login(&params);

 // Handle mandatory one-time password (OTP) change seamlessly
 if (rc == 2) {
     int chg_rc = 1;

     while (1) {
         printf("Enter new password: ");
         get_keyboard_input(newpwd1, sizeof(newpwd1), true);

         printf("Confirm new password: ");
         get_keyboard_input(newpwd2, sizeof(newpwd2), true);

         chg_rc = perform_change_pwd(password, newpwd1, newpwd2);
         if (chg_rc == 0) {
             perform_logout();
             params.password = newpwd1;
             rc = perform_login(&params);
             break;
         }

         memset(newpwd1, 0, sizeof(newpwd1));
         memset(newpwd2, 0, sizeof(newpwd2));

         printf("Try setting a new password again? (y/n): ");
         char retry[16] = {0};
         get_keyboard_input(retry, sizeof(retry), false);
         if (retry[0] != 'y' && retry[0] != 'Y') {
             perform_logout();
             rc = 1;
             break;
         }
     }
 }

 // Wipe all password buffers and free config strings
 memset(password, 0, sizeof(password));
 memset(newpwd1,  0, sizeof(newpwd1));
 memset(newpwd2,  0, sizeof(newpwd2));
 free_server_config(&cfg);
 return rc;
}

static bool cli_cfg_explicit = false;

static char *find_cli_config_file(void)
// Returns a newly allocated path to pcscli.cfg if found, or NULL if none exists.
// Caller must g_free() the returned string.
{
    char *candidate = NULL;

    // 0. If the user explicitly supplied a path (or -c / --config), check it directly first
    if (cli_cfg_explicit && g_file_test(cli_cfg_file, G_FILE_TEST_IS_REGULAR)) {
        return g_strdup(cli_cfg_file);
    }

    // 1. Check the directory where the .pcs / .dove script resides
    if (LastScriptDir[0] != '\0') {
        candidate = g_build_filename(LastScriptDir, cli_cfg_file, NULL);
        if (g_file_test(candidate, G_FILE_TEST_IS_REGULAR)) return candidate;
        g_free(candidate);
    }

    // 2. Check the standard OS user config directory (e.g. ~/.config/pardus/pcscli.cfg)
    candidate = g_build_filename(g_get_user_config_dir(), "pardus", cli_cfg_file, NULL);
    if (g_file_test(candidate, G_FILE_TEST_IS_REGULAR)) return candidate;
    g_free(candidate);

    // 3. Check current working directory
    if (g_file_test(cli_cfg_file, G_FILE_TEST_IS_REGULAR)) {
        return g_strdup(cli_cfg_file);
    }

    // 4. Check the directory where the pcs-cli executable itself is installed
#if defined(_WIN32)
    gchar *pkg_dir = g_win32_get_package_installation_directory_of_module(NULL);
    if (pkg_dir != NULL) {
        candidate = g_build_filename(pkg_dir, cli_cfg_file, NULL);
        g_free(pkg_dir);
        if (g_file_test(candidate, G_FILE_TEST_IS_REGULAR)) return candidate;
        g_free(candidate);
    }
#elif defined(__linux__)
    gchar *exe_path = g_file_read_link("/proc/self/exe", NULL);
    if (exe_path != NULL) {
        gchar *exe_dir = g_path_get_dirname(exe_path);
        candidate = g_build_filename(exe_dir, cli_cfg_file, NULL);
        g_free(exe_dir);
        g_free(exe_path);
        if (g_file_test(candidate, G_FILE_TEST_IS_REGULAR)) return candidate;
        g_free(candidate);
    }
#elif defined(__APPLE__)
    uint32_t bufsize = 0;
    _NSGetExecutablePath(NULL, &bufsize);
    if (bufsize > 0) {
        gchar *raw_exe = g_malloc(bufsize);
        if (_NSGetExecutablePath(raw_exe, &bufsize) == 0) {
            gchar *real_exe = g_canonicalize_filename(raw_exe, NULL);
            gchar *exe_dir = g_path_get_dirname(real_exe ? real_exe : raw_exe);
            candidate = g_build_filename(exe_dir, cli_cfg_file, NULL);
            g_free(exe_dir);
            g_free(real_exe);
            if (g_file_test(candidate, G_FILE_TEST_IS_REGULAR)) {
                g_free(raw_exe);
                return candidate;
            }
            g_free(candidate);
        }
        g_free(raw_exe);
    }
#endif

    return NULL;
}

static int cli_passwd_command(void)
{
    char *cfg_path = find_cli_config_file();
    if (cfg_path == NULL) {
        fprintf(stderr, "pcs-cli: Could not find config file '%s'.\n", cli_cfg_file);
        return 1;
    }

    // Authenticate with the server first (also handles mandatory OTP change if rc == 2)
    int login_rc = cli_login(cfg_path);
    g_free(cfg_path);

    if (login_rc != 0) {
        tidy_up();
        return 1;
    }

    // Now that the session is authenticated, run the voluntary password change flow
    int rc = cli_change_pwd_flow();
    tidy_up();
    return (rc == 0) ? 0 : 1;
}

// ----------------------------------------------------------------------------
// opname(): the op code as text, for 'dump' and 'libinfo'.  Only the op codes
// that matter to the structure of a program are named; any other op code is
// printed as the command word that produced it.
// ----------------------------------------------------------------------------
static const char *opname(const TokenizedLine *t)
{
    switch (t->op) {
        case OP_UNKNOWN:      return "UNKNOWN";
        case OP_LABEL_DECL:   return "LABEL";
        case OP_DEF_VAR:      return "DEF_VAR";
        case OP_DEF_ARRAY:    return "DEF_ARRAY";
        case OP_DEF_FUNC:     return "DEF_FUNC";
        case OP_RETURN:       return "RETURN";
        case OP_CALL_FUNC:    return "CALL_FUNC";
        case OP_GOTO:         return "GOTO";
        case OP_EXIT:         return "EXIT";
        case OP_NULL_COMMAND: return "NULL_COMMAND";
        case OP_ADD_LIB:      return "ADD_LIB";
        case OP_MAKE_LIB:     return "MAKE_LIB";
        default:              return "cmd";
    }
}

// print_rows(): one line per row: "row <n>  line <l>  <OPNAME>  <words>".
// Rows are numbered from 1 for the reader; the program's own index is n-1.
static void print_rows(const TokenizedLine *rows, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        printf("ROW %4u  line %4u  %-12s ", i + 1, rows[i].orig_linenum, opname(&rows[i]));
        for (int a = 0; a < rows[i].argcount; a++) printf(" %s", rows[i].args[a].raw_text);
        printf("\n");
    }
}

// ----------------------------------------------------------------------------
// compile_script(): exactly what load_and_process_pcs_script() does for a
// .pcs file, minus the GTK calls: set LastScriptDir and script_pcsfile from
// the script's path, run the pre-processor (which writes _pppcs.pcs in the
// current directory), then compile _pppcs.pcs into ProgramSlots[0] -- which
// now links any libraries at its end.
// Returns 0 on success, 1 on failure (with a pcs-cli line saying where).
// ----------------------------------------------------------------------------
static int load_script_to_slot(const char *path, int slot_idx)
{
    char errmsg[ERRMSG_LEN * 3] = {0};
    unsigned int linenum = 0;
    const char *ext = strrchr(path, '.');
    PCS_Program *slot = &ProgramSlots[slot_idx];

    // Resolve to an absolute path first so relative CLI args still open after g_chdir()
    gchar *abs_path = g_canonicalize_filename(path, NULL);
    const char *use_path = (abs_path != NULL) ? abs_path : path;

    if (slot_idx == 0) {
        directory_from_path(use_path, LastScriptDir, PCS_PATH_MAX);
        if (LastScriptDir[0] != '\0') {
            g_chdir(LastScriptDir);
        }
        snprintf(script_pcsfile, PCS_PATH_MAX, "%s", use_path);
    }

    if (ext != NULL && !g_ascii_strcasecmp(ext, ".dove")) {
        int rc = load_compiled_program_from_disk(slot, use_path, errmsg);
        if (rc) {
            printf("pcs-cli: DOVE LOAD FAILED (%s) : %s\n", use_path, errmsg);
            g_free(abs_path);
            return 1;
        }
        if (Verbosity > 1 || slot_idx == 0) {
            printf("pcs-cli: DOVE LOAD OK%s (rows %u)\n",
                   slot_idx == 1 ? " [INTERRUPT]" : "", slot->TotalProgramLines);
        }
        g_free(abs_path);
        return 0;
    }

    FILE *fp = fopen(use_path, "rb");
    if (fp == NULL) {
        printf("pcs-cli: OPEN FAILED %s\n", use_path);
        g_free(abs_path);
        return 1;
    }

    int prev_int_req = Interrupt_script_request;
    Interrupt_script_request = (slot_idx == 1) ? 1 : 0;
    int r = script_preprocess(fp, &linenum, errmsg);
    Interrupt_script_request = prev_int_req;
    fclose(fp);

    if (Verbosity > 1) {
        printf("pcs-cli: PREPROCESS%s %s (code %d)%s%s\n",
               slot_idx == 1 ? " [INTERRUPT]" : "", r ? "FAILED" : "OK", r,
               r ? " : " : "", r ? errmsg : "");
    }
    if (r) {
        g_free(abs_path);
        return 1;
    }

    const char *pp_file = (slot_idx == 1) ? pp_path_interrupt : pp_path_main;
    fp = fopen(pp_file, "rb");
    if (fp == NULL) {
        printf("pcs-cli: cannot open %s\n", pp_file);
        g_free(abs_path);
        return 1;
    }
    free_compiled_program(slot);
    r = compile_pcs_to_memory(slot, fp, errmsg);
    fclose(fp);
    g_free(abs_path);

    if (Verbosity > 1) {
        printf("pcs-cli: COMPILE+LINK%s %s (code %d, rows %u)%s%s\n",
               slot_idx == 1 ? " [INTERRUPT]" : "", r ? "FAILED" : "OK", r,
               slot->TotalProgramLines, r ? " : " : "", r ? errmsg : "");
    }
    return r ? 1 : 0;
}

static int compile_script(const char *path)
{
    return load_script_to_slot(path, 0);
}

// ----------------------------------------------------------------------------
// check_and_run(): what btn_runpcs_click() does once a program is in
// ProgramSlots[0]: script_check() for a freshly compiled program, or
// link_aot_flow_control() for one loaded from a .dove (is_checked == 1),
// then script_run().
// ----------------------------------------------------------------------------
static int check_and_run(void)
{
    char errmsg[ERRMSG_LEN * 3] = {0};
    unsigned int linenum = 0;
    int r;
    PCS_Program *p = ProgramSlots;
    PCS_Program *int_p = ProgramSlots + 1;

    // 1. Verify main program ONLY (do not pre-populate global Varlist with int_p yet)
    if (p->is_checked) {
        r = link_aot_flow_control(p, &linenum, errmsg);
        if (r != 0 || Verbosity > 1) {
            printf("pcs-cli: AOT-LINK %s (code %d)%s%s\n", r ? "FAILED" : "OK", r,
                   r ? " : " : "", r ? errmsg : "");
        }
    } else {
        r = script_check(p, &linenum, errmsg);
        if (r == PCHK_ALL_GOOD) {
            p->is_checked = 1;
            if (Verbosity > 1) printf("pcs-cli: CHECK OK (line %u)\n", linenum);
        } else {
            print_cli_check_error(r, linenum, errmsg);
        }
    }
    if (r) { tidy_up(); return 1; }

    // 2. Authenticate if main script needs remote access
    if (p->needs_login && !logged_in) {
        char *cfg_path = find_cli_config_file();
        if (cfg_path == NULL) {
            fprintf(stderr, "ERROR: This script requires server access, but no %s was found.\n", cli_cfg_file);
            tidy_up();
            return 1;
        }

        printf("pcs-cli: Remote access required. Using config %s\n", cfg_path);
        int login_rc = cli_login(cfg_path);
        g_free(cfg_path);

        if (login_rc != 0) {
            fprintf(stderr, "ERROR: Server login failed; aborting script execution.\n");
            tidy_up();
            return 1;
        }
    }

    // 3. Execute the main script
    memset(errmsg, 0, sizeof(errmsg));
    r = script_run_timed(p, &linenum, errmsg, NULL);
    if (cli_saved_cursor_y > 0) {
        printf("\033[%dB\r", cli_saved_cursor_y);
        cli_saved_cursor_y = 0;
    }
    fflush(stdout);
    if (r != 0 || Verbosity > 1) {
        printf("pcs-cli: RUN %s (code %d, line %u)%s%s\n", r ? "FAILED" : "OK", r, linenum,
               r ? " : " : "", r ? errmsg : "");
    }

    // 4. Run the interrupt script automatically if main script was interrupted by Ctrl+C
    if (r == PRUN_ERROR && script_interrupted && Interrupt_script_active) {
        Verbosity = default_verbosity();
        script_interrupted = 0;
        run = 1;
        printf("pcs-cli: Starting Interrupt Script Execution...\n");

        linenum = 0;
        memset(errmsg, 0, sizeof(errmsg));
        int chk_ir = int_p->is_checked
                     ? link_aot_flow_control(int_p, &linenum, errmsg)
                     : script_check(int_p, &linenum, errmsg);
        if (chk_ir != PCHK_ALL_GOOD) {
            fprintf(stderr, "pcs-cli: INTERRUPT CHECK FAILED (line %u): %s\n", linenum, errmsg);
            r = 1;
        } else {
            if (int_p->needs_login && !logged_in) {
                char *cfg_path = find_cli_config_file();
                if (cfg_path == NULL) {
                    fprintf(stderr, "ERROR: Interrupt script requires server access, but no %s was found.\n", cli_cfg_file);
                    tidy_up();
                    return 1;
                }
                int login_rc = cli_login(cfg_path);
                g_free(cfg_path);
                if (login_rc != 0) {
                    fprintf(stderr, "ERROR: Server login failed for interrupt script.\n");
                    tidy_up();
                    return 1;
                }
            }

            linenum = 0;
            memset(errmsg, 0, sizeof(errmsg));
            script_in_progress = 1;
            int ir = script_run(int_p, &linenum, errmsg, NULL);
            script_in_progress = 0;

            if (cli_saved_cursor_y > 0) {
                printf("\033[%dB\r", cli_saved_cursor_y);
                cli_saved_cursor_y = 0;
            }
            fflush(stdout);

            if (ir == PRUN_ALL_GOOD) {
                printf("pcs-cli: INTERRUPT RUN OK\n");
            } else {
                fprintf(stderr, "pcs-cli: INTERRUPT RUN FAILED (code %d, line %u): %s\n",
                        ir, linenum, errmsg);
                r = ir;
            }
        }
    }

    // 5. Prepare logout header if still logged in, then clean up resources
    if (logged_in) update_message_header();
    tidy_up();
    return r ? 1 : 0;
}

// ----------------------------------------------------------------------------
// count_bindings(): the number of rows that carry a positional binding of the
// kind script_check() / link_aot_flow_control() make (test T17).
// verify_bindings(): checks every such binding against the tables those
// functions built.  Returns the number of failures; *n receives the number of
// bindings verified.
// These use only what PJT's original client also has, so test T17 can
// run them on both clients.
// ----------------------------------------------------------------------------
static uint32_t count_bindings(const PCS_Program *p)
{
    uint32_t n = 0;
    for (uint32_t r = 0; r < p->TotalProgramLines; r++) {
        const TokenizedLine *t = &p->CompiledProgram[r];
        switch (t->op) {
            case OP_GOTO: case OP_CALL_FUNC:
                if (t->argcount > 1 && (t->args[1].arg_opcode & (ARG_OP_GOTO | ARG_OP_FUNC))) n++;
                break;
            case OP_DEF_FUNC:
                if (t->argcount > 1 && (t->args[1].arg_opcode & ARG_OP_FUNC)) n++;
                break;
            case OP_LOOP_WHILE: case OP_ENDLOOP: case OP_DO_IF: case OP_ELSE: case OP_ENDDO:
                if (t->args[0].symbol_id != 0) n++;   // a node index other than the first
                break;
            default: break;
        }
    }
    return n;
}

static unsigned int verify_bindings(const PCS_Program *p, unsigned int *n)
{
    unsigned int bad = 0;
    const TokenizedLine *rows = p->CompiledProgram;
    *n = 0;
    for (uint32_t r = 0; r < p->TotalProgramLines; r++) {
        const TokenizedLine *t = &rows[r];
        uint32_t id;
        switch (t->op) {
        case OP_GOTO: {
            // a literal label must be bound to its label row
            unsigned int g = goto_index((char *)t->args[1].raw_text);
            if (g >= NGotos) break;                       // computed goto: stays dynamic
            (*n)++;
            if (!(t->args[1].arg_opcode & ARG_OP_GOTO) ||
                t->args[1].target_line != Gotolist[g].Array_Idx ||
                rows[t->args[1].target_line].op != OP_LABEL_DECL) {
                bad++; printf("BAD goto row %u -> %u (label row %u)\n", r + 1,
                              t->args[1].target_line + 1, Gotolist[g].Array_Idx + 1);
            }
            break; }
        case OP_CALL_FUNC: {
            unsigned int f = func_index((char *)t->args[1].raw_text);
            if (f >= NFuncs) break;                       // computed call: stays dynamic
            (*n)++;
            if (!(t->args[1].arg_opcode & ARG_OP_FUNC) || t->args[1].symbol_id != f ||
                rows[Funclist[f].start_idx].op != OP_DEF_FUNC ||
                strcmp(rows[Funclist[f].start_idx].args[1].raw_text, t->args[1].raw_text)) {
                bad++; printf("BAD call_func row %u\n", r + 1);
            }
            break; }
        case OP_DEF_FUNC:
            (*n)++; id = t->args[1].symbol_id;
            if (!(t->args[1].arg_opcode & ARG_OP_FUNC) || id >= NFuncs || Funclist[id].start_idx != r) {
                bad++; printf("BAD def_func row %u\n", r + 1);
            }
            break;
        case OP_LOOP_WHILE:
            (*n)++; id = t->args[0].symbol_id;
            if (id >= NLoops || Looplist[id].start_idx != r) { bad++; printf("BAD loop_while row %u\n", r + 1); }
            break;
        case OP_ENDLOOP:
            (*n)++; id = t->args[0].symbol_id;
            if (id >= NLoops || Looplist[id].end_idx != r) { bad++; printf("BAD endloop row %u\n", r + 1); }
            break;
        case OP_DO_IF:
            (*n)++; id = t->args[0].symbol_id;
            if (id >= NIfs || Iflist[id].start_idx != r) { bad++; printf("BAD do_if row %u\n", r + 1); }
            break;
        case OP_ELSE:
            (*n)++; id = t->args[0].symbol_id;
            if (id >= NIfs || Iflist[id].else_idx != r) { bad++; printf("BAD else row %u\n", r + 1); }
            break;
        case OP_ENDDO:
            (*n)++; id = t->args[0].symbol_id;
            if (id >= NIfs || Iflist[id].end_idx != r) { bad++; printf("BAD enddo row %u\n", r + 1); }
            break;
        default: break;
        }
    }
    return bad;
}

static void init_like_client_main(void)
// The client's own main() (which this harness replaces) marks every actuator
// slot as free before the GUI starts; without that, every slot reads as "in
// use" and def_actuator is refused.  These loops are copied from the client's
// main(), in the same order, so the harness starts from the same state.
{

 #if IS_WINDOWS_NATIVE || defined(_WIN32)
 WSADATA wsaData;
 if(WSAStartup(MAKEWORD(2, 2), &wsaData) != 0){
   // If the OS network stack fails, fallback before GTK allocates resources
   fprintf(stderr, "CRITICAL ERROR: Windows Network Subsystem failed to start.\n");
   exit(1);
  }
 SetConsoleOutputCP(CP_UTF8);
 SetConsoleCP(CP_UTF8);
#else
    setlocale(LC_ALL, "");
#endif

 // Do some initialisations
 setlocale(LC_NUMERIC, "C");
 if(non_gui_init()) exit(1); 

 // Allocate memory to the preview image pixel array
 PreviewWd = 640;
 PreviewHt = 480;
 PreviewImg_size = PreviewHt * PreviewWd;
 PreviewWd_stride = 3 * PreviewWd;
 PreviewImg_rgb_size = PreviewHt * PreviewWd_stride;
 PreviewImg = malloc(PreviewImg_rgb_size);
 if(PreviewImg == NULL){
     show_message("No RAM available for preview image pixels.", "Error: ", MT_ERR, 0);
     return;
 }
 clear_preview_image();

 // Make the QVGA and QQVGA buffer for saving smaller previews
 PreviewQ = malloc(3*320*240);
 if(PreviewQ == NULL){
     show_message("No RAM available for the smaller previews.", "Error: ", MT_ERR, 0);
     return;
 }

 // Create the preview overlay drawing buffer pixel array
 PreviewBuffer = malloc(PreviewImg_rgb_size);
 if(PreviewBuffer == NULL){
     show_message("No RAM available for preview buffer pixels.", "Error: ", MT_ERR, 0);
     return;
 }
 // Clear the buffer
init_preview_draw_bufer();

 //  Initialise the offline preview file path
 sprintf(Prevfile,"OfflinePreview.ppm"); 

 // Starting positions to plot a preview image in the preview area.
 // Top row
 Prevtile_start[0]  = 0;    // Top left corner
 Prevtile_start[1]  = 160*3; // One QQVGA step along
 Prevtile_start[2]  = 320*3; // Two QQVGA steps along
 Prevtile_start[3]  = 480*3; // Three QQVGA steps along
 // One QQVGA row down
 Prevtile_start[4]  = (120 * PreviewWd_stride) + 0;     // Left edge
 Prevtile_start[5]  = (120 * PreviewWd_stride) + 160*3; // 1 QQVGA step along
 Prevtile_start[6]  = (120 * PreviewWd_stride) + 320*3; // 2 QQVGA steps along
 Prevtile_start[7]  = (120 * PreviewWd_stride) + 480*3; // 3 QQVGA steps along
 // Two QQVGA rows down
 Prevtile_start[8]  = (240 * PreviewWd_stride) + 0;     // Left edge
 Prevtile_start[9]  = (240 * PreviewWd_stride) + 160*3; // 1 QQVGA step along
 Prevtile_start[10] = (240 * PreviewWd_stride) + 320*3; // 2 QQVGA steps along
 Prevtile_start[11] = (240 * PreviewWd_stride) + 480*3; // 3 QQVGA steps along
 // Three QQVGA rows down
 Prevtile_start[12] = (360 * PreviewWd_stride) + 0;     // Left edge
 Prevtile_start[13] = (360 * PreviewWd_stride) + 160*3; // 1 QQVGA step along
 Prevtile_start[14] = (360 * PreviewWd_stride) + 320*3; // 3 QQVGA steps along
 Prevtile_start[15] = (360 * PreviewWd_stride) + 480*3; // 3 QQVGA steps along
    
 return;
}

int main(int argc, char **argv)
{
    char errmsg[ERRMSG_LEN * 3] = {0};
    const char *cmd = NULL;
    const char *path = NULL;
    bool launched_by_file = false;
    static bool atexit_registered = false;
    const char *custom_cfg = NULL;
    int clean_argc = 1;
    const char *interrupt_path = NULL;

    if(!atexit_registered){
      atexit(restore_cli_graphics_echo_atexit);
      atexit_registered = true;
    }
    init_like_client_main();
    #if IS_WINDOWS_NATIVE
    init_windows_utf8_console();
    atexit(restore_windows_console_cp);
    #endif
    signal(SIGINT, cli_sigint_handler);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-c") || !strcmp(argv[i], "--config")) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: %s requires a config file path\n", argv[i]);
                return 2;
            }
            custom_cfg = argv[++i];
        } else {
            argv[clean_argc++] = argv[i];
        }
    }
    argc = clean_argc;
    argv[argc] = NULL;

    if (custom_cfg != NULL) {
        cli_cfg_file = custom_cfg;
        cli_cfg_explicit = true;
    }

    if (argc == 2 && !strcmp(argv[1], "passwd")) {
        return cli_passwd_command();
    }

    if (argc >= 2) {
        const char *ext1 = strrchr(argv[1], '.');
        // Shorthand: pcs-cli <script.pcs|.dove> [interrupt_script.pcs|.dove]
        if (ext1 != NULL && (!g_ascii_strcasecmp(ext1, ".pcs") || !g_ascii_strcasecmp(ext1, ".dove"))) {
            if (argc > 3) {
                fprintf(stderr, "error: too many arguments for shorthand run\n");
                return 2;
            }
            cmd = "run";
            path = argv[1];
            launched_by_file = true;
            if (argc == 3) {
                interrupt_path = argv[2];
            }
        } else if (argc >= 3) {
            cmd = argv[1];
            path = argv[2];
            if (!strcmp(cmd, "run") && argc >= 4) {
                interrupt_path = argv[3];
            }
        }
    }

    if (cmd == NULL || path == NULL) {
        fprintf(stderr, "usage: %s [-c <config_file>] [run] <script.pcs|.dove> [interrupt_script.pcs|.dove]\n"
                        "       %s [-c <config_file>] passwd\n"
                        "       %s [-c <config_file>] dump|compile|libinfo|decompile|recompile|locators|bindings|pclname <file or name> [out.dove|out.pcs]\n",
                argv[0], argv[0], argv[0]);
        return 2;
    }

    const char *ext = strrchr(path, '.');

    if (!strcmp(cmd, "run")) {
        if (interrupt_path != NULL) {
            Interrupt_script_request = 1;
            if (load_script_to_slot(interrupt_path, 1)) {
                Interrupt_script_request = 0;
                return 1;
            }
            Interrupt_script_request = 0;
            Interrupt_script_active = 1;
        }
        if (load_script_to_slot(path, 0)) {
            return 1;
        }
        return check_and_run();
    }

    if (!strcmp(cmd, "dump")) {
        if (compile_script(path)) return 1;
        print_rows(ProgramSlots[0].CompiledProgram, ProgramSlots[0].TotalProgramLines);
        return 0;
    }
    if (!strcmp(cmd, "compile")) {
        if (argc < 4) { fprintf(stderr, "compile needs an output file\n"); return 2; }
        if (compile_script(path)) return 1;
        if (save_compiled_program_to_disk(&ProgramSlots[0], argv[3], errmsg)) {
            printf("pcs-cli: DOVE SAVE FAILED : %s\n", errmsg);
            return 1;
        }
        printf("pcs-cli: DOVE SAVE OK %s\n", argv[3]);
        return 0;
    }
    if (!strcmp(cmd, "decompile")) {
        if (argc < 4) {
            fprintf(stderr, "usage: %s decompile <in.dove|in.pcl> <out.pcs>\n", argv[0]);
            return 2;
        }
        if (ext != NULL && !g_ascii_strcasecmp(ext, ".dove")) {
            if (load_compiled_program_from_disk(&ProgramSlots[0], path, errmsg)) {
                printf("pcs-cli: DOVE LOAD FAILED : %s\n", errmsg);
                return 1;
            }
            if (decompile_program_to_pcs(&ProgramSlots[0], argv[3], false, 0, errmsg)) {
                printf("pcs-cli: DOVE DECOMPILE FAILED : %s\n", errmsg);
                free_compiled_program(&ProgramSlots[0]);
                return 1;
            }
            printf("pcs-cli: DOVE DECOMPILED OK -> %s (rows %u)\n",
                   argv[3], ProgramSlots[0].TotalProgramLines);
            free_compiled_program(&ProgramSlots[0]);
            return 0;
        } else if (ext != NULL && !g_ascii_strcasecmp(ext, ".pcl")) {
            uint32_t ndefs = 0, nfuncs = 0;
            if (lib_load(&ProgramSlots[PCS_LIB_SLOT], path, &ndefs, &nfuncs, errmsg)) {
                printf("pcs-cli: PCL LOAD FAILED : %s\n", errmsg);
                return 1;
            }
            if (decompile_program_to_pcs(&ProgramSlots[PCS_LIB_SLOT], argv[3], true, ndefs, errmsg)) {
                printf("pcs-cli: PCL DECOMPILE FAILED : %s\n", errmsg);
                free_compiled_program(&ProgramSlots[PCS_LIB_SLOT]);
                return 1;
            }
            printf("pcs-cli: PCL DECOMPILED OK -> %s (defs %u, func rows %u)\n",
                   argv[3], ndefs, nfuncs);
            free_compiled_program(&ProgramSlots[PCS_LIB_SLOT]);
            return 0;
        } else {
            fprintf(stderr, "decompile requires a .dove or .pcl input file\n");
            return 2;
        }
    }
    if (!strcmp(cmd, "bindings")) {
        unsigned int linenum = 0, n = 0, bad;
        int r;
        PCS_Program *p = &ProgramSlots[0];
        if (ext != NULL && !strcmp(ext, ".dove")) {
            directory_from_path(path, LastScriptDir, PCS_PATH_MAX);
            if (load_compiled_program_from_disk(p, path, errmsg)) {
                printf("pcs-cli: DOVE LOAD FAILED : %s\n", errmsg); return 1;
            }
        } else if (compile_script(path)) {
            return 1;
        }
        printf("pcs-cli: BEFORE CHECK %u rows carry a binding\n", count_bindings(p));
        if (p->is_checked) {
            r = link_aot_flow_control(p, &linenum, errmsg);
            printf("pcs-cli: AOT-LINK %s\n", r ? "FAILED" : "OK");
        } else {
            r = script_check(p, &linenum, errmsg);
            printf("pcs-cli: CHECK %s\n", r ? "FAILED" : "OK");
        }
        if (r) { printf("pcs-cli: %s\n", errmsg); return 1; }
        bad = verify_bindings(p, &n);
        printf("pcs-cli: AFTER CHECK %u rows carry a binding\n", count_bindings(p));
        if (bad) { printf("pcs-cli: BINDINGS FAILED %u of %u\n", bad, n); return 1; }
        printf("pcs-cli: BINDINGS OK %u verified\n", n);
        return 0;
    }
#ifdef PCS_LIB_SLOT
    if (!strcmp(cmd, "pclname")) {
        char name[PCS_PATH_MAX];
        snprintf(name, sizeof(name), "%s", path);
        int c = lib_force_pcl_extension(name, sizeof(name));
        printf("pcs-cli: PCLNAME %s -> %s (%s)\n", path, name,
               c > 0 ? "changed" : (c == 0 ? "kept" : "too long"));
        return 0;
    }
    if (!strcmp(cmd, "locators")) {
        unsigned int linenum = 0, f, l, q = 0, bad = 0;
        if (compile_script(path)) return 1;
        if (script_check(&ProgramSlots[0], &linenum, errmsg) != PCHK_ALL_GOOD) {
            printf("pcs-cli: CHECK FAILED : %s\n", errmsg);
            return 1;
        }
        for (f = 0; f < NFuncs; f++) {          // every row of every function
            for (uint32_t r = Funclist[f].start_idx; r <= Funclist[f].end_idx; r++) {
                q++;
                if (func_id_from_idx(r) != f) { bad++; printf("BAD func %u row %u\n", f, r); }
            }
        }
        for (l = 0; l < NLoops; l++) {
            q += 2;
            if (loop_index_open(Looplist[l].start_idx) != l) { bad++; printf("BAD loop_open %u\n", l); }
            if (loop_index_close(Looplist[l].end_idx)  != l) { bad++; printf("BAD loop_close %u\n", l); }
        }
        for (l = 0; l < NIfs; l++) {
            q += 2;
            if (if_index_open(Iflist[l].start_idx) != l) { bad++; printf("BAD if_open %u\n", l); }
            if (if_index_close(Iflist[l].end_idx)  != l) { bad++; printf("BAD if_close %u\n", l); }
            if (Iflist[l].elsecount) {
                q++;
                if (else_index(Iflist[l].else_idx) != l) { bad++; printf("BAD else %u\n", l); }
            }
        }
        printf("pcs-cli: %u functions, %u loops, %u if-blocks\n", NFuncs, NLoops, NIfs);
        if (bad) { printf("pcs-cli: LOCATORS FAILED %u of %u queries\n", bad, q); return 1; }
        printf("pcs-cli: LOCATORS OK %u queries\n", q);
        return 0;
    }
#endif
#ifdef PCS_LIB_SLOT   // 'libinfo' exists only when built against a client
                      // that has the library system (test T10 also builds
                      // this harness against PJT's original client)
    if (!strcmp(cmd, "libinfo")) {
        uint32_t nd = 0, nf = 0;
        if (lib_load(&ProgramSlots[PCS_LIB_SLOT], path, &nd, &nf, errmsg)) {
            printf("pcs-cli: LIBINFO FAILED : %s\n", errmsg);
            return 1;
        }
        printf("pcs-cli: LIBINFO OK ndefs %u nfuncs %u\n", nd, nf);
        print_rows(ProgramSlots[PCS_LIB_SLOT].CompiledProgram,
                   ProgramSlots[PCS_LIB_SLOT].TotalProgramLines);
        free_compiled_program(&ProgramSlots[PCS_LIB_SLOT]);
        return 0;
    }
#endif
    fprintf(stderr, "unknown command %s\n", cmd);
    return 2;
}


