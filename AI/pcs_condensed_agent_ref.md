# PCS v1 — Condensed Agent Reference

*Distilled from the PCS (PARDUS Command Script) reference manual by P. J. Tadrous, for AI coding agents. Terse by design: rules, syntax, constraints, gotchas. Notation: `<x>` required, `[x]` optional, **varval** = a variable or a constant. Part A = language; Part B = hardware (actuators); Part C = imaging, graphics, I/O, trackers; Part D = complete command index.*

# PART A — THE LANGUAGE

## A0. What PCS is

- Flat, line-oriented, Turing-complete, compiled language for PARDUS robots (Arduino Mega2560 breakout board via `pardserver`; cameras via `pardcap`). Run by `pardclient` (GTK GUI) or `pcs-cli` (headless; system var `CLImode`=1). Local-only commands (variables, math, strings, files, graphics, trackers) run with no server login; hardware/camera commands need a pardserver login.
- File types: `.pcs` source, `.dove` compiled static program (libraries baked in), `.pcl` compiled library, `.pch` header (preprocessor-only). Preprocessor output goes to `_pppcs.pcs` (never load that file).
- The **script checker** is a linear pre-pass that executes nothing; it cannot catch run-time range/hardware errors. Run-time errors terminate the script unless a command is documented as non-fatal.
- Scripts drive real hardware: check pin assignments/limits and never assume AI-written hardware code is safe.

## A1. Lexical and file rules

- ASCII/UTF-8 text. **First coding line must be exactly `Pardus pcs 1`.** Every script/header must contain `exit`; **everything after the first `exit` is ignored free text** (use it as a scratchpad: plan, notes, JSON). `exit` ignores any args. `exit` inside a loop/function makes the checker report an unclosed block (it never sees the `endloop`/`return`).
- Line kinds: blank; `#` comment; coding line (one **prime command** + args). `#` starts a comment anywhere, even mid-word (`myvar#iable` → `myvar`), unless it is a preprocessor directive (`#define`/`#include` with `#` the first non-space char and the name immediately after it). No block comments. A literal `#` in a string must come from system var `Hash` or `cat_char s 35`.
- Words are split on runs of spaces/tabs (whitespace is collapsed and irrelevant, as is indentation). Limits: ≤31 words/line (command + 30 args); ≤2048 bytes/line (2047 after `#define` expansion); each argument ≤64 bytes (`MAX_ARGLEN`; BYTES, not characters — UTF-8 chars are 2–4 bytes, `═` is 3; over-length is silently truncated, possibly breaking UTF-8 or the closing `]` of an index); path-type args ≤`SCNFNAME_MAX` (\~4095 POSIX / 259 Windows); actuator and limit-switch names ≤31 bytes. **Case-sensitive everywhere.**
- **One operation per line; no expressions anywhere.** `a[i+1]`, `x > 20`, `f(g(y))` are never valid arguments. Compute stepwise (`add_var`, `math_var`, …) then use the result. The only secondary command is `if <varval>` on `goto` and `call_func`. `loop_while idx > 20` and `goto L if idx > 20` are illegal; use `set_condexp` then `loop_while Condexp` / `goto L if Condexp`.
- Goto label = `name:` alone on a line (colon attached, no space), constant, unique across the script; may be numeric (`1234:`).

## A2. Data types, varval, casting

**Types:** `int` (int32), `float` (C double), `string`. Only these. Int constants (via strtol): decimal, `0x` hex, `0` octal, `0b` binary, optional sign; out of int32 range → treated as a string. Float constants (via strtod): optional sign, digits with `.` (never a comma), optional `e`/`E` exponent; `1e3` is a number. A word followed by any extra non-space char is NOT a number (`5.4abc`, `5,431` → string).

**Strings have no delimiters.** Quotes are literal characters: `set_str m "3"` stores 3 chars `"3"`. No escape characters (`\n` is two chars). A literal string constant is one whitespace-free word ≤64 bytes. Build larger strings with `set_str`/`cat_str` (multi-word constants, not evaluated), `add_var` (appends the *value* of any varval), `cat_char` (append by code point; use 32 for space, 35 for #), and system vars `Space`, `Hash`.

**Word resolution rule:** when an argument is evaluated, if it names a defined user/system variable (or valid indexed element) it is that variable; else if it parses as a number it is a number; else it is a string constant. Consequences: a typo, or the name of an `undef_var`'d variable, silently becomes a string literal (no error unless the command needs a writable variable or a number). Numeric-looking words can never be variable names or (as literals) file names; put such names in string variables.

**varval** = variable or constant. A command needing a receptacle (to write a result) needs a user-defined variable, not a varval.

**Binary-choice varval** (used by `do_if`, `loop_while`, `goto…if`, `call_func…if`, ON/OFF-style args): numeric → nonzero=1, 0=0; numeric-looking string → its number; else exactly `TRUE`/`ON`→1, `FALSE`/`OFF`→0 (case-sensitive); anything else = error.

**Auto-casting (LHS master):**

- Float args accept ints (exact). Int args given floats are rounded half-away-from-zero `(int)(f+0.5)` / `(int)(f-0.5)` in all `*_var` commands (not necessarily elsewhere); out-of-int32 → error.
- In `add_var/sub_var/mul_var/div_var/set_var/math_var`, the RHS is converted to the **receptor's** type. `def_var int i 38000; mul_var i 1.25` gives 38000 (1.25→1). For fractional math use a float intermediate: `def_var float f i; mul_var f 1.25; set_var i f`. Int division truncates; int overflow wraps; divide by 0 terminates.
- String receptor ← number: numbers written as text (`%g`; floats ≤9 sig. figs.). A numeric *constant* typed in the script is copied exactly as written (`def_var string s 007` → `007`; `1.50` stays `1.50`); numeric *variables* are written as numbers.
- Number ← string: the whole string must be numeric (`5.431` ok; `5,431`, `5.4abc`, `micrometers` → error). Where a command does not need a number, an un-castable string just stays a string.
- `print_value` prints a string variable verbatim (`1.200`), but a typed numeric constant as a number (`1.200`→`1.2`).
- Comparisons (`set_condexp`): RHS cast to LHS type; string-vs-string only `==`/`!=` and compares text (`a==5.0` true but `a==5` false if `a` is the string `5.0`). To compare numerically, put the value in an int/float variable first.

## A3. Variables and arrays

- `def_var <type> <name> <init>` (scalar); `def_array <type> <name> <size> <init>` (every element = `<init>`). `<type>` may be a string variable; `<name>` must be a literal. `<size>` is a varval evaluated at run time (≥1) → **dynamic allocation**. `def_var` ≡ `def_array` size 1 (just clearer). Name: not a number-like word, no spaces or brackets, not already used (incl. system vars), not protected. Command names are legal variable names (`def_var int print_value 1`).
- **All variables are arrays; all are global.** 1-D only (emulate 2-D by raster: `set_var p y; mul_var p width; add_var p x; … arr[p]`).
- **Brackets rule (checker-enforced):** size-1 variables are used by bare name — never `v[0]`. Size ≥2 arrays are always used with `name[index]` — never bare — except in `def_array` and `undef_var` (and note `fset_var/fput_var/preview_draw_*/stats_array` etc. take `arr[start]`). Index = non-negative int-castable varval: constant, variable (strings castable), or another indexed element (`A[B[i]]`); **no expressions**; 0-based; out of range → fatal. Whole token (`A[B[i]]`) must stay ≤64 bytes.
- **Temporal scope:** a variable exists from its `def_*` line to its `undef_var`. The checker processes lines linearly in file order, performing every `def_*`/`undef_var` it meets regardless of branches/loops/functions, and executes nothing else. So (1) it **only ever sees initial values** — any variable used as an array size, goto label, function name, math function name, operator string, numeric-from-string etc. must be *initialised* to a valid value (e.g. `def_var int sz 2`, not 0; `def_var string s 0.0`, not `_`; label vars initialised to a real label); (2) definitions inside conditional code are still registered. Avoid `def_*`/`undef_var` inside loops/conditionals unless pairing is guaranteed (no escaping `goto` between them).
- Resize: back up → `undef_var arr` → `def_array` new size → restore. `undef_var` takes the bare array name.
- Protected (cannot be variable names): all system variables, `CurrAct`, `FITS`, `StageXY`, `StageXYZ`, `exit`, `ignore`, number-like words, anything with a bracket.

## A4. System variables (always exist; read-only unless a command sets them)

| Name | Type | Meaning |
| --- | --- | --- |
| `Space`, `Hash` | string | `" "`, `"#"` for building strings |
| `DateTime` | string | `YYYYMMDD_HHMMSS`, fresh on each evaluation |
| `Condexp` | int | Result of last `set_condexp` (default 0); persists until next `set_condexp` |
| `Sysval` | int | Exit code of last `execute_system` (-1 = failed to start) |
| `TRUE`/`ON`=1, `FALSE`/`OFF`=0 | int | constants |
| `CLImode` | int | 0 GUI, 1 pcs-cli |
| `NoTissue`/`HasTissue` | int | complementary flags from tissue-content thresholding (default NoTissue=1) |
| `PreviewReport` | int | Result of last preview grab (see C1) |
| `Ps_SatL_R/G/B`, `Ps_SatU_R/G/B`, `Ps_Min_*`, `Ps_Max_*` | int | after `get_preview_stats` |
| `Ps_Integral_*`, `Ps_Var_*`, `Ps_Mean_*`, `Ps_DCT_*`, `Ps_MAL_*`, `Ps_Focus_*`, `Ps_Npix_*` | float | after `get_preview_stats` (Focus\_\* also after `get_preview_focus`); `Ps_Msupport` = pixels in mask |
| `NSteppers`,`NBinaries`,`NMlevels`,`NServos`,`NSensors` | int | after `get_actuators` |
| `CurrActIdx`,`CurrActType` | int | current actuator; set by `def_actuator`/`act_getid`/any command naming an actuator (-1/-1 if not found) |
| `NReads`,`NWrites` | int | result of `fset_var`/`fput_var` (default -4; negative = error) |
| `NewLines`,`NBytes` | int | file-read/write byte/line bookkeeping |
| `Millis` | int | rolling ms counter (never negative, wraps \~24 days) |
| `PreviewPos`,`PreviewInc`,`PreviewVal` | int | pixel reader of VGA RGB preview buffer (see `set_previewpos`) |
| `PMRep1..4` | float | last PARDUS move report values (X,Y,Z step counts, last-moved) |
| `CurrTC` | float | latest tissue-content value |

## A5. Preprocessor (global scope, runs before checking)

- `#define <find> <replace>`: whole-word textual substitution (word = letters/digits/underscore; `MY_X` and `X2` are not hit by `#define X`). No args/multi-word. `<find>` ≤64 bytes, alnum/underscore. Duplicate `<find>` = error. Works anywhere, so it can **alias commands** (`#define PRINT print_value`). Convention: CAPS.
- `#include <file>`: no spaces in name, no angle brackets. Searched as written, then in the including script's directory. The file must start with `Pardus pch 1`, may contain comments/directives, and **must end with `exit`**. Recursive (depth ≤16); silent include guards (already-included files ignored).
- `stdpard.pch` (from the PARDUS repo) defines every symbol used in this manual: `ACT_*`, `INFO_*`, `LIM_*`, `SD_*`, `MLT_*`, `PWM_PSC_*`, `SRV_FREQ_*`, `SEN_*`, `CCHAN_*`, `GSI_*`, `OP_PREVIEW_DRAW_*`, `PDS_*`, `TKSET_*`, `TKGET_*`, `TRKS_*`, `TMC_*`, `FREE_PIN`(255), `SERVO_RESET`, … **Include it in nearly every hardware script**; without it use the raw integers (given in Part B/C).

## A6. Control of flow — first-class, dynamic, and meta-programmable

This is PCS's defining AI-friendly feature: **labels, function names and operators are ordinary strings that can live in variables and arrays**, so control flow is data.

**Structured blocks (command pairs, no braces):**

```
do_if <bool> … [else …] enddo       loop_while <bool> … endloop       def_func <name> … return
```

- `do_if`: one `enddo`, at most one `else` (before `enddo`). `loop_while`: condition re-evaluated each pass at the `loop_while` line; **if the condition is `Condexp`, recompute `set_condexp` inside the loop** (it is a stored value, not a live expression). `do_if`/`loop_while` blocks and their `enddo`/`endloop` may sit in different functions (legal but unwise). `goto` may break out of loops.
- Condition-building: `set_condexp <v1> <op> <v2>` sets `Condexp`. `<op>` is a **string varval** ∈ `== >= > <= < != && ||` (C semantics on cast values; `&&`/`||` act on truthiness). `Condexp` is itself a legal operand: `set_condexp Condexp == Condexp` forces 1, `!=` forces 0. Compose conditions by saving `Condexp` into int vars and combining: with e1,e2 ∈{0,1}: AND `e1+e2==2`; OR `sum>0`; XOR `e1!=e2`; XNOR `e1==e2`; NAND `sum<2`; NOR `sum==0`; `e1 > e2` = e1∧¬e2. Or directly `set_condexp a || b`.

**`goto <label> [if <varval>]`** — `<label>` may be a **string variable or array element** (`goto L[i]`). Context-aware, non-local: any label in the script is legal.

- Jump to a label in a calling function → call stack unwinds. Jump to a label in a function not in the active call path → stack resets to main level and that function's `return` then behaves like main's `return` (exits script). Same-function jumps are stack-neutral. Loop/`do_if` nesting counters are re-synchronised automatically; jumping into a block works (entering the `if` part: at `else`/`enddo` the block is treated as done; entering the `else` part: runs to `enddo`). Programmer must ensure variable state is valid at the landing point.
- **Label placement:** if the script has any `def_func` other than `main` (even uncalled), every label must be inside a `def_func…return` block or between the last `return` and `exit` — never between functions (checked only if the script contains a `goto`). With no functions (or only `main`) labels may go anywhere.
- **goto-subroutines with computed return** (informal functions): `def_var string rp _; set_var rp ret1; goto sub; ret1:` … `sub:` … `goto rp`. The subroutine may change `rp` and return elsewhere.

**`call_func <name> [if <varval>]`** — `<name>` may be a string variable/array element. Pushes a return point; recursion allowed (stack overflow possible). `def_func <name>` needs a literal, unique, whitespace-free name.

**Operator/function dispatch as data:** `math_var r <func> [v]`, `math_array`, `cmath_*` (`<cx_op>`), `set_condexp a <op> b`, `fourier`/`stats_array` mode strings, `def_var <type>` — all take their operator/mode as a string varval, which may be a variable/array element, so one line can perform different operations by changing a string.

**Jump-table / state-machine template:**

```pcs
Pardus pcs 1
def_array string Op 6 ==          # init must be a valid operator (checker!)
def_array string Fn 4 f_a         # init must be a real function
def_array string Lb 3 done        # init must be a real label
def_var int oi 0
def_var int fi 0
def_var int gi 0
def_var float x 0.0
def_var float y 0.0
def_func main
  set_var Op[0] ==
  set_var Op[1] <
  set_var Fn[1] f_b
  set_var Lb[1] done
  set_condexp x Op[oi] y
  call_func Fn[fi] if Condexp
  goto Lb[gi] if Condexp
return
def_func f_a
return
def_func f_b
return
done:
exit
```

Prefer index-driven tables of labels/functions/operators over nested if/else trees. Remember: initialise such arrays to valid values for the checker, and keep labels legal per the placement rule above (the `done:` label above sits between the last `return` and `exit`).

## A7. Functions

- Optional. If you define **any** function you must define `main`, and it must be the **first** `def_func`; all others come after it, unique names, no nesting, each with exactly one `return` (no args). Functions may be called before they are defined.
- No local variables, no parameters, no return values: pass arguments through dedicated global variables (use unique name prefixes per function) and read results from globals. Optionally `undef_var` temporaries before `return`.
- `return` from `main` ≡ `exit`. Without functions the script just runs top-to-bottom to `exit`.

## A8. Libraries (.pcl)

- A library is a PCS file whose first coding line after the magic line is `make_lib`. Loading it into pardclient and pressing Run **archives it** (to `.pcl`, via a save dialog) instead of running it. Comment out `make_lib` to test it as a normal script.
- Library source layout (strict):

```
Pardus pcs 1
make_lib                       # first, once
def_var/def_array …            # ONLY definitions before main; no labels, no add_lib
def_func main  … return        # test harness; not copied; no labels inside; must not define names clashing with others; others must not depend on it
def_func <fn1> … return        # ≥1 function after main; no gaps/labels between functions
exit                           # only `exit` after the last return
```

- Rules: full syntax check passes; no library inside a library (paste source instead); main's variables/arrays and actuators must not be used outside main (define a library's actuators inside its own init function); no function may call `main`; every function must pass the check with main's body removed. Not checkable at archive time: reliance on `CurrAct`, and function/label names computed at run time. Good practice: prefix all names (e.g. `rv_`), assume no hardware, put a self-test in main.
- Using a library: `add_lib <file.pcl>` lines (name = rest of line, spaces allowed) must come **immediately after the magic line, contiguous, before any other command or label**, and the script needs ≥1 formal function (`main`). Libraries share one global namespace with the script (duplicate names, or the same library twice → error). Searched as written, then in the folder of the *last script loaded* (watch stop scripts in other folders; prefer full paths). A `.dove` embeds its libraries.

# PART B — ACTUATORS AND HARDWARE

## B1. Actuator model

- Types (`ACT_*`): **1 Stepper, 2 Binary, 3 MLevel, 4 Servo, 5 Sensor**. Index 0–9 per type (max 10 each). Steppers 0,1,2 = stage X,Y,Z by convention; 254 = X+Y, 255 = X+Y+Z (only if defined). In motion commands use names `StageXY` / `StageXYZ` as `<act_id>`.
- `<act_id>` = actuator **name** (string constant or variable, case-sensitive) or the word `CurrAct` (uses `CurrActIdx`/`CurrActType`). Any command naming an actuator updates `CurrAct*`. Names: unique across all types, 1–31 bytes, no spaces; same text as a variable/function/label/command is allowed.
- **Lifecycle:** `get_actuators` (always first: the client's list is wiped each check/run, the server's persists across scripts/logins until the Arduino resets) → `def_actuator <type> <idx> <name>` (nascent) → `act_set` mandatory properties until mature → use. Using an immature actuator is fatal. Safe re-creation: `act_getname <type> <idx> s`, `set_condexp s != null`, `do_if Condexp`/`undef_actuator s`/`enddo`. `act_getid <name>` gives CurrActIdx=-1 if absent (existence test). `act_getid`/`act_getname` use the local list only.
- **Commands:** `act_accel`, `act_moveto`, `act_moveby`, `act_home`, `act_halt`, `act_get`, `act_set`, `act_unit`, `act_movereport`, `act_getid`, `act_getname`, `def_actuator`, `undef_actuator`, `get_actuators` (details in Part D).
- **`act_get`/`act_halt` interrupt motion**; most other orders sent to the Arduino during motion give a "Busy" error. `act_get` of Arduino-held items ("Ask Ardu") perturbs step timing — avoid mid-motion; TMC register reads are slowest. Prefer polling `act_movereport` (passive) to detect motion end; any order sent to the Arduino flushes an unread move report.
- **Pins:** digital pins 0 and 1 are forbidden (USB). Stepper limit-switch/diag pins are **analogue** pins given as plain numbers (no `A`). Pin value `255` (`FREE_PIN`) unassigns. To move a pin: set `FREE_PIN` first, then the new pin (except M-Level, see B4). `act_set` may cascade (unsetting a limit pin un-homes; changing a soft limit/stepcount re-evaluates `limstate`).
- **Readback sources:** *Server only*, *Ardu only (Ask Ardu)*, or *LG* (local server copy). `act_get` needs a user-defined receptacle variable of suitable type; extra `<var_y> <var_z>` only for 3-valued items (stepper `NHS`, `XYZCOORDS`; M-Level `TTABLE`).
- **Units (`act_unit`):** stepper unit 0 = atomic steps (1/maxmsdenom microstep), 1 = calibrated units; servo 0 = µs, 1 = degrees. Applies to *values you set*; `act_get` of coordinates/limits returns atomic ints and servo positions return µs regardless. Default 0 per type; persists for the login session (not reset between scripts) — always set it explicitly.

### Info-item numbers (`INFO_*`; `R`=read-only)

| Type | Items |
| --- | --- |
| Stepper | ENA\_STATE 1, HOMED\_TO 2 R, DIRN1 3, DIRN2 4, DIRNC 5, DIRNP 6 R, MSDENOM 7, MSDEMAX 8, PULWIDTH 9, BLSTEPS 10 R, BLSTMAX 11, CALIMAX 12, STEPCOUNT 13, SW\_LIM1 14, SW\_LIM2 15, NHS 16 (3 vals), ISD\_HOBL 17, LIMSTATE 18 R, PIN\_PUL 19, PIN\_DIR 20, PIN\_LIM1 21, PIN\_LIM2 22, PIN\_DIAG 23, PIN\_ENA 24, NAME\_LS1 25, NAME\_LS2 26, NAME\_SW1 27, NAME\_SW2 28, NAME\_UNI 29, MATURITY 30 R, XYZCOORDS 31 (3 vals), DURALAM 32 R, STEPLAM 33 R, ONESTEP 34 R, FASTMSD 35, MMODE 36, TMCSGVAL 37 R, TMCSTATUS 38 R, TMCCHCONF 39 R, SG\_BUFFPC 40, SG\_THRESH 41, TURBO\_BST 42, TMCMSCNT 43 R, TENSTEPS1 44, TENSTEPS2 45, UNIT\_TYPE 46 R (set via `act_unit`), PIN\_MS1 47, PIN\_MS2 48, PIN\_MS3 49, SD\_TYPE 53 |
| Binary | BIN\_STATE 1, ON\_VAL 3, OFF\_VAL 4, PIN\_BIN 24, MATURITY 30 R |
| M-Level | ML\_STATE 1 R, ML\_TYPE 3, ML\_PSC 4, ML\_TIMER 6 R, ML\_TTABLE 8 (3 vals), PIN\_P0..P5 = 19..24, PIN\_P6 47, PIN\_P7 48, ML\_FREQ 33 R, ML\_LEVEL 49, MATURITY 30 R |
| Servo | SRV\_STATE 1 R, SRV\_TIMER 6 R, SRV\_FREQ 9, SRV\_THMAX 10, SRV\_THMIN 11, SRV\_CALIB 12 R, SRV\_MIN 14, SRV\_MAX 15, SRV\_ISD 17, SRV\_PINS 19, SRV\_PINP 21, SRV\_AFB 33 R, SRV\_QPOS 44, SRV\_SPOS 45, SRV\_ACCEL 49, MATURITY 30 R |
| Sensor | SEN\_TYPE 3, SEN\_PIN 21, SEN\_VAL 33 R, MATURITY 30 R |
| Camera (`camera_get`) | CAM\_MIN 50, CAM\_MAX 51, CAM\_CURR 52 |

### Constants

`ACT_STEPPER 1`, `ACT_BINARY 2`, `ACT_MLEVEL 3`, `ACT_SERVO 4`, `ACT_SENSOR 5` · `ENA_DISABLED 0`, `ENA_ENABLED 1`, `ENA_SPINNING 2` · `LIM_UNSET 0, SWTC1 1, SWTC2 2, NULL1 3, NULL2 4, SOFT1 5, SOFT2 6, ZERO 7 (tried negative step count), STALL 8` · `SD_UNDEF 0, SD_A4988 1, SD_DRV8825 2, SD_TMC2209 3` · `BIN_OFF 0, BIN_ON 1, BIN_UNSET 3` · `MLT_PARALLEL 0, MLT_PWM 1, MLT_UNSET 3` · `PWM_PSC_1 0, _8 3, _32 5, _64 6, _128 7, _256 8, _1024 10` · `SRV_FREQ_MID50 1, _MID200 2, _MID333 3` · `SEN_DIGITAL 0, SEN_ANALOGUE 1, SEN_UNSET 3` · `FREE_PIN 255`, `SERVO_RESET`=`MLEVEL_RESET`=2147483647.

## B2. Stepper (type 1)

- Drivers: TMC2209 (Mega only; StallGuard4, UART info), DRV8825 (≤1/32 microstep), A4988 (≤1/16). **First `act_set s INFO_SD_TYPE <SD_*>`; nothing else is settable until then, and it can never change** (undef/redefine instead). Driver-specific items are refused for the wrong driver: TMC-only = `PIN_DIAG, TMCSGVAL, TMCSTATUS, TMCCHCONF, SG_BUFFPC, SG_THRESH, TURBO_BST, TMCMSCNT`; non-TMC-only = `PIN_MS1/2/3`.
- **Maturity** (each satisfied item = 1 point; need ≥7 for TMC, ≥10 non-TMC): `SD_TYPE, PIN_PUL, PIN_DIR, PIN_ENA, DIRN1, DIRN2` + one homing route (`PIN_LIM1` and/or `PIN_LIM2`; **or** null-homing: `NHS` with nhs1>0 and nhs0 and/or nhs2>0) + (A4988/DRV8825 only) `PIN_MS1,2,3`. A mature stepper can **only be homed** until homing succeeds.
- Bring-up skeleton: `get_actuators; def_actuator 1 0 X_Axis; act_set CurrAct INFO_SD_TYPE SD_TMC2209; act_set CurrAct INFO_PIN_PUL <p>; … INFO_PIN_DIR; INFO_PIN_ENA; INFO_DIRN1 <0|1>; INFO_DIRN2 <0|1>; INFO_PIN_LIM1 <analogue#>; INFO_PIN_LIM2 <analogue#>`; then `act_accel 1 …` and `act_home`.
- `DIRN1`/`DIRN2`: pin level (0/1) that moves toward limit 1/2. `DIRNC` = current direction (used by `act_moveby`).
- **Microsteps and units:** `MSDEMAX` (maxmsdenom, ≥1, server-only) defines the atomic step = 1/maxmsdenom microstep; `MSDENOM` ≤ MSDEMAX (refused if MSDEMAX ≤1). `ONESTEP` (R) = maxmsdenom/msdenom atomic steps per real step. All distances/positions are rounded to whole real steps: `n = (uint32)(0.5 + d/onestep)` (e.g. d=15, onestep=32 → 0 steps; d=16 → 1). Don't change msdenom mid-session without care (position drift). `CALIMAX` (maxcalibri) = atomic steps per calibrated unit, float ≥ 1e-3. Exceptions: `NHS` steps are always integer steps at the current msdenom; `BLSTEPS` (R) is in current-msdenom steps (= maxblsteps/onestep); `BLSTMAX`, `TENSTEPS1/2`, `SW_LIM*`, `STEPCOUNT` are atomic steps. Naming items: `NAME_LS1/2` limit-switch names (≤31 bytes), `NAME_SW1/2` soft-limit names, `NAME_UNI` unit name. Forbidden values: maxmsdenom<1, maxcalibri<1e-3, negatives (unless stated).
- **Homing:** `act_home <id> <LIM_SWTC1|SWTC2|NULL1|NULL2>` returns immediately; verify later with `act_get … INFO_HOMED_TO` (equals requested type on success; LIM\_UNSET if failed/halted; may report busy) or `act_movereport`. Homing resets step count to 0; motion is then only allowed away from the home end; a request below 0 stops with `LIM_ZERO`. Null homing runs `nhs0` (dirn2), `nhs1` (dirn1), `nhs2` (dirn2) steps with no physical switch; the resulting `homed_to` is the *opposite* of the last move direction; it checks no end-stop unless a limit pin is also set (hardware-damage risk). For free-rotation axes, after homing `act_set … INFO_STEPCOUNT <large positive>`. Soft limits: `SW_LIM1` stops when stepcount ≥ value, `SW_LIM2` when ≤; hardware switches override them; cannot home to soft limits. `ISD_HOBL` = inter-step delay (µs) for homing/backlash; `FASTMSD` = fast-phase homing microstep denom.
- **Motion:** `act_accel 1 <prop_acc 0..1> <isd_min µs (>1, realistically ≥~200)> <isd_max_min µs (≫ isd_min)>` once per login before any non-homing move; then `act_moveto`/`act_moveby`. After motion read `STEPCOUNT`, `DURALAM` (last duration µs), `STEPLAM`, `LIMSTATE`, `ENA_STATE` (2 moving, 1 rest, 0 disabled). `INFO_ENA_STATE` can be set (0 to disable coils). A stall (TMC) reports an error without naming the motor: cold-read each motor's `LIMSTATE` (LIM\_STALL=8).
- **TMC extras:** `SG_THRESH` (0 = StallGuard off), `SG_BUFFPC` (<100 off), `MMODE` (stealthy/precise), `TURBO_BST` (0 off; 1 = boost for next move then auto-reset; ≥2 = latched until you set 0 or 1; stationary only, 100 ms enforced pause; raises holding current — don't leave on), `TMCSTATUS`/`TMCCHCONF` return int32 casts of the registers (sign: status<0 = standstill flag set; chconf<0 = short-protection-low-side OFF) and refresh the local copy used by `tmc_status_parse`/`tmc_chconf_parse`.

## B3. Binary (type 2)

Mature when `PIN_BIN`, `ON_VAL`, `OFF_VAL` set (each 0/1). `act_set b INFO_BIN_STATE <v>`: nonzero writes `on_val` to the pin, 0 writes `off_val` (so inverted logic is a configuration, and `on_val==off_val` pins the state). `act_halt` has no effect.

## B4. M-Level (type 3)

**Set `INFO_ML_TYPE` first (once; `MLT_PARALLEL` or `MLT_PWM`); until then nothing else is settable.** Max 10. Level is set with `act_set m INFO_ML_LEVEL <n>`; an unchanged level is not resent (see below). `MATURITY` counts: type +1, p0 +1, (PWM) table row +1; parallel accepts levels at ≥2, PWM at ≥3.

**Parallel:** 1–8 digital output pins `INFO_PIN_P0..P7` (any non-reserved pin, D2–D13 Uno/Nano, D2–D53 Mega; no timers used). Level `0..2^n-1` is a binary pattern (p0 = LSB). Assigned pins must be contiguous from p0; free pins from the highest down (a gap makes every level refused). `INFO_ML_PSC` accepted but unused.

**PWM:** only `INFO_PIN_P0`, on a PWM-capable pin; level = **row** of a server-side translation table you build:

- `act_set m INFO_ML_PSC <PWM_PSC_*>` (default `PWM_PSC_8`) **only while p0 is unassigned**; then `act_set m INFO_PIN_P0 <pin>` (pin set LOW, inactive). 16-bit timers refuse PSC 32/128.
- `act_set m INFO_ML_TTABLE <row> <freq Hz 0..8000000> <duty 0..4999>` — rows append-only (row 0 first, then n, never gaps; existing rows overwritten; none deleted). Duty 0–1000 = per mille; **1001–4999 = special-effect codes** (2000, 3000 refused). Fixed meanings, tested in order: duty 0 → constant LOW ("clean silence"; on 16-bit timers keeps the pin on its timer); duty 1000 → constant HIGH; freq 0 → constant LOW (disconnects the timer).
- `act_get m INFO_ML_TTABLE <rowvar> <fvar> <dvar>` — errors are non-fatal and signalled by `rowvar` changing: +1 = no table, +2 = no such row (then outputs are garbage). All receptors int variables.
- **Level checks (server, in order):** type set; mature; (parallel) no pin gaps; row exists / level ≤2^n−1; **same level as last sent with identical stored values → not sent (reports success)**. So a timed noise/burst selected twice in a row sounds once: select another row in between or `act_halt` (clears the record). The server also doesn't know of kill-switch, ended timed noise, or an effect ended by another actuator on the same timer — same remedy.
- `act_halt m`: pins LOW/timer disconnected, effect ended, state 0, last-level record cleared (`ML_LEVEL` reads 2147483647 after creation/halt). `ML_STATE`=1 once a level order was carried out (a 4000–4999 code leaves it unchanged); 0 after creation, p0 assignment, halt, kill switch, last pin freed.
- **Pins/timers** (PWM M-Level and servo use the same pins):

| Board | Usable PWM pins (timer) | Refused |
| --- | --- | --- |
| Uno/Nano | D3 (T2, 8-bit); D9, D10 (T1, 16-bit) | D11 (T2 channel A = TOP reg), D5/D6 (T0 = millis), others (no timer) |
| Mega2560 | D3, D5, D2\* (T3); D6, D7, D8 (T4); D9 (T2); D11, D12\* (T1); D44, D45, D46 (T5) | D10 (T2 A), D4/D13 (T0), all others |

\*With `FULL_SAFETY` (automatic on Mega) D2 (kill-switch input) and D12 (motor-voltage relay) are locked → 10 usable pins (max 12 otherwise). Refusals report "Pin number out of range". Pins above D13 (Uno/Nano) / D53 (Mega) and D0/D1 are always refused.

- **Timer physics:** 16-bit timers 1,3,4,5 = phase-and-frequency-correct, `TOP=ICR`, prescalers 1,8,64,256,1024. Timer 2 = fast PWM, `TOP=OCR2A`, all 7 prescalers. `F=16 MHz`, `N`=prescaler. **All pins on a timer share one prescaler and one TOP (frequency)**; each pin keeps its own compare value (duty) computed only when *its* level is set → the last member to set a level decides the group's frequency and the others' duty drifts. Mega groups: {D11,D12}, {D2,D3,D5}, {D6,D7,D8}, {D44,D45,D46}; Uno/Nano: {D9,D10}. At most one special effect per timer.
- **Formulas (integer arithmetic in firmware):** 16-bit: `f` capped at `F/(2N)`; `TOP=floor(F/(2·N·f))` clamped 1..65535; `f_out=F/(2·N·TOP)`; `OCR=floor((TOP·d+500)/1000)` (0 → LOW; ≥TOP and TOP>1 → HIGH); pulse = OCR·N/8 µs; period = TOP·N/8 µs; duty step = 1000/TOP ‰. Timer 2: `f` capped at `F/N`; `TOP=floor(F/(N·f))−1` clamped 1..255; `f_out=F/(N·(TOP+1))`; `H=floor(((TOP+1)·d+500)/1000)` (0 → LOW; `OCR2B=H−1`; H=TOP+1 → HIGH); pulse = H·N/16 µs. Requests round to the nearest achievable frequency **at or above** the request (clamped to prescaler range). Ceilings (`ML_FREQ`): 16-bit `F/(2N)` = 8,000,000/1,000,000/125,000/31,250/7,812 Hz for N=1/8/64/256/1024; T2 `F/N`. At TOP=1 (16-bit) only duty 0/1000 exist; at TOP=2 (16-bit) / highest T2 frequency only 0/500/1000 per mille. Lowest freq in table: 1 Hz for N=256/1024 on 16-bit. Duty resolution: 10‰ steps up to \~80 kHz (N=1, 16-bit) / 10 kHz (N=8) / 1.25 kHz (N=64); 1‰ steps up to 8 kHz/1 kHz/125 Hz.
- **Special-effect duty codes** (`duty = 1000·E + P`; Arduino performs them autonomously by timer overflow interrupt):

| Code | Effect | Row freq | P |
| --- | --- | --- | --- |
| 1001–1999 | **NOISE** until ended: each clock period a pulse of width P‰ is present/absent with p=½ (first always present) | noise clock ≤16,000 Hz | pulse width ‰ (1–999) |
| 2001–2999 | **TIMED NOISE**: noise for d ms then silence | noise clock ≤16,000 | `P=100c+d`: d ms (0 = 100), width class c (0–8: pulse width halves per class: 500,250,125,62,31,15,7,3,1‰; 9 = 999‰ telegraph); codes 2001–2099 = 1–99 ms at 500‰ |
| 3001–3999 | **BURST+HISS**: 10 ms noise at 500‰, then noise at P‰ until ended; sharp start | noise clock ≤16,000 | hiss width ‰ |
| 4000–4999, 16-bit pin | **GLIDE TIME** (setting; nothing audible): following plain levels glide linearly in *period* over \~P ms; 4000 = off | ignored | ms |
| 4000–4999, Timer-2 pin | **ATTACK STEP** (setting): continuous/≥30 ms noise starts at ⅛ width, grows by that every P ms (≤8 widths); 4000 = sharp | ignored | step ms |

Effects end when: another level (plain duty or noise code, not 4000+) on the same timer is set; `act_halt`/pin freed/destroyed; kill switch; timed noise expires. Noise/burst with freq 0 or P=0 → refused at level time. A noise code on a timer carrying a servo → "Read only". The glide/attack setting belongs to the timer *and the actuator that last set/used it* (survives halt; re-send 4000+P after sharing a timer or moving the pin). Glide not made (level set immediately) when TOP<256, steps<2, no glide time, not owner, servo present, pin not on timer, target is silence/HIGH/f=0, or OCR=0/≥TOP. Clean onset (16-bit): from a silent-but-connected pin a pulse train starts with a whole first pulse \~1.5 periods later; silence with duty-0 rows (not `act_halt`) when the next sound must start promptly. On Timer 2 noise clocks ≥7813 Hz force prescaler 8. Timer 0 is never used.

- **Sharing rules (all "Read only" refusals):** assigning an M-Level `p0` to a timer carrying a servo or an *active* PWM M-Level; assigning a servo pin to a timer with an active PWM M-Level or servos of another band; noise on a servo timer; setting `ML_PSC` while p0 assigned. If both an M-Level and a servo share a 16-bit timer, **assign the M-Level pin first, the servo pin last**; the servo's frame (50/200/333 Hz) and prescaler 8 then rule (row frequency ignored; duty is against TOP 20000/5000/3003). Moving an active PWM pin to another pin of the same timer is refused (halt first). Freeing/moving a pin releases the old one (LOW, high-impedance input); the new pin stays LOW until the next level order; `INFO_ML_PSC` and glide settings do not follow the move.
- Freeing p0 (`FREE_PIN`) makes it immature but keeps table and prescaler.
- Typical audio script: `def_actuator ACT_MLEVEL 1 Voice; act_set Voice INFO_ML_TYPE MLT_PWM; act_set Voice INFO_ML_PSC PWM_PSC_8; act_set Voice INFO_PIN_P0 9; act_set Voice INFO_ML_TTABLE 0 0 0; act_set Voice INFO_ML_TTABLE 1 440 500; act_set Voice INFO_ML_LEVEL 1; sleep 1.0; act_set Voice INFO_ML_LEVEL 0`. (For PC-speaker beeps with no hardware use the local `beep` command.)

## B5. Servo (type 4)

- Position by pulse width (µs) at 50/200/333 Hz frame; 3- or 4-wire hobby servos only. Max 10. Mature when `INFO_SRV_PINS` (signal pin) set. Pins as for PWM M-Level (same timers/refusals). **Set `INFO_SRV_FREQ` (`SRV_FREQ_MID50|MID200|MID333`) before assigning the pin** (or free pin first). All servos on one timer must share a band. Pin assignment configures the timer but sends no pulses; the first `act_moveto` energises the servo (it then holds position until `act_halt`).
- Frames: 16-bit timer, prescaler 8: 50 Hz/20000 µs, 200 Hz/5000, 333 Hz/3003, pulse resolution **1 µs**, max pulse 19999/4999/3000. Timer 2: 50 Hz band actually 61.04 Hz (tick 64 µs, max 16320), 200 Hz band 250 Hz (tick 16 µs, max 3984), 333 Hz band 334.22 Hz (tick 16 µs, max 2976); pulse rounded to nearest tick. Steps over range: 16-bit = `srv_max−srv_min`; T2 = `floor((max+tick/2)/tick)−floor((min+tick/2)/tick)`. Calibrate on the timer you will use.
- **Calibration (must be physically accurate):** `INFO_SRV_MIN` (default 544 µs) / `INFO_SRV_MAX` (default 2400; server accepts ≤20000/5000/3000 per band) = linear range limits; `INFO_SRV_THMIN` (default 0°) / `INFO_SRV_THMAX` (default 180°) = angles at those limits; `INFO_SRV_CALIB` (R) = µs per degree = (max−min)/(thmax−thmin). Angle from µs: `theta = theta_min + (musec − srv_min)/calibri`. Hum/buzz at an extreme = past the linear range.
- **Motion model (3 steps):** (1) `act_accel 4 <s_acc 0..255> <duration µs>` *before queueing* (the profile decision is stored in each queue entry; re-queue after changing it; linear ramp; ramp steps `=(s_acc·max_ramp_steps)/256`; if `duration` ≤ native time `isd_min×steps` the servo runs at native speed); (2) `act_set <srv> INFO_SRV_QPOS <pos>` lists it (units per `act_unit`: 0 µs int, 1 degrees float); re-set overwrites, `SERVO_RESET` removes it; `act_get … QPOS` reads the server shadow (µs, `SERVO_RESET` if unqueued); (3) `act_moveto <any srv> TRUE` fires **all queued servos** (queue then cleared); `FALSE`/`OFF`/`0` wipes the queue without moving. Servos that can be paced to `duration` move together (Bresenham); those too slow/fast for it run native and snap to target at the end. Without a profile (`duration` 0) all start together and finish separately at native speed.
- `INFO_SRV_ISD` = native µs per atomic step. `INFO_SRV_SPOS` silently overwrites the recorded current position without moving (a new servo is assumed at 1500 µs — set it to the true position before the first move to avoid a violent first motion); `act_get SPOS` reads it. 4-wire: `INFO_SRV_PINP` = analogue pin (no `A`); `INFO_SRV_AFB` returns 0–1023 raw ADC — you must map it to µs yourself.

## B6. Sensor (type 5)

`act_set s INFO_SEN_TYPE SEN_ANALOGUE|SEN_DIGITAL` first (permanent); `act_set s INFO_SEN_PIN <n>` (analogue: number without `A`); `act_get s INFO_SEN_VAL v` (int). Hot-swap pin within a type: `INFO_SEN_PIN FREE_PIN` then new pin. Change type: `undef_actuator`, redefine. `act_halt` n/a.

# PART C — IMAGING, GRAPHICS, I/O, MATH, TRACKERS

## C1. Cameras, preview and capture

- Cameras = `pardcap` servers, selected with `select_camera <n>` (1-based, default 1). Camera settings, correction images and `preview_tile` are **per camera**. `select_camera` does not refresh the GUI: call `get_camsettings` + `get_resolutions` afterwards if the GUI should follow. Loading/checking a script resets some camera/correction state.
- `camera_get <ctrl_id> <var> [50|51|52]` (50/51 = min/max, answered locally; 52 or omitted = current, asks the server); `camera_set <ctrl_id> <int>` (value within min..max). Control IDs are the v4l2 IDs listed by `get_camsettings`. min==max ⇒ read-only (some controls are read-only in context, e.g. exposure while auto-exposure is on).
- **Full capture (`grab_image`, no args)** needs first: `save_format`, `save_resolution`, `save_root`, `save_path` (directory, must contain a separator, no trailing slash), `frames_to_skip`, `frame_number`, `frames_to_average`. Optional: `save_doubles`, `jpeg_quality`, `df_correction`, `ff_correction`, `corrections_mask`, `tc_threshold`, `save_coords`, `grabber_predelay/timeout/retries`, `average_scale_means`. File name = `<root><frame#><ending>`; frame# auto-increments (`frame_number` sets it).
- Formats (`save_format yuv|pgm|bm8|int|ppm|bmp|png|jpg`, literal constant, case-sensitive): yuv → `_yuyv.raw` (16 bpp YUYV, no averaging/doubles/corrections); int → `_i.dou/_i.fit` (HSI intensity); pgm/bm8 → `_y.pgm/_y.bmp`; ppm/bmp/png → `_rgb.*`; jpg → `.jpg` (direct if MJPEG stream and no averaging, else re-encoded). With `save_doubles ON` mono→`_y.dou`, colour→`_r/_g/_b.dou` (+`.qih` headers); `FITS` → `.fit` instead (FITS wins over ON). Changing `save_resolution` invalidates loaded correction images.
- **Preview grabs** (`get_preview`, `get_preview_focus`, `get_preview_stats ON`, `get_preview_tc`) **ignore** `frames_to_skip`, averaging and `grabber_timeout` (one frame, \~100 µs timeout). So: `skip_frames 3` before them to get a current frame (grabber queue = 2), and `sleep 0.2`+ after a full-size grab. They fail softly: read `PreviewReport` (reset to 1 at each capture): 1 OK, 2 select error, 3 timeout, 9 no stream, 10 busy, 138 colour-convert failed, 139 zoom failed. Decide: retry, ignore, or exit. Other errors (no camera, memory) are fatal.
- `get_preview <zoom ON|OFF> <cx> <cy> [display?]`: ON = crop around full-res centre; OFF = scaled whole. Use `display?=0` then draw overlays/track then `preview_show` to avoid flicker.
- Stats: `get_preview_stats ON|OFF` (ON = new frame; OFF = recompute stats of last preview) fills `Ps_*`. **Focus statistic** (`Ps_Focus_*`) chosen with `prv_toggle`: none of `fc_dct`/`fc_mal` → variance; `fc_dct` → mid-frequency DCT power; `fc_mal` → mean absolute Laplacian; both → DCT×MAL; `fc_vmul` multiplies the chosen statistic by variance (no effect if neither dct nor mal). `get_preview_focus` = fast, updates only `Ps_Focus_*`. Masks restrict stats; `hgm_restrictvar` limits mean/variance to pixels inside saturation limits.
- **Tissue content (TC):** do `get_preview_tc OFF` on a blank illuminated reference once per pardcap session (and after objective/camera/illumination change) to store MALref/VARref; then `get_preview_tc ON` sets `CurrTC = (MAL/MALref)·log10(VAR/VARref)` (Y channel, 3×3 median) and, if `tc_threshold` lower ≥0, `NoTissue`/`HasTissue`. `tc_threshold <lower> <upper>`: lower<0 → off; upper<0 → no upper limit; pass if `lower ≤ tc ≤ upper`. `grab_image` is skipped (no file) when the test fails; `save_coords ON` still writes coordinates. If reference unset, TC testing is silently inactive.
- **Autofocus** (Z stepper must exist, be mature and homed): `af_setup <init_z float ≥0> <init_step float >0> <max_retries int ≥0> <dirn_up 0|1> <col_chan 2|3|4 (R/G/B)> <ignore|exit>`, `af_ccrit <max_iter ≥1> <max_diff float ≥0 (0=unlimited)> <min_peak ≥0> <timeout ms (0=none)>`, focus function via `prv_toggle fc_*`, then `autofocus` (hill-climb Z; on success updates best-Z; on failure returns to last best-Z, and `exit` terminates the script). Skipped silently (reports success) when TC thresholding finds no tissue. Positions/steps obey `act_unit` rounding.
- **Preview display:** `prv_size 0|1|2` (VGA/QVGA/QQVGA); `preview_tile 0..15` (QQVGA grid raster; VGA ignores it; QVGA honours only 0,2,8,10); `prv_lut 0..5` (linear, inverted, log10, exp10, sqrt, square); `prv_toggle <opt> <bool>` with opts `mono invert flip_h flip_v laplacian premedian postmedian dctpostlut fc_dct fc_mal fc_vmul mask_show hgm_show hgm_cum_r hgm_cum_g hgm_cum_b hgm_nofit hgm_restrictvar` (`laplacian/premedian/postmedian` only in mono mode); `prv_fc_show 0..4` (off, RGB, R/Y, G, B), `prv_fc_posn <y 20..428>`, `prv_hgm_posn <x 2..382> <y 2..222>`, `prv_hgm_satlim_l <R/Y G B 0..254>` then `prv_hgm_satlim_u <R/Y G B 1..255>` (set lower first; upper must exceed lower), `prv_hgm_scales <f f f ≥0>` (with hgm\_nofit), `prv_m_integral 1..20`, `prv_m_bias <−4845..512>`. **Image-file options use a two-call protocol:** `prv_mask_use <file>` / `prv_cdf_use <file>` (pgm/ppm/bmp; ppm/bmp for colour dark field) only *submit* the file; call again with `ON`/`OFF` to apply (status persists in pardcap). Mono corrections: `prv_mdf_set <file_y.dou>`, `prv_mff_set <file_y.dou>` (raw doubles, preview size), undone only by `prv_mcor_eject`. `df_correction`/`ff_correction`/`corrections_mask <ON|OFF|file>` (full-frame; same two-call protocol; file endings: ff `.pgm/.ppm/.bmp/_y|_i|_r|_g|_b.dou`; df `.dou` forms; mask `.pgm/.ppm/.bmp`).
- **Preview buffer I/O:** `preview_clear`; `preview_show` (always) / `preview_draw_show` (only if overlay active); `preview_file <path>` + `preview_save` / `preview_load [display?]` (P6 ppm; load requires matching preview resolution; saves must set `preview_file` first; offline default `OfflinePreview.ppm`, reset each run); pixel read: `set_previewpos <pos 0..921599> <inc 1..3>` then each evaluation of `PreviewVal` returns `PreviewImg[PreviewPos]` (R,G,B interleaved) and advances `PreviewPos` by `PreviewInc` (wraps); `preview_copy <h 1..480> <w 1..640> <recv_array[idx]> <colchan>` copies from top-left corner `PreviewPos` into an int array (colchan: 0 = all three channel bytes unpacked, one per cell, needs ≥ h·w·3 cells; otherwise one cell per pixel, needs ≥ h·w cells: 1 = intensity Y=(R+G+B)/3 (`CCHAN_Y`), 2 = R (`CCHAN_R`), 3 = G (`CCHAN_G`), 4 = B (`CCHAN_B`) — the same numbering as trackers/autofocus). `cli_path <rootpath>`: in pcs-cli, write the displayed preview (incl. overlay) to `<root>_preview.ppm` on every update (live-view with feh/IrfanView).
- **XY scan (`xyzs_scan`):** prerequisites: all `grab_image` prerequisites; `xyzs_start <x y z ≥0>`, `xyzs_finish <x y ≥0>`, `xyzs_stepsize <sx sy sz each ≥1e-3 even if unused>`, `xyzs_process <prog|NULL> [args with [img]]` (must be issued once; NULL = none), `xyzs_af_period <≥0>` (0 = never; counts only tissue-containing positions when TC is on; needs autofocus prerequisites if ≥1), `xyzs_semino_z <slices ≥0>` (>0 requires successful earlier `af_setup`: Z direction must be known). Serpentine (snake) raster; X, Y, Z mature, homed, camera present. Step size = frame size ⇒ no overlap; smaller ⇒ overlap for stitching. Writes `<root>_xys.xys` (parameters + per-image coordinates; any write error aborts the scan), saves `_preview.ppm` at the end; in CLI mode tiles are stitched into that file atomically as the scan proceeds. Coordinates in act\_unit units, rounded to onestep. Z-stacks (`xyzs_semino_z` > 0): at each XY position Z starts at z0 and first moves down slices×sizeZ (z0 − slices×sizeZ < 0 aborts the scan, out of bounds); it then steps up by sizeZ per slice, capturing at each, until back at z0 (verified); captures the central z0 image (also used for the preview tile and the daemon command); steps up again and captures `slices` more above z0; returns to z0 and moves on. Result: a symmetric stack of **2×slices+1** images per XY position. `xyzs_process` spawns the program each tile (queued per `max_daemons`).

## C2. Graphics sub-interpreter (GSI) — preview overlay

Draws into a separate 640×480 RGB overlay merged over the preview at display time (CLI: burned into the preview buffer). **(0,0,0) is transparent**: darkest paintable colour is (1,1,1). All drawing is clipped silently to the VGA bounds; no antialiasing. Works offline. Defaults at script start: lwidth 1, chamfer 0, angles 0–360, blend off, desttype false (absolute), cursor (0,0).

- Env commands: `preview_draw_colour <R G B 0..255>`; `preview_draw_cursor <X 0..639> <Y 0..479>`; `preview_draw_lwidth <1..16>` (1 = fast path; applies to all but text/bitmaps); `preview_draw_chamfer <0..479>` (rect corner radius, capped at half side); `preview_draw_angles <a1 a2 0..360>` (ellipse arc; 0° = 3 o'clock, clockwise); `preview_draw_blend <bool>` (average with existing pixels); `preview_draw_desttype <bool>` (true: line/rect dest args are offsets from the cursor and may be negative; false: absolute ≥0); `preview_draw_buffer <0..4>` (0 ignore all draw cmds except this; 1 immediate; 2 buffered — shown on `preview_draw_show` or after `get_preview`, buffer kept; 3/4 = 1/2 but buffer cleared after showing).
- Draw commands: `preview_draw_shape <type 0..3> [filled? X Y]` — `0` PIXEL (no more args), `1` LINE (to X,Y), `2` RECT (to X,Y), `3` ELLIPSE (cursor = centre; X,Y = radii); X in −639..639, Y in −479..479 (negatives only with desttype true on line/rect). `preview_draw_string <size 0..63> <varval> [bgR bgG bgB invert?]` (scale factor of the 8×8 font; any type is cast to text; `\n`(char 10) breaks lines; text from cursor downward/rightward; size 0 draws nothing). `preview_draw_file <ppm>` (P6, 1:1 at cursor; black pixels transparent — pre-convert to 1,1,1). `preview_draw_save <file>`, `preview_draw_clear`, `preview_draw_show`, `preview_draw_pcg <char 0..255> <int_array[idx]>` (8 ints, low 8 bits each = row, top to bottom; respects index; font = ASCII 32–126 in their slots, 256 slots total).
- **Batch drawing: `preview_draw_array <int_array>`** executes a pre-built command list (always read **from index 0**, ignoring any index given). Format — repeated records, each starting with a negative type code, ended by `GSI_TYPE_END` (-8; missing end = malformed-array error):

| Code | Record |
| --- | --- |
| `GSI_TYPE_ENV` -1 | then op + args: `3 <px>` lwidth; `4 <px>` chamfer; `5 <a1> <a2>` angles; `6 <tf>` blend; `9 <R> <G> <B>` colour; `10 <X> <Y>` cursor; `17 <tf>` desttype |
| `GSI_TYPE_PLOT` -2 | then `0` pixel; `1 <filled> <X> <Y>` line; `2 <filled> <X> <Y>` rect; `3 <filled> <Xr> <Yr>` ellipse (`PDS_*`) |
| `GSI_TYPE_STRING` -3 | `<size> <bgR> <bgG> <bgB> <invert>` then char codes until the next negative code (all five params mandatory; bg>255 ⇒ background/invert ignored) |
| `GSI_TYPE_MGLYPH` -4 | `<R0 G0 B0 R1 G1 B1 h w>` then rows of packed 1-bit pixels, MSB first, `(w+31)/32` ints per row (0-bits use colour 0, 1-bits colour 1; (0,0,0) transparent) |
| `GSI_TYPE_CGLYPH` -5 | `<h w>` then `h·w` ints `0x00RRGGBB` (0 = transparent) |
| `GSI_TYPE_CBITMP` -6 | `<h w use_alpha>` then `h·w·3` ints R,G,B (same layout as `preview_copy … 0`); use\_alpha 0 ⇒ black becomes (0,0,1) |
| `GSI_TYPE_GBITMP` -7 | `<h w R G B use_alpha>` then `h·w` ints 0..255 (as `preview_copy … >0`) |

Use arrays to pre-compile scenes/fonts/animation frames and emit them in one call; combine with `fourier fvpow` on an 8-bpp array for live spectra.

## C3. Console, input and timing

- **`print_value <varval> [fmt 0..7]`**: fmt bits = debug(4) newline(2) overwrite(1). Default fmt 6 (debug+newline); **a new fmt persists for later calls**; resets to 6 each run. Overprint a line = `fmt 3`. Debug mode prefixes name/type. No trailing newline unless bit 2 set. Prints string variables verbatim; typed numeric literals as numbers. Empty-string print with overwrite set deletes the previous line (repeat for multi-line blocks).
- `print_clear`; `print_colour <P 0..255>` (RGB332 palette) or `<R G B>`; `print_size <1..72>` (default 12, no effect in CLI); `print_cursor <X chars from left> <Y lines from bottom>` (non-latching; applies to next print); `print_graphics <bool>` (square font, zero line gap — for UTF-8 block/box graphics; auto-off at script end; in CLI also hides cursor/echo); `print_file <exclusive?> <path|NULL>` (diverts `print_value` to an appended file; exclusive blocks console/log; one open file; no auto newline; NULL stops); `gui_buffer 0|1|2` (1 = XY-aware off-screen buffer, 2 = continuous-stream buffer (faster, no cursor moves), 0 = flush; can't switch 1↔2 without flushing; unflushed data is discarded at exit); `verbosity 0..3` (0 errors only; 1 + show print\_value; 2 + log it; 3 everything; default 1, reset each run).
- GUI responsiveness: `gui_update_period <n>` (0 never; 1..16 = every 2^n lines; ≥17 every line = default, ≈150× slower local code; Ctrl-C in the launching terminal interrupts an unresponsive script, but kills pardclient if no script is running); `gui_update` forces one refresh; `update_gui_coords <server_sync?>` (needs X,Y,Z defined).
- `poll_user <wait> <return> <var>`: reads the GUI entry box into `var` (then clears it). wait=OFF,return=OFF: immediate peek. OFF,ON: returns immediately; reads only on a later poll after Return was pressed. ON,OFF: blocks until ≥1 char, reads at once. ON,ON: blocks until text + Return. Non-numeric/empty input leaves a numeric var unchanged. With "Divert to script" ticked, GUI direction/focus buttons emit the single characters `<`, `>`, `^`, `v`, `+`, `-`. With `gui_update_period 0` call `gui_update` before polling.
- `sleep <seconds float ≥0>`; `milliduration <start_ms> <recv_int>` (ms since a saved `Millis`); `beep <ms 10..5000> <Hz 50..5000> [amp 0..100]` (non-blocking PC sine, 8-voice, values silently clipped; sleep for the same duration in seconds to block); `randpcg <init?> <recv> [pos?]` (deterministic cross-platform PCG32, same sequence every run; int in INT32 range (or ≥0 with pos?), float in \[0,1); init? re-seeds).

## C4. Files, external programs, strings

- `fset_var <var[idx]> <skip> <file>` reads a file into a variable/array from that index: string var = whitespace-separated tokens (≤1023 chars each; longer is truncated with a warning); float = raw 64-bit doubles; int = raw int32 (with `fbyte_mode ON`: one unsigned byte per int slot). `<skip>` counts data items (or bytes if `fskip_size n>0`). Stops at EOF, error or when the variable is full. **Check `NReads`** (≥0 count; -1 can't open; -2 file shorter than skip; -3 read error; -4 default/unspecified); `NBytes` = bytes consumed (after skip); `NewLines` (string reads only) = line endings seen after the last token. Use `NBytes` + `fskip_size 1` to jump to binary data after a text header.
- `fput_var <var[idx]> <append?> <file>`: string var → raw text (no separators added), float → raw doubles, int → raw int32 (or low byte in `fbyte_mode`); writes to end of the array from the index. **Check `NWrites`** (≥0 items; -1 can't open; -3 I/O error; -4 default); `NBytes` = bytes written. Purely numeric file names must be passed in string variables.
- `execute_system <prog> [args]` (synchronous, no shell; sets `Sysval` = exit code, -1 if failed); `execute_daemon <prog> [args]` (asynchronous; queued; at most `max_daemons` (1..100, default 1) run at once). Args are literal words (variables are NOT evaluated) except `[img]`, replaced by the absolute path of the last saved image (`NoTissue` if TC suppressed it; empty if none). Windows: use Windows paths or a bare name; never `./prog`.
- String tools: `set_str <s> [w1…w24]` (replace with up to 24 literal words joined by single spaces; none given → empty; words are not evaluated), `cat_str <s> [w1…w24]` (append; no space inserted between old text and first word), `add_var s v` (append value of v), `cat_char s <code 1..1114111 excl. 55296–57343>` (accepts `U+1F600` form here only), `set_var s src [charpos]` (charpos≥0: `s` = the single character at that position of string `src` (no-op if beyond end); charpos<0: `s` = text of strlen(src); ignored for numeric src), `eval_char <string> <pos ≥0> <int_recv>` (byte value 0..255 at pos).

## C5. Math, arrays, signal analysis

- Basic: `set_var, add_var, sub_var, mul_var, div_var <var> <varval>` (RHS cast to receptor type; `add_var` on strings appends).
- **`math_var <recv> <func> [varval]`** (`func` a string varval): `add sub mul div =` (exact like the `_var` commands) and C-math on doubles: `sin cos tan asin acos atan atan2 sinh cosh tanh exp log log10 pow sqrt ceil floor abs fmod imod` (`imod` = integer `%`; two-argument funcs use `varval` as C's y; missing needed `varval` is a run-time error, extra is ignored; int receptors round half-away). `norls norl1 norl2` normalise (signed-L1 / L1 / L2) — meaningful on arrays (single value ⇒ ±1). Errors (range, cast) terminate. The checker needs string `func`/numeric string operands initialised validly.
- `math_array <recv_arr[i]> <func> [mod_arr[j]]`: same, element-wise over the common remaining span of both arrays (stops at the shorter); modifier must be an array; string receptors allow only `=`; aliasing OK.
- `cmath_var <re> <im> <op> [mre mim]` / `cmath_array <re[i]> <im[i]> <op> [mre[j] mim[j]]` (floats only; array modifiers must be arrays): ops `= add sub mul div` (need modifier; division via Smith's algorithm), `conj`, `pow` (spectral power), `pol` (→ arg & magnitude), `cycf`/`cycb` (cyclic shift forward/backward in the Fourier sense: DC to centre / back; no-ops for scalars; 1-D only in arrays).
- **`fourier <action> <forward?> <height> <width> <in_re> <in_im> [<out_re> <out_im>]`**: 1-D data ⇒ height=1; transforms need width≥4 and height 1 or ≥4; every array needs ≥ height·width cells from its index; float/int arrays only. Actions: `transform` (in situ FFT, direction by `forward?`), `cyclise` (quadrant shift; forward = DC to centre) → outputs, `power` → outputs, `vpower` (log-scaled 0–255 visual power, assumes forward FT of an 8-bit image) → outputs, `fvpow` (in situ: real 8-bit image in `in_re`, `in_im` all 0 and a *different* array; forward only; result in diffraction order in `in_re`, native order in `in_im`). Aliasing inputs/outputs is allowed (except fvpow's re/im).
- **`stats_array <arr[i]> <mode> <threshold> <args…>`** (int/float arrays; values below `threshold` ignored; for int arrays the threshold applies only if >0 and histogram stats assume 0..255): all receptors are variables (any type, doubles cast). Sizes (`len`,`height`,`width`) are int varvals ≥1 and must fit in the array from the index. Modes: **`basic <len> <mean> <integral> <min> <max> <range> <var> <SD> <energy(RMS)> <sparsity>`**; **`spatial <height> <width/com_c (variable: input width, overwritten by centre-of-mass column)> <com_r> <bb_r> <bb_c> <bb_ht> <bb_wd> <orient (rad)> <eccent 0..1>`**; **`distro <len> <median> <lquart> <uquart> <iqr> <ptile (in/out: percentile 0..100 in, value out; out of range ⇒ 95th)> <mode> <peak> <entropy>`**; **`texture <height> <width> <tv> <gradenergy>`** (width must be a constant/var of size here). No active pixels ⇒ zeros (sparsity 1). At least as many args as the mode needs.

## C6. Trackers (client-side; all work on the 640×480 preview buffer, coords = top-left (row,col) of a square template window)

Workflow: `def_tracker` once → put target image in preview → `init_tracker` → per frame: load frame, `track`, `get_tracker` → `set_tracker`/`match_tracker` to tune/recover → `undef_tracker`.

- `def_tracker <type> <name> <template_dim ≥16> <search_dim (>template; ≤480)> <cell_dim>`: types (lower case) **zmncc** (zero-mean normalised cross-correlation; adaptive template; textured slowly changing targets; occl. thr 0.80), **hog** (9-bin edge-direction histograms; cell\_dim 8/16/32, template a multiple of and ≥2× cell, search ≥ template+cell; fixed template; 0.50), **com** (centroid of bright/dark blob; 0.50), **ght** (translation-only Generalised Hough on strong edges; tolerates partial occlusion; polarity-sensitive; 0.35), **klt** (pyramidal Kanade-Lucas-Tomasi corner features + background compensation; whole frame, `search_dim` ignored but must be valid; several MB; 0.30). zmncc/com/ght need search ≥ template+8. `cell_dim` unused except hog (still must be ≥8). Name: any non-numeric text, case-sensitive; type/name may be string variables. A target can move at most \~(search−template)/2 px between tracks (not klt).
- `init_tracker <name> <row 0..480-T> <col 0..640-T> <colchan 1..4>` (`CCHAN_Y 1, R 2, G 3, B 4`; remembered). Fails softly with `TRKS_FLAT_TEMPLATE` on featureless templates (ght < `MINEDGES` edges; klt < `MINPTS` corners) and leaves the tracker un-initialised.
- `track <name> <row_var int> <col_var int> <score_var float>`: returns best position + confidence; confidence < occlusion threshold ⇒ `TRKS_OCCLUDED`, occlusion counter++ (position still returned); un-seeded/bad window ⇒ score −1.0 and row/col untouched (they never receive invalid values).
- `match_tracker <name> <row> <col> <float_recv>`: score 0..1 of a chosen window vs the reference (does not disturb the tracker; −1.0 + status set on failure; success leaves status unchanged). Use to scan for a lost target, then redirect with `set_tracker … TKSET_ROW/COL` or `init_tracker`. (ght scores clip at 1 — take the middle of the max plateau; klt scores aren't comparable with the threshold.)
- `get_tracker <name> <item 1..11> <recv>`: `TKGET_OCCLCOUNT 1, ERRSTATUS 2, SEEDED 3, ROW 4, COL 5, NPOINTS 6 (klt), NINLIERS 7 (klt), LOSTFRAMES 8 (klt), NREFEDGES 9 (ght), VOTES 10 (ght), IMGEDGES 11 (ght)`.
- Status codes `TRKS_`: `OK 0, UNSEEDED 1, BAD_WINDOW 2, BAD_CHANNEL 3, FLAT_TEMPLATE 4, OCCLUDED 5, BAD_SETTING 6`. The flag is sticky: `init_tracker`/`track` always overwrite it; `match_tracker` only on failure; `set_tracker` sets 6 on rejection. To test one command: `set_tracker t TKSET_STATUS TRKS_OK`, run it, then `get_tracker t TKGET_ERRSTATUS s`.
- Failure policy: tracker problems never terminate the script; only invalid arguments (unknown tracker, out-of-range coordinates, wrong variable type, non-existent setting number) or fatal memory errors do.
- `set_tracker <name> <setting> <value>` (values ≥0; out-of-range or wrong-type-for-tracker ⇒ no change + `TRKS_BAD_SETTING`). General: `TKSET_TEMPDIM 1` (≥16; change un-seeds), `SEARCHDIM 2`, `COLCHAN 3` (change un-seeds), `SEARCHSIG 4` (zmncc/ght position-prior width, float >0; default ¼(search−template)²), `MERGEALPHA 5` (zmncc template adaptation 0..1, default 0.02), `OCCLTHRESH 6` (0..1), `CELLDIM 7`, `OCCLCOUNT 8`, `ROW 9`, `COL 10`, `STATUS 39`. COM: `COM_THRESH 11` (float ≤255; negative = auto), `COM_POLARITY 12` (0 bright, 1 dark), `COM_ITER 13` (1..100, def 10). KLT: `KLT_MAXTGT 14` (1..128, 40), `MAXBG 15` (0..128, 24; 0 = no background compensation), `RING 16` (1..64, 32), `LEVELS 17` (1..8, 4), `HW 18` (2..12, 7), `MAXITER 19` (1..100, 20), `EPS 20` (1e-6..1, 0.03), `MINEIG 21` (0..1e6, 1.0), `FBCHECK 22` (0/1, 1), `FBTHRESH 23` (0.001..100, 1.0), `CONSIST 24` (0.001..100, 2.0), `RELTOL 25` (0..10, 0.15), `FGTOL 26` (0..100, 1.5), `MINPTS 27` (1..128, 3), `MINBG 28` (1..128, 3), `REFILL 29` (0..1, 0.6), `QUALITY 30` (0..1, 0.02), `MINDIST 31` (1..64, 6), `MAXLOST 32` (0..1e6, 30). GHT: `GHT_BINS 33` (8..90, 36), `BINTOL 34` (0..3, 1), `MAGTHR 35` (float ≤1000, neg = auto), `EDGEFRAC 36` (0.01..1, 0.25), `SMOOTH 37` (0/1, 1), `MINEDGES 38` (1..100000, 20). Only `ROW/COL/OCCLCOUNT` can be read back with `get_tracker`.

# PART D — COMMAND INDEX (signatures; semantics in Parts A–C)

Conventions: `bool` = binary-choice varval; `recv` = writable user variable; `[img]` etc. as stated. Every command is one line, one prime command. § = section with details.

## Language, flow, variables

- `exit` — end of code; text after is a free scratchpad (§A1). `null_command [anything]` — no-op; also neutralises a line (used by the linker).
- `def_var <type> <name> <init>` · `def_array <type> <name> <size> <init>` · `undef_var <name>` (§A3).
- `set_var <var> <varval> [charpos]` · `add_var` · `sub_var` · `mul_var` · `div_var <var> <varval>` · `math_var <recv> <func> [v]` · `math_array <arr[i]> <func> [arr[j]]` · `cmath_var` · `cmath_array` · `fourier` · `stats_array` (§C4–C5).
- `set_str <s> [words…]` · `cat_str <s> [words…]` · `cat_char <s> <code>` · `eval_char <string> <pos> <recv>` (§C4).
- `set_condexp <v1> <op> <v2>` (op ∈ `== >= > <= < != && ||`, string varval) · `do_if <bool>` / `else` / `enddo` · `loop_while <bool>` / `endloop` · `goto <label|strvar> [if <bool>]` · `call_func <name|strvar> [if <bool>]` · `def_func <name>` / `return` (§A6–A7).
- `make_lib` · `add_lib <file>` (§A8). Preprocessor: `#define`, `#include` (§A5).
- `sleep <sec>` · `milliduration <start_ms> <recv>` · `randpcg <init?> <recv> [pos?]` · `beep <ms> <Hz> [amp]` · `verbosity <0..3>` (§C3).

## Actuators (§B)

- `get_actuators` — refresh local actuator list and `N*` counts (call first in every hardware script).
- `def_actuator <type 1..5> <index 0..9> <name>` · `undef_actuator <act_id>` · `act_getid <name>` · `act_getname <type> <idx> <strvar>` ("null" if absent).
- `act_set <act_id> <info> <varval> [vy vz]` · `act_get <act_id> <info> <recv> [ry rz]` — NHS/ML\_TTABLE take 3 values; XYZCOORDS read forces ids 0,1,2.
- `act_unit <type> <0|1>` · `act_accel 1 <prop_acc> <isd_min> <isd_max_min>` / `act_accel 4 <s_acc 0..255> <duration_us>` (not for binary; once per login before moves).
- `act_home <id> <1..4>` · `act_moveto <id> <duration_us> <pos> [y [z]]` (stepper) / `act_moveto <servo> <bool>` (fire/wipe queue) · `act_moveby <id> <duration_us> <dist ≥0> [dy [dz]]` (StageXY/StageXYZ need the extra args) · `act_halt <id>` · `act_movereport <int_recv> <type_recv> <id_recv>` (non-blocking PMR poll; fatal on error PMR; sets `PMRep1..4`).
- TMC helpers (local, after `act_get … TMCSTATUS/TMCCHCONF`): `tmc_status_parse <item> <recv>` — `TMC_SI_` OVER\_TEMP 1, PREO\_TEMP 2, SHORTGNDA 3, SHORTGNDB 4, SHORTSUPA 5, SHORTSUPB 6, OPENLOADA 7, OPENLOADB 8, CS\_ACTUAL 9 (0..31), TEMP\_THRS 10 (0..15), STNDSTILL 11, SPRDCYCLE 12 (others 0/1); `tmc_chconf_parse <item> <recv>` — `TMC_CI_` TOFFLEVEL 1 (0..15), HYSTSTART 2 (1..8), HYST\_\_END 3 (−3..12), CPBLANKTM 4 (0..3 = 16/24/36/54 clocks), VSENSITIV 5, MRESMSDEN 6 (1..256), INTERPOLN 7, DOUBLEDGE 8, DISSHRTTG 9, DISSHRTTL 10 (others 0/1).

## Camera, capture, scan (§C1)

- `select_camera <n>` · `get_camsettings` · `get_resolutions` · `camera_get <id> <recv> [50|51|52]` · `camera_set <id> <int>`.
- Capture config: `save_format <yuv|pgm|bm8|int|ppm|bmp|png|jpg>` · `save_doubles <OFF|ON|FITS>` · `save_resolution <w> <h>` · `save_root <name>` · `save_path <dir>` · `save_coords <bool>` · `jpeg_quality <1..100>` (client only) · `frame_number <n>` · `frames_to_average <≥1>` · `average_scale_means <bool>` · `frames_to_skip <≥0>` · `skip_frames <≥0>` (one-off) · `grabber_predelay <0..172800 s>` · `grabber_timeout <0..4096 s>` · `grabber_retries <0..4096>` · `yuyv_gain <float, default 1.0>` · `yuyv_bias <float, default 0.0>` (YUYV→RGB conversion).
- Corrections: `df_correction`, `ff_correction`, `corrections_mask <ON|OFF|file>` (two-call protocol).
- `grab_image` · `tc_threshold <lower> <upper>` · `af_setup …` · `af_ccrit …` · `autofocus` · `xyzs_start <x y z>` · `xyzs_finish <x y>` · `xyzs_stepsize <sx sy sz>` · `xyzs_semino_z <slices>` · `xyzs_af_period <n>` · `xyzs_process <prog|NULL> [args]` · `xyzs_scan`.

## Preview (§C1)

- `get_preview <bool zoom> <cx> <cy> [display?]` · `get_preview_focus` · `get_preview_stats <bool>` · `get_preview_tc <bool>` · `preview_clear` · `preview_show` · `preview_file <path>` · `preview_load [display?]` · `preview_save` · `preview_tile <0..15>` · `preview_copy <h> <w> <recv_arr[i]> <colchan>` · `set_previewpos <pos> <inc>` · `cli_path <root>`.
- `prv_size`, `prv_lut`, `prv_toggle <opt> <bool>`, `prv_fc_show`, `prv_fc_posn`, `prv_hgm_posn`, `prv_hgm_satlim_l`, `prv_hgm_satlim_u`, `prv_hgm_scales`, `prv_mask_use`, `prv_cdf_use`, `prv_mdf_set`, `prv_mff_set`, `prv_mcor_eject`, `prv_m_integral`, `prv_m_bias` (ranges in §C1).

## Overlay graphics (§C2)

`preview_draw_colour|cursor|lwidth|chamfer|angles|blend|desttype|buffer|shape|string|file|save|clear|show|pcg|array`.

## Console / user / system (§C3–C4)

`print_value`, `print_clear`, `print_colour`, `print_size`, `print_cursor`, `print_graphics`, `print_file`, `gui_buffer`, `gui_update`, `gui_update_period`, `update_gui_coords`, `poll_user`, `fset_var`, `fput_var`, `fbyte_mode`, `fskip_size`, `execute_system`, `execute_daemon`, `max_daemons`.

## Trackers (§C6)

`def_tracker`, `init_tracker`, `track`, `get_tracker`, `set_tracker`, `match_tracker`, `undef_tracker`.

# PART E — GOTCHAS CHECKLIST (read before writing code)

1. First line `Pardus pcs 1`; last line `exit`; `#include stdpard.pch` for symbolic names; define `main` if you define any function; all non-main functions after main.
2. No expressions, no quotes, no escapes; one command per line (≤31 words). Build text with `set_str`/`cat_str`/`add_var Space`/`add_var Hash`/`cat_char`.
3. Size-1 variables: no brackets. Size≥2 arrays: always brackets (except `def_array`/`undef_var`). Index must be a simple varval.
4. Checker uses initial values only: initialise sizes ≥1, numeric strings as numbers (`0.0`), label/function/operator variables to valid names.
5. Undefined/mistyped names silently become string literals — spell-check variable names; `undef_var` then reuse = literal.
6. `mul_var`/`add_var` convert the RHS to the receptor's type; use float temporaries for fractional math on ints.
7. `loop_while Condexp` needs `set_condexp` re-run inside the loop; `Condexp` is stored state.
8. Labels not between functions when more than `main` exists; `goto` is non-local and stack-aware; use string arrays of labels/functions/operators for dispatch tables.
9. Hardware: `get_actuators` → define/mature → `act_accel` → home → move; poll `act_movereport` rather than `act_get` during motion; units via `act_unit`; positions ≥ 0 and rounded to onestep.
10. M-Level/servo: set type/band first; assign p0/pin per timer-sharing rules; re-select a different row (or `act_halt`) before repeating an identical level; freq/prescaler is per timer.
11. Preview grabs ignore skip/average/timeouts: `skip_frames 3` first; check `PreviewReport`; check `NReads`/`NWrites` after file I/O.
12. GSI: black is transparent (use 1,1,1); `preview_draw_array` always reads from index 0 and must end with -8.
13. Don't rely on `act_get` of coordinates in calibrated units — reads are atomic; servo reads are µs.
14. Never claim an AI-written hardware script is safe: verify pins, limit switches, soft limits and kill switch before running.
