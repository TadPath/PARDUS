***Interplanetary Marauders (PCS Example)**

***A fully functional, arcade-style "Space Invaders" clone written entirely in PCS (Pardus Command Script). This project demonstrates the Turing-completeness of PCS and its ability to handle real-time graphical rendering, user input, and complex game logic within a terminal environment.**

🚀 ***Features**

- ***Aether-Grid Engine: Implements a coordinate-addressable graphical plane by pre-initializing the terminal buffer.**

- ***Dynamic Fleet Logic: Two rows of Marauders with collective descent and wall-bounce mechanics.**

- ***V-Bomb System: Enemy counter-attacks using an array-based projectile manager.**

- ***Script-Level PRNG: A custom Linear Congruential Generator (LCG) built from atomic math primitives to handle enemy firing probability.**

- ***Self-Erasing Sprites: Optimized "flicker-free" animation using leading/trailing spaces for lateral erasure.**

- ***Interactive HUD: Real-time score tracking and state-driven splash screens.**

🛠 ***Technical Implementation**

***This script adheres to the core PCS Version 1 specifications:**

- ***Atomic Operations: All mathematical and logical steps are performed as single-line instructions.**

- ***Global State Management: Uses unique variable prefixes (e.g., `fleet\_bounce`, `M\_Active`) to manage state across function boundaries.**

- ***Non-Latching Cursor: Handles the PCS default behavior of resetting the cursor to `0,0` after every print operation.**

- ***String Construction: Builds complex Unicode and whitespace-heavy strings using `cat\_char` and `add\_var` to bypass delimiter constraints.**

🕹 ***How to Play**

1. ***Run the script in the Pardus Interpreter.**

2. ***Click the keyboard input box to focus.**

3. ***Press \[S\] to start the game.**

4. ***Use \[,\] (Comma) to move Left and \[.\] (Period) to move Right.**

5. ***Press \[Space\] to fire your turret.**

6. ***Goal: Neutralise all marauders before they reach the turret line. Avoid the 'v' bombs!**


***Developed as an AI-Friendly coding experiment in the PARDUS ecosystem.***


  

