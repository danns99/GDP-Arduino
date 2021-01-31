# GDP-Arduino
This is a repository for all of the Arduino code for GDP Group 52 - Variable Stability Demonstrator

## Control Model Testing
The Control_Model_Testing folder contains the code for implementing and testing the control model in C.

### Source
#### Code
`source` contains all the C source code. The source code is organised into the following subfolders:
- `control`
  - PID controller
  - Main control loop
- `sim`
  - Initilisation of the simulation
  - Starting the simulation
  - Running the simulation
- `state_space_equations`
  - Numerical ODE solvers
  - State-space equations
- `util`
  - Input/Output to external files
  - Dynamic memory management

#### Input/Output files
The simulation takes as input 3 files:
- `sim_settings.txt`
  
  The file is of the format:
  ```
  Simulation time (seconds): <int>
  Simulation timestep (seconds): <double>
  Time to put in input (seconds): <double>

  Directory to aircraft state space matrices (relative or absolute): <string>
  ```
- Aircraft data files:
  - `scout_state_matrices.txt`
  - `target_state_matrices.txt`
  
  These are of the format:
  
  <img src="https://latex.codecogs.com/svg.latex?\begin{matrix}\mathring{X_u}%20&%20\mathring{X_w}%20&%200%20&%20-mg\cos(\gamma_0)%20\\\mathring{Z_u}%20&%20\mathring{Z_w}%20&%20\mathring{Z_q}+mU_\infty%20&%20-mg\sin(\gamma_0)%20\\\mathring{M_u}%20&\mathring{M_w}%20&%20\mathring{M_q}%20&%200%20\\0%20&%200%20&%201%20&%200%20\\\\u%20&%20w%20&%20q%20&%20\theta\end{matrix}" title="state_matrices" /> 
  
  Note the full version of the state-space matrices must always be provided.
  The aircraft data files should be placed in the same folder.
  The directory to this folder needs to be included in `sim_settings.txt`.

The simulation outputs the results to `data.txt`.
The format of the output is:
```
time scout_data target_data modified_scout_data
```
where `x_data` is 4 values:
```
horizontal_velocity _vertical_velocity pitch_rate pitch_angle
```
 
### Building executables
At each level in the directory of the project there is a `CMakeLists.txt` file which is used for building the executable.
`CMakeLists.txt` in `source` contains the instructions for building an executable.
The preprocessor definition `STATE_SPACE_MATRIX_SIZE` is used for determining what system of state-space matrices are used when building and running a simulation target:
- `4`: Full state-space longitudinal equations are used
- `2`: The Short Period Oscillation (SPO) approximation of the state-space matrices is used.

The SPO approximation reduces the full longitudinal model state matrices to:

<img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}\mathring{Z_w}%20&%20\mathring{Z_q}+mU_\infty%20\\\mathring{M_w}%20&%20\mathring{M_q}%20\\\end{bmatrix}\begin{bmatrix}w%20\\q\end{bmatrix}" title="SPO_matrices" />
