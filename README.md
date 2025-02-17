# Description of the data and file structure

Files are stored in *.csv*, *.mat*, *.txt*, and *.tdms*  formats.


## *TRUNC/matlab/training* (motion capture)

### */data*

**Data headers:**
* date and time: Time stamp of collection formatted in m:dd:yyyy hh:mm
* Trajectory: Index of sampled trajectory
* Waypoint: Waypoint index in trajectory
* x_end_avg: x position measured in m
* y_end_avg: y position measured in m
* z_end_avg: z position measured in m
* qx_end_avg: quaternion x component
* qy_end_avg: quaternion y component
* qz_end_avg: quaternion z component
* qw_end_avg: quaternion scalar
* l0: delta length from home position of cable 0 in mm
* l1: delta length from home position of cable 1 in mm
* l2: delta length from home position of cable 2 in mm
* l3: delta length from home position of cable 3 in mm
* l4: delta length from home position of cable 4 in mm
* l5: delta length from home position of cable 5 in mm
* l6: delta length from home position of cable 6 in mm
* l7: delta length from home position of cable 7 in mm
* l8: delta length from home position of cable 8 in mm
* l9: delta length from home position of cable 9 in mm


Figure generated using *TRUNC/matlab/workspace_analysis.mat*

* */2024_02_19_21_08_57/positions_norm_full.csv*
    *  Data set used to learn the arm's inverse kinematics and visualize the workspace (Fig. 4D).

### */experiments*

**Data headers:**
* date and time: Time stamp of collection formatted in m:dd:yyyy hh:mm
* p: Waypoint index
* x_end_avg: x position measured in m
* y_end_avg: y position measured in m
* z_end_avg: z position measured in m
* qx_end_avg: quaternion x component
* qy_end_avg: quaternion y component
* qz_end_avg: quaternion z component
* qw_end_avg: quaternion scalar

Figures generated using *TRUNC/matlab/trajectory_analysis.m*

* */circle_2024_02_21_15_16_35/positions.csv*
    *  Circle trajectory test (Fig. S9A)
* */triangle_2024_02_22_15_48_39/positions.csv*
    *  Triangle trajectory test (Fig. S9B) 
* */line_2024_02_22_17_32_07/positions.csv*
    *  Line trajectory test (Fig. S9C) 

### */repeatability*

**Data headers:**
* date and time: Time stamp of collection formatted in m:dd:yyyy hh:mm
* Repeat num: Trial number
* Test num: Data collection index (same as p_idx for trajctory precision test)
* p_idx: Waypoint index
* x_end_avg: x position measured in m
* y_end_avg: y position measured in m
* z_end_avg: z position measured in m
* qx_end_avg: quaternion x component
* qy_end_avg: quaternion y component
* qz_end_avg: quaternion z component
* qw_end_avg: quaternion scalar


Figures generated using *TRUNC/matlab/repeatability_analysis.m*

* */2024_02_06_11_33_51/positions.csv*
    * Point precision test (Fig. 4E)
* */2024_02_06_08_53_05/positions.csv*
    * Trajectory precision test (Fig. 4F)

## *TRUNC/matlab/instron* (mechanical characterization)

### */force-displacement*

**Data headers:**
* Displacement: Linear displacement measured in mm
* Force: Torque measured in N

Figures generated using *TRUNC/matlab/instron_analysis.m*

* */flex-shaft.csv*
    * Printed flex shaft force-displacment (Fig. 3B)
* */simple-force-displacement_1.csv*
    * Force-displacement data for equitorial cell (Fig. 2C)
* */complex-force-displacement_1.csv*
    * Force-displacement data for truss cell (Fig. 2D)
* */simple-printed-force.csv*
    * Force-displacement data for printed equitorial cell (Fig. S5C)

### */bending-rotation*

**Data headers:**
* Time: Elapsed time in seconds
* Torque: Torque measured in N∙mm
* Rotation: Angular displacement measured in degrees

Figures generated using *TRUNC/matlab/instron_analysis.m*

* */simple-bending-rotation_1.csv*
    * Bending-rotation data for equitorial cell (Fig. 2C)
* */complex-bending-rotation_1.csv*
    * Bending-rotation data for truss cell (Fig. 2D)
* */simple-printed-bending.csv*
    * Bending-rotation data for truss cell (Fig. S5C)

### */torsion-rotation*

**Data headers:**
* Time: Elapsed time in seconds
* Torque: Torque measured in N∙mm
* Rotation: Angular displacement measured in degrees

Figures generated using *TRUNC/matlab/instron_analysis.m*

* */simple-torsion-rotation_1.csv*
    * Torsion-rotation data for equitorial cell (Fig. 2C)
* */complex-torsion-rotation_1.csv*
    * Torsion-rotation data for truss cell (Fig. 2D)
* */simple-printed-torsion.csv*
    * Torsion-rotation data for truss cell (Fig. S5B)

### */cross-coupling*

**Data headers:**
* Time: Elapsed time in seconds
* Torque: Torque measured in N∙mm
* Rotation: Angular displacement measured in degrees

Figures generated using *TRUNC/matlab/cross_coupling_analysis.m*

* */trial_1_1.csv*
    * Cross coupling data when inner cell is driven (Fig. S7B)
* */trial_1_2.csv*
    * Cross coupling data when outer cell is driven (Fig. S7C)

### */bellows*

Figures generated using *TRUNC/matlab/bellows_analysis.m*

* */rubber-bellows_1.csv*
    * Torsion-rotation data for rubber bellows (Fig. S6B)
* */steel-bellows_1.csv*
    * Torsion-rotation data for rubber bellows (Fig. S6B)


## *TRUNC/matlab/daq* (kinematic cross coupling)

**Data headers:**
* input_pos: Angular position of output shaft in radians
* output_pos: Angular position of output shaft in radians
* t: Elapsed time in seconds

Figures generated using *TRUNC/matlab/daq/cross_coupling.m*

* */theta_12.mat*
    * Encoder data to show independent DOFs (Fig. 3D)
* */theta_21.mat*
    * Encoder data to show independent DOFs (Fig. 3D)

## *TRUNC/matlab/cv* (constant velocity)

**Data headers:**
* t1: Elapsed time for input measurement in seconds
* inp: Angular position of output shaft in radians
* t2: Elapsed time for output measurement in seconds
* outp: Angular position of output shaft in radians

Figures generated using *TRUNC/matlab/plot_cvjoint.m*

Bending constant velocity (Fig. S2A) 
* */bend/zerozero.txt*
    * $\beta = 0^\circ$
* */bend/fivedeg.txt*
    * $\beta = 5^\circ$
* */bend/tendeg.txt*
    * $\beta = 10^\circ$
* */bend/fifdeg.txt*
    * $\beta = 15^\circ$
* */bend/twenfdeg.txt*
    * $\beta = 20^\circ$

Extending constant velocity (Fig. S2B) 
* */extend/minus13mm.txt*
    * $\Delta L = -13$ (mm)
* */extend/minus6p5mm.txt*
    * $\Delta L = -6.5$ (mm)
* */extend/plus6p5mm.txt*
    * $\Delta L = 6.5$ (mm)
* */extend/plus13mm.txt*
    * $\Delta L = 13$ (mm)
* */extend/plus22p5mm.txt*
    * $\Delta L = 22.5$ (mm)

## *TRUNC/matlab/effciency* (efficiency testing)

**Data headers:**
* Torque/N.m: Torque measurement in N∙m
* Speed/RPM: Angular velocity in RPM
* Power/W: Power measurement in W

Figures generated using *TRUNC/matlab/efficiency_analysis.m*

Rubber bellows data (Fig. S6)
* */rubber*
    * */input*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*
    * */output*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*

Steel bellows data (Fig. S6)

* */steel*
    * */input*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*
    * */output*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*

TRUNC data (Fig. S6)

* */trunc*
    * */input*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*
    * */output*
        * */0deg.tdms*
        * */5deg.tdms*
        * */10deg.tdms*
        * */15deg.tdms*
        * */20deg.tdms*
        * */25deg.tdms*
        * */30deg.tdms*
        * */35deg.tdms*
        * */40deg.tdms*
        * */45deg.tdms*
        
## *TRUNC/matlab/ansys* (ansys data)

**Data headers:**
* Name: DX where X is diameter in (mm)
* Material type: Assigned link material in ansys
* Bend: Resulting bend deflection in degrees
* Twist: Resulting twist deflection in degrees
* Bend Moment (Nmm): Applied bending moment in N∙mm
* Twist Moment (Nmm): Applied twisting moment in N∙mm

Figures generated using *TRUNC/matlab/ansys_analysis.m

Equitorial cell (Fig. S3A)

* */equitorial*
    * */D28_sim_results.csv*
    * */D42_sim_results.csv*
    * */D56_sim_results.csv*
    * */D70_sim_results.csv*
    * */D84_sim_results.csv*
    * */D98_sim_results.csv*
    * */D112_sim_results.csv*

Truss cell (Fig. S3B)

* */truss*
    * */D28_sim_results.csv*
    * */D42_sim_results.csv*
    * */D56_sim_results.csv*
    * */D70_sim_results.csv*
    * */D84_sim_results.csv*
    * */D98_sim_results.csv*
    * */D112_sim_results.csv*

    


