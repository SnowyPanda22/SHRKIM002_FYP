# SHRKIM002_FYP

The files in this GitHub repository are the MATLAB / Simulink files used and created to assist in writing the Final Year Project report

The track was created using the centerlineTrack.m function which took the track information from SaoPaulo.csv to create a waypoint matrix 

The SHRKIM002_Dubins.slx and SHRKIM002_Splines.slx were used for the trajectory comparison and have identical simulation environments, with only the trajectory generation method differing. 

The controller comparison was done inside the SHRKIM002_Splines.slx environment using a variant subsystem. Click on the variant subsystem to change the active controller.

The Pure Pursuit information is fully inside of the SHRKIM002_Splines.slx file whereas the MPC rely on other resources.

The NMPC is created and setup through the controlNMPC.m file. Once this file is run, it creates a file called nmpc_robot.mat which houses the NMPC model information. 
This nmpc_robot.mat file is called within the SHRKIM002_Splines.slx environment and is called on in the NMPC controller block. If the NMPC doesn't update then try double clicking on the nmpc_robot.mat file to rerun it.

The ALMPC uses the MPC Designer app in MATLAB to create the MPC model. To edit the MPC, run setEstimator(mpc_opt, 'default'); in the command window since the MPC is adaptive and uses custom state estimation instead of the default.
After setting it to default, open MPC Designer, click on mpc_opt and import it to make changes. 
After making changes, export the controller to the workspace, run setEstimator(mpc_opt, 'custom'); in the command window and then navigate to Model Explorer and copy the mpc_opt from the Base Workspace into the Model Workspace.
Save the space and reassign mpc_opt into the adaptive MPC controller block to update the MPC.

The SHRKIM002_Optimisation.slx file uses the base ALMPC and Splines trajectory from SHRKIM002_Splines.slx and optimises it. Do the same process as the base ALMPC to update the controller.

This project builds on Project 1 from MEC4127F as seen in MEC4127F_Project.mlx and the simulation was built on the MEC4127F_Base_sim.slx file.
