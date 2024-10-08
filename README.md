# Drone Band Simulator

## 3D model of the lab using NeRF studio
### I. Installation:
- You will need to create an environment to run and install nerfstudio.
- Please follow the guid on this link to install nerfstudio that fits to your computer: https://docs.nerf.studio/quickstart/installation.html.
### II. Use nerfstudio on your own data:
1. Take photos of your 3D modeling object, in our case the drone lab. Make sure that you take enough photos to model the object properly.
2. Process the image using the following commad:
   ns-process-data images --data <path_to_your_images_directory> --output-dir <path_to_your_output_directory>
   This command uses colmap to process the images, if there isn't enough images or the quality of the images is bad this process will fail.
3. Train your nerf model using the following command:
   ns-train nerfacto --pipeline.model.predict-normals True --data <pr]ath_to_your_processed_data>
4. Export the 3D model using the following command:
   ns-export poisson --load-config <path_to_the_output_of_the_training_should_be_in_this_format outputs/data/nerfacto/2024-04-27_133234/config.yml> --output-dir <path_to_your_output_directory>
5. Now you can display the 3D model using any mesh data program such as MeshLab - https://www.meshlab.net/#download.

## Lab Simulator

## Credits:
This project inculdes drones simulator that based on the lab simulator which created by Liam Vanunu.
You can find more information on the simulator in the following link : https://github.com/rbdlabhaifa/simulatorMapping.

## Requirements:
- Ubuntu 22.04
- At least 4 cores(If you're working from virtual machine please allocate at least 4 cores to the VM)
- Prefered: 6 cores
## Students Guide

For comprehensive guide about the simulator: 
https://drive.google.com/file/d/1HKZ_ecanpfv_IFJg1f98f6U__ivAlVef/view?usp=sharing

## I. Installation:

### 0. Install the simulator
- `./install.sh`*

* There is a chance that you will encouter some make errors - they happen because you don't give enough resources to the Virtual Machine. Just give more resources(You can find how easily in the internet). When you run again don't run `./install.sh` instead `cd build` and then `make`.

### 1. Use existing data or build new simulator:
This part is only for users of existing simulator and not meant for users who want to create new simulator model.
If you want to use existing simulator(like the one of the TLV University Lab) download this zip file:
https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing

After you downloaded the zip file go to `generalSettings.json` and change the paths described in part 1 as follows:
if the path is in the `simulatorMapping` directory - just change the path to your `simulatorMapping` path.
if the path isn't in the `simulatorMapping` directory - change it to the equivalent file in the all-data file you downloaded

### 2. Configure the parameters to your machine:
Open generalSettings.json and change:
- `slam_configuration/vocabulary_path` to the path of your vocabulary on the project
- `slam_configuration/drone_yaml_path` to the path of your drone config file
- `offline_video_test_path` to the path of the video you want to run of offline orb slam(described later)
- `load_map_path`to the path of the map you created with orb slam
- `simulator_output_dir` to the directory you want the output maps will go to
- `simulator_configuration/model_path` to the path of your `.obj` model


## II. Usage:
### 1. Run simulator:
- run `./exe/runSimulator` from the `build` folder in order to use the simulator within the model
pay attention to the prints:
you will need to press tab from the model viewer in order to start running the slam through the simulator with pre defined moves

### 2. Create orb slam map from webcam/drone:
- run `./exe/mapping` from the `build` folder

### 3. create Orb Slam map from video:
- run `./exe/offline_orb_slam` from the `build` folder

