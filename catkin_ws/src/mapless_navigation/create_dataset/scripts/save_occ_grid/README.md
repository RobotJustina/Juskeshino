# Capture DataSet
## Change scenario
If you need to change the scenario when are using "justina_mapless_simul.launch" 
- FIRST change world arg value="rnd_room_c2_01_obst" to new map name in "justina_mapless_save_from_map.launch" file
- launch: "justina_mapless_save_from_map.launch" from pkg="create_dataset"
- CLOSE the launch
- launch "justina_mapless_simul.launch". The new map will be loaded

## Occ Grid Dataset from path

In order to save ooc grid data, follow the instructions:
- launch "justina_mapless_simul.lauch" from pkg="mapless_nav"
- from pkg="create_dataset":
    - run the node "path_markers.py", you can create manually a path to navigate by clicking "Publish Point" on RViz, read "commands.png" to use keyboard commands
    - run the node "justina_occgrid_data_from_path.py", this manages and save the data as .npz files

## Occ Grid Dataset from controller

- launch "justina_mapless_simul.lauch" from pkg="mapless_nav"
- run the node "justina_occgrid_data_controller.py" from pkg="create_dataset", put the objective by clicking "Publish Point" on RViz. Type 's' or 'space' to start/stop recording, type 'q' to exit node

## Occ Grid Dataset using map



# RUN experiments
Report file will be save as "nav_register.csv" in  this folder

## Navigation using MAP
- launch "justina_nav_from_map.launch" from pkg="create_dataset"
- run the node "results_time_map.py" from pkg="create_dataset"

 Change scenario
- Manually change world value from "static_map_file" and "prohibition_map_file" in "justina_nav_from_map.launch"


## Mapless navigation using trained models
- launch "justina_mapless_simul_maped.launch" from pkg="mapless_nav"
- run the node "mapless_local_occ_matr_timed.py" from pkg="mapless_nav"
- run the node "results_time_reactive.py" from pkg="create_dataset"

## Mapless navigation using State Machine
- launch "justina_mapless_simul_maped.launch" from pkg="mapless_nav"
- run the node "state_machine.py" from pkg="mapless_nav"
- run the node "results_time_reactive.py" from pkg="create_dataset"

 Change scenario
- FIRST manually change world value from "static_map_file" and "prohibition_map_file" in "justina_mapless_save_from_map.launch"
- launch "justina_mapless_save_from_map.launch" from pkg="create_dataset"
- run the node "results_time_map.py" from pkg="create_dataset"
- close all
- relaunch "justina_mapless_simul_maped.launch". The new map will be loaded