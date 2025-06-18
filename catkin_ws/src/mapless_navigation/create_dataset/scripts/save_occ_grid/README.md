# Capture DataSet
## Change scenario
If you need to change the scenario when using "justina_mapless_simul.launch" 
- fist run: 

## Occ Grid Dataset from path

In order to save ooc grid data, follow the instructions:
- launch "justina_mapless_simul.lauch" from pkg="mapless_nav"
- from pkg="create_dataset":
    - run the node "path_markers.py", you can create manually a path to navigate by clicking "Publish Point" on RViz, read "commands.png" to use keyboard commands
    - run te node "justina_occgrid_data_from_path.py", this manages and save the data as .npz files

## Occ Grid Dataset from controller

- launch "justina_mapless_simul.lauch" from pkg="mapless_nav"
- run te node "justina_occgrid_data_controller.py" from pkg="create_dataset", put the objective by clicking "Publish Point" on RViz. Type 's' or 'space' to start/stop recording, type 'q' to exit node

## Occ Grid Dataset using map



# RUN experiments
Report file will be save as "nav_register.csv" in  this folder

## Navigation using MAP

## Mapless navigation using trained models
- launch "justina_mapless_simul_maped.launch" from pkg="mapless_nav"
- run te node "mapless_local_occ_matr_timed.py" from pkg="mapless_nav"
- run te node "results_time_models.py" from pkg="create_dataset"

