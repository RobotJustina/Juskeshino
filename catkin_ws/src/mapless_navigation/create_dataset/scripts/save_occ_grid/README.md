## Occ Grid Dataset from path

In order to save ooc grid data, follow the instructions:
- launch "justina_mapless_simul.lauch" from pkg="mapless_nav", use the arg name="world" to select the scenario
- from pkg="create_dataset":
    - run te node "path_markers.py", so you can manually create paths to navigate by clicking with "Publish Point" on RViz, read "commands.png" to use it correctly
    - run te node "justina_occgrid_data_from_path.py", this allow to save the data as .npz files

## Occ Grid Dataset from controller

- launch "justina_mapless_simul.lauch" from pkg="mapless_nav", use the arg name="world" to select the scenario
- run te node "justina_occgrid_data_controller.py" from pkg="create_dataset", put the objective by clicking with "Publish Point" on RViz, type 's' or 'space' to start/stop recording, type 'q' to exit node