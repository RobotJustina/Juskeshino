## Occ Grid Dataset

In order to save ooc grid data, follow the instructions:
- launch "justina_mapless.lauch" from pkg="mapless_nav", use the arg name="world" to select the scenario
- run te node "path_markers.py", so you can manually create paths to navigate by clicking with "Publish Point" on RViz, read "commands.png" to use it correctly
- run te node "justina_occgrid_data_from_path.py", this allow to save the data as .npz files
