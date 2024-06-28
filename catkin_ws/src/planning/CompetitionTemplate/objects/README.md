# Instructions

- Create folders in `known_objects` for each object category. Longer category names in snake case, i.e. `example_categories`
- Create folders in `test_objects` for each test that needs additional objects
- Place image (jpg or png) of objects in respective folder. Longer objects names in snake case, i.e. `example_object.png`
- Dependencies: 
    - `sudo apt install python3`
    - `sudo apt install imagemagick`
    - `sudo apt install pandoc`
- Create markdown file of known objects:
    - `python generate_object_list.py known_objects/ > objects.md`
- Create markdown file of all test objects:
    - `python generate_object_list.py test_objects/ > objects.md`
- Add singular version of object category in brackets to markdown file for command generation
- Or for individual tests if needed:
    - `python generate_object_list.py test_objects/example_test/ > objects.md`
- Create pdf file from markdown file:
    - `pandoc objects.md -o objects.pdf`
- Delete this readme file if you want object markdown file to be displayed in github
