#!/usr/bin/env python
# -*- coding: utf-8 -*-

# python impliment library
from gpsr_modules.general_states import MyObject

# class MyObject:
#    def __init__(self, name='', synonyms=[], info = '', location = '', possetion = '', num = 1, pose = []):
#        self.name = name
#        self.synonyms = synonyms
#        self.info = info
#        self.location = location
#        self.possession = possession
#        self.num = num
#        self.pose = pose
   
#    def __getattr__(self, name):


#( room (type Room) (name nil) (room nil) (zone nil) (center 0 0 0) (num 1) )
def load_room_info(location="stream"):
    
    if location == "stream":
        room_info = [
            ##DEFAULT LOCATIONS
            MyObject(name='hallway', synonyms=['hallway', 'hallway'], location='hallway', pose=[0.80, 0.20, 0.00, 0.00]),
            
            MyObject(name='living_room_unknown', synonyms=['living_room_unknown', 'living room', 'living_room', 'living room unknown'], location='living_room', pose=[2.30, 0.56, 0.00, 0.00]),    
            MyObject(name='office_unknown', synonyms=['office_unknown', 'office', 'office unknown'], location='office', pose=[2.30, 0.56, 0.00, 0.00]),
            MyObject(name='studio_unknown', synonyms=['studio_unknown', 'studio', 'studio unknown'], location='studio', pose=[2.30, 0.56, 0.00, 0.00]),
            
            MyObject(name='dining_room_unknown', synonyms=['dining_room_unknown', 'dining room', 'dining room unknown'], location='dining_room', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='kitchen_unknown', synonyms=['kitchen_unknown', 'kitchen', 'kitchen unknown'], location='kitchen', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='bedroom_unknown', synonyms=['bedroom_unknown', 'bedroom', 'bedroom unknown'], location='bedroom', pose=[3.00, 3.60, 0.00, 3.14]),
            
            
            #TIDY UP LOCATIONS
            MyObject(name='front_entrance', synonyms=['front_entrance', 'front entrance'], location='living_room', pose=[0.80, 0.00, 0.00, 0.00]),
            MyObject(name='room_center', synonyms=['room_center', 'room center'], location='living_room', pose=[3.75, 0.10, 0.00, 0.00]),
            
            MyObject(name='drawer_bottom_close', synonyms=['drawer_bottom_close', 'drawer bottom close'], location='living_room', pose=[1.39, 0.09, 0.35, -1.54]),
            MyObject(name='drawer_bottom_open', synonyms=['drawer_bottom_open', 'drawer bottom open'], location='living_room', pose=[1.35, 0.32, 0.35, -1.54]),
            MyObject(name='drawer_top_close', synonyms=['drawer_top_close', 'drawer top close'], location='living_room', pose=[1.37, 0.05, 0.55, -1.54]),
            MyObject(name='drawer_top_open', synonyms=['drawer_top_open', 'drawer top open'], location='living_room', pose=[1.35, 0.27, 0.55, -1.54]),
            MyObject(name='drawer_left_close', synonyms=['drawer_left_close', 'drawer left close'], location='living_room', pose=[1.67, 0.03, 0.35, -1.53]),
            MyObject(name='drawer_left_open', synonyms=['drawer_left_open', 'drawer left open'], location='living_room', pose=[1.67, 0.26, 0.35, -1.53]),
            
            MyObject(name='tall_table', synonyms=['tall_table', 'tall table'], location='living_room', pose=[1.03, 1.24, 0.60, 1.59]),
            MyObject(name='large_table_left', synonyms=['large_table_left', 'large table left'], location='living_room', pose=[1.86, 1.32, 0.40, 1.59]),
            MyObject(name='large_table_center', synonyms=['large_table_center', 'large table center'], location='living_room', pose=[2.23, 1.32, 0.40, 1.59]),
            MyObject(name='large_table_right', synonyms=['large_table_right', 'large table right'], location='living_room', pose=[2.59, 1.33, 0.40, 1.59]),

            
            MyObject(name='container_1', synonyms=['container_1', 'container 1', 'container one'], location='living_room', pose=[2.13, 0.00, 0.51, -1.57]),
            MyObject(name='container_2', synonyms=['container_2', 'container 2', 'container two'], location='living_room', pose=[2.38, -0.02, 0.52, -1.57]),
            
            MyObject(name='tray_1', synonyms=['tray_1', 'tray 1', 'tray one'], location='living_room', pose=[2.89, 0.05, 0.40, -1.57]),
            MyObject(name='tray_2', synonyms=['tray_2', 'tray 2', 'tray two'], location='living_room', pose=[2.99, 0.04, 0.40, -1.57]),
            
            MyObject(name='basket_1', synonyms=['basket_1', 'basket 1', 'basket one'], location='living_room', pose=[3.57, 0.00, 0.31, -1.57]),
            MyObject(name='basket_2', synonyms=['basket_2', 'basket 2', 'basket two'], location='living_room', pose=[4.09, -0.05, 0.31, -1.57]),
            
            MyObject(name='living_room_entrance', synonyms=['living_room_entrance', 'living room entrance'], location='living_room', pose=[3.81, 2.12, 0.00, 1.58]),
            MyObject(name='living_room_center', synonyms=['living_room_center', 'living room center'], location='living_room', pose=[2.76, 3.76, 0.00, 3.14]),
            MyObject(name='living_room_delivery', synonyms=['living_room_delivery', 'living room delivery'], location='living_room', pose=[2.76, 3.76, 0.00, 3.14]),
            MyObject(name='living_room_floor', synonyms=['living_room_floor', 'living room floor'], location='living_room', pose=[2.76, 3.76, 0.00, 3.14]),
            
            MyObject(name='shelf_top', synonyms=['shelf_top', 'shelf top'], location='dining_room', pose=[3.21, 3.95, 1.065, 1.56]),
            MyObject(name='shelf_middle', synonyms=['shelf_middle', 'shelf middle'], location='dining_room', pose=[3.21, 3.95, 0.77, 1.56]),
            MyObject(name='shelf_middle_left', synonyms=['shelf_middle_left', 'shelf middle left'], location='dining_room', pose=[3.01, 3.95, 0.77, 1.56]),
            MyObject(name='shelf_middle_right', synonyms=['shelf_middle_right', 'shelf middle right'], location='dining_room', pose=[3.41, 3.95, 0.77, 1.56]),
            MyObject(name='shelf_bottom', synonyms=['shelf_bottom', 'shelf bottom'], location='dining_room', pose=[3.21, 3.95, 0.45, 1.56]),
            MyObject(name='shelf_bottom_left', synonyms=['shelf_bottom_left', 'shelf bottom left'], location='dining_room', pose=[3.01, 3.95, 0.45, 1.56]),
            MyObject(name='shelf_bottom_right', synonyms=['shelf_bottom_right', 'shelf bottom right'], location='dining_room', pose=[3.41, 3.95, 0.45, 1.56]),
            
            #GPSR LOCATIONS
            MyObject(name='large_table_a', synonyms=['large_table_a', 'large table a', 'long table a'], location='living_room', pose=[2.89, 0.05, 0.4, 1.57]),
            MyObject(name='large_table_b', synonyms=['large_table_b', 'large table b', 'long table b'], location='living_room', pose=[2.23, 1.32, 0.4, 1.57]),
            MyObject(name='drawer', synonyms=['drawer', 'drawer'], location='living_room', pose=[1.35, 0.27, 0.55, -1.54]),
            MyObject(name='shelf', synonyms=['shelf', 'shelf'], location='dining_room', pose=[3.21, 3.95, 0.77, 1.57]),
            
            MyObject(name='chair_a', synonyms=['chair_a', 'chair left', 'chair a'], location='dining_room', pose=[1.57, 3.20, 0.40, -3.14]),
            MyObject(name='chair_b', synonyms=['chair_b', 'chair right', 'chair b'], location='dining_room', pose=[1.57, 4.20, 0.40, -3.14]),
            
            MyObject(name='dining_room_entrance', synonyms=['dining_room_entrance', 'dining room entrance'], location='dining_room', pose=[3.81, 2.12, 0.00, 1.58]),
            MyObject(name='dining_room_center', synonyms=['dining_room_center', 'dining room center'], location='dining_room', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='dining_room_delivery', synonyms=['dining_room_delivery', 'dining room delivery'], location='dining_room', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='dining_room_floor', synonyms=['dining_room_floor', 'dining room floor'], location='dining_room', pose=[3.00, 3.60, 0.00, 3.14]),
            
            MyObject(name='kitchen_entrance', synonyms=['kitchen_entrance', 'kitchen entrance'], location='kitchen', pose=[3.81, 2.12, 0.00, 1.58]),
            MyObject(name='kitchen_center', synonyms=['kitchen_center', 'kitchen center'], location='kitchen', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='kitchen_delivery', synonyms=['kitchen_delivery', 'kitchen delivery'], location='kitchen', pose=[3.00, 3.60, 0.00, 3.14]),
            MyObject(name='kitchen_floor', synonyms=['kitchen_floor', 'kitchen floor'], location='kitchen', pose=[3.00, 3.60, 0.00, 3.14]),
            
            MyObject(name='front_door', synonyms=['front_door', 'front door'], location='living_room', pose=[0.0, 0.0, 0.0, 0.0]),
            MyObject(name='kitchen_door', synonyms=['kitchen_door', 'kitchen door'], location='kitchen', pose=[3.81, 2.12, 0.0, 1.58])
        ]
    elif location == "robocup":
        room_info = [
            MyObject(name='entrance', synonyms=['entrance', 'entrance', 'front entrance', 'main entrance'], location='living_room', pose=[0.98, 3.99, 0.00, 0.00]),
            MyObject(name='sofa', synonyms=['sofa', 'sofa'], location='living_room', pose=[2.32, 3.72, 0.40, 1.57]),
            MyObject(name='tv_stand', synonyms=['tv_stand', 'tv stand', 'tv'], location='living_room', pose=[1.62, 3.02, 0.37, -1.57]),
            MyObject(name='book_shelf', synonyms=['book_shelf', 'book shelf', 'bookshelf', 'book'], location='living_room', pose=[2.28, 3.69, 0.10, 2.44]),
            MyObject(name='storage_rack', synonyms=['storage_rack', 'storage rack'], location='living_room', pose=[2.83, 2.78, 0.40, 0.0]),
            MyObject(name='lamp', synonyms=['lamp', 'lamp'], location='living_room', pose=[3.60, 4.28, 0.00, 1.0]),
            MyObject(name='side_table', synonyms=['side_table', 'side table', 'side tables'], location='living_room', pose=[2.32, 3.72, 0.45, 1.57]),
            MyObject(name='living_room_unknown', synonyms=['living_room_unknown', 'living room unknown', 'living room'], location='living_room', pose=[2.88, 1.80, 0.0, 1.57]),
            
            MyObject(name='trash_bin', synonyms=['trash_bin', 'trash bin', 'trashbin'], location='kitchen', pose=[7.36, 2.25, 0.35, -0.60]),
            MyObject(name='pantry', synonyms=['pantry', 'pantry'], location='kitchen', pose=[6.66, 4.32, 0.43, 1.57]),
            MyObject(name='dish_washer', synonyms=['dish_washer', 'dish washer', 'dishwasher'], location='kitchen', pose=[7.36, 2.25, 0.35, 0.0]),
            MyObject(name='sink', synonyms=['sink', 'sink'], location='kitchen', pose=[7.01, 2.11, 0.0, 3.14]),
            MyObject(name='refrigerator', synonyms=['refrigerator', 'refrigerator'], location='kitchen', pose=[7.01, 2.11, 0.0, 3.14]),
            MyObject(name='chair', synonyms=['chair', 'chair', 'chairs'], location='kitchen', pose=[6.53, 3.75, 0.0, 0.0]),
            MyObject(name='potted_plant', synonyms=['potted_plant', 'potted plant'], location='kitchen', pose=[7.61, 5.10, 0.0, 0.72]),
            MyObject(name='kitchen_table', synonyms=['kitchen_table', 'kitchen table'], location='kitchen', pose=[6.53, 3.75, 0.74, 0.0]),
            MyObject(name='kitchen_unknown', synonyms=['kitchen_unknown', 'kitchen unknown', 'kitchen'], location='kitchen', pose=[5.80, 3.73, 0.0, 0.0]),
            
            MyObject(name='shelf', synonyms=['shelf', 'shelf'], location='bedroom', pose=[5.68, -1.46, 0.45, -1.57]),
            MyObject(name='bed', synonyms=['bed', 'bed'], location='bedroom', pose=[8.0, -0.5, 0.42, -1.57]),
            MyObject(name='beside_table', synonyms=['beside_table', 'beside table'], location='bedroom', pose=[8.0, -0.5, 0.57, -1.57]),
            MyObject(name='bedroom_unknown', synonyms=['bedroom_unknown', 'bedroom unknown', 'bedroom'], location='bed', pose=[6.88, 0.08, 0.00, -1.57]),
            
            MyObject(name='coat_rack', synonyms=['coat_rack', 'coat rack', 'coatrack'], location='study', pose=[1.57, -0.13, 0.0, -1.90]),
            MyObject(name='desk', synonyms=['desk', 'desk'], location='study', pose=[1.20, 0.40, 0.80, -0.82]),
            MyObject(name='desk_lamp', synonyms=['desk_lamp', 'desk lamp'], location='study', pose=[1.20, 0.40, 0.0, -0.82]),
            MyObject(name='cabinet', synonyms=['cabinet', 'cabinet'], location='study', pose=[2.88, 1.80, 0.66, 1.57]),
            MyObject(name='waste_basket', synonyms=['waste_basket', 'waste basket'], location='study', pose=[1.20, 0.40, 0.0, -0.82]),
            MyObject(name='armchair', synonyms=['armchair', 'armchair'], location='study', pose=[1.20, 0.40, 0.0, -0.82]),
            MyObject(name='main_exit', synonyms=['main_exit', 'main exit', 'exit'], location='study', pose=[1.06, -0.13, 0.0, 3.14]),
            MyObject(name='study_unknown', synonyms=['study_unknown', 'study unknown', 'study'], location='study', pose=[1.20, 0.40, 0.0, -0.82]),
            
            MyObject(name='corridor', synonyms=['corridor', 'corridor'], location='corridor', pose=[1.20, 0.40, 0.0, -0.82]),
            MyObject(name='safe_point', synonyms=['safe_point', 'safe_point', 'safe point'], location='living_room', pose=[0.8, 4.10, 0.0, 0.0]),
            MyObject(name='instruction_point', synonyms=['instruction_point', 'instruction_point', 'instruction point'], location='living_room', pose=[1.78, 4.10, 0.0, 0.0])
        ]
    else:
        room_info = [
            MyObject(name='front_entrance', synonyms=['front_entrance', 'front_entrance', 'front entrance'], location='living_room', pose=[1.00, 0.20, 0.00, 0.00])
        ]
    
    return room_info



#( item (type Objects) (name nil) (room nil) (zone nil) (attributes nil) (lower base) (upper nothing) (possession nobody) (num 1) )
def load_objects_info(location="stream"):
    
    if location == "stream":
        objects_info = [
            #DEMO
            MyObject(name='001_chips_can', synonyms=['001_chips_can', 'chips', 'chips can'], info='food items', location='large_table_center', num=1),
            MyObject(name='011_banana', synonyms=['011_banana', 'banana'], info='food items', location='shelf', num=1),
            MyObject(name='013_apple', synonyms=['013_apple', 'apple'], info='food items', location='large_table_center', num=1),
            MyObject(name='014_lemon', synonyms=['014_lemon', 'lemon'], info='food items', location='shelf', num=1),
            MyObject(name='015_peach', synonyms=['015_peach', 'peach'], info='food items', location='shelf', num=1),
            MyObject(name='016_pear', synonyms=['016_pear', 'pear'], info='food items', location='shelf', num=1),
            MyObject(name='017_orange', synonyms=['017_orange', 'orange'], info='food items', location='large_table_center', num=1),
            ##
            
            MyObject(name='002_master_chef_can', synonyms=['002_master_chef_can', 'master', 'master chef', 'master chef can'], info='food items', location='tray_1', num=1),
            MyObject(name='003_cracker_box', synonyms=['003_cracker_box', 'crackers', 'cracker box'], info='food items', location='tray_1', num=1),
            MyObject(name='004_sugar_box', synonyms=['004_sugar_box', 'sugar', 'sugar box'], info='food items', location='tray_1', num=1),
            MyObject(name='005_tomato_soup_can', synonyms=['005_tomato_soup_can', 'tomato', 'tomato soup', 'tomato soup can'], info='food items', location='tray_1', num=1),
            MyObject(name='006_mustard_bottle', synonyms=['006_mustard_bottle', 'mustard', 'mustard bottle'], info='food items', location='tray_1', num=1),
            MyObject(name='007_tuna_fish_can', synonyms=['007_tuna_fish_can', 'tuna', 'tuna fish', 'tuna fish can'], info='food items', location='tray_1', num=1),
            MyObject(name='008_pudding_box', synonyms=['008_pudding_box', 'pudding', 'pudding box'], info='food items', location='tray_1', num=1),
            MyObject(name='009_gelatin_box', synonyms=['009_gelatin_box', 'gelatine', 'gelatine box'], info='food items', location='tray_1', num=1),
            MyObject(name='010_potted_meat_can', synonyms=['010_potted_meat_can', 'potted meat', 'potted meat can'], info='food items', location='tray_2', num=1),
            MyObject(name='012_strawberry', synonyms=['012_strawberry', 'strawberry'], info='food items', location='tray_2', num=1),
            MyObject(name='018_plum', synonyms=['018_plum', 'plum'], info='food items', location='tray_2', num=1),
            
            MyObject(name='035_power_drill', synonyms=['035_power_drill', 'power drill'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='036_wood_block', synonyms=['036_wood_block', 'wood block'], info='tool items', location='drawer_top_open', num=1), 
            MyObject(name='037_scissors', synonyms=['037_scissors', 'scissors'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='038_padlock', synonyms=['038_padlock', 'padlock'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='039_key', synonyms=['039_key', 'key'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='040_large_marker', synonyms=['040_large_marker', 'large marker'], info='tool items', location='container_2', num=1),
            MyObject(name='041_small_marker', synonyms=['041_small_marker', 'small marker'], info='tool items', location='container_2', num=1),
            MyObject(name='042_adjustable_wrench', synonyms=['042_adjustable_wrench', 'adjustable wrench'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='043_phillips_screwdriver', synonyms=['043_phillips_screwdriver', 'phillips screwdriver'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='044_flat_screwdriver', synonyms=['044_flat_screwdriver', 'flat screwdriver'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='047_plastic_nut', synonyms=['047_plastic_nut', 'plastic nut'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='048_hammer', synonyms=['048_hammer', 'hammer'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='049_small_clamp', synonyms=['049_small_clamp', 'small clamp'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='050_medium_clamp', synonyms=['050_medium_clamp', 'medium clamp'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='051_large_clamp', synonyms=['051_large_clamp', 'large clamp'], info='tool items', location='drawer_top_open', num=1),
            MyObject(name='052_extra_large_clamp', synonyms=['052_extra_large_clamp', 'extra large clamp'], info='tool items', location='drawer_top_open', num=1),
            
            MyObject(name='036_wood_blocks', synonyms=['036_wood_blocks', 'wood blocks'], info='task items', location='basket_1', num=1),    
            MyObject(name='070-a_colored_wood_blocks', synonyms=['070-a_colored_wood_blocks', 'a colored wood blocks'], info='task items', location='basket_1', num=1),
            MyObject(name='070-b_colored_wood_blocks', synonyms=['070-b_colored_wood_blocks', 'b colored wood blocks'], info='task items', location='basket_1', num=1),
            MyObject(name='071_nine_hole_peg_test', synonyms=['071_nine_hole_peg_test', '071 nine hole peg test'], info='task items', location='basket_1', num=1),
            MyObject(name='072-a_toy_airplane', synonyms=['071_nine_hole_peg_test', 'nine hole peg test'], info='task items', location='basket_1', num=1),
            MyObject(name='072-b_toy_airplane', synonyms=['072-b_toy_airplane', 'b toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-c_toy_airplane', synonyms=['072-c_toy_airplane', 'c toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-d_toy_airplane', synonyms=['072-d_toy_airplane', 'd toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-e_toy_airplane', synonyms=['072-e_toy_airplane', 'e toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-f_toy_airplane', synonyms=['072-f_toy_airplane', 'f toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-g_toy_airplane', synonyms=['072-g_toy_airplane', 'g toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-h_toy_airplane', synonyms=['072-h_toy_airplane', 'h toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-i_toy_airplane', synonyms=['072-i_toy_airplane', 'i toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-j_toy_airplane', synonyms=['072-j_toy_airplane', 'j toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='072-k_toy_airplane', synonyms=['072-k_toy_airplane', 'k toy airplane'], info='task items', location='basket_1', num=1),
            MyObject(name='073-a_lego_duplo', synonyms=['073-a_lego_duplo', 'a lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-b_lego_duplo', synonyms=['073-b_lego_duplo', 'b lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-c_lego_duplo', synonyms=['073-c_lego_duplo', 'c lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-d_lego_duplo', synonyms=['073-d_lego_duplo', 'd lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-e_lego_duplo', synonyms=['073-e_lego_duplo', 'e lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-f_lego_duplo', synonyms=['073-f_lego_duplo', 'f lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-g_lego_duplo', synonyms=['073-g_lego_duplo', 'g lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-h_lego_duplo', synonyms=['073-h_lego_duplo', 'h lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-i_lego_duplo', synonyms=['073-i_lego_duplo', 'i lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-j_lego_duplo', synonyms=['073-j_lego_duplo', 'j lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-k_lego_duplo', synonyms=['073-k_lego_duplo', 'k lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-l_lego_duplo', synonyms=['073-l_lego_duplo', 'l lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='073-m_lego_duplo', synonyms=['073-m_lego_duplo', 'm lego duplo'], info='task items', location='basket_1', num=1),
            MyObject(name='076_timer', synonyms=['076_timer', 'timer'], info='task items', location='basket_1', num=1),
            MyObject(name='077_rubiks_cube', synonyms=['077_rubiks_cube', 'rubiks cube'], info='task items', location='basket_1', num=1),
            
            MyObject(name='053-a_mini_soccer_ball', synonyms=['053-a_mini_soccer_ball', 'a mini soccer ball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='053-b_mini_soccer_ball_v2', synonyms=['053-b_mini_soccer_ball_v2', 'b mini soccer ball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='054_softball', synonyms=['054_softball', 'softball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='055_baseball', synonyms=['055_baseball', 'baseball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='056_tennis_ball', synonyms=['056_tennis_ball', 'tennis ball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='057_racquetball', synonyms=['057_racquetball', 'racquetball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='058_golf_ball', synonyms=['058_golf_ball', 'golf ball'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='059_chain', synonyms=['059_chain', 'chain'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='061_foam_brick', synonyms=['061_foam_brick', 'foam brick'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='062_dice', synonyms=['062_dice', 'dice'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-a_marbles', synonyms=['063-a_marbles', 'a marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-b_marbles', synonyms=['063-b_marbles', 'b marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-c_marbles', synonyms=['063-c_marbles', 'c marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-d_marbles', synonyms=['063-d_marbles', 'd marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-e_marbles', synonyms=['063-e_marbles', 'e marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='063-f_marbles', synonyms=['063-f_marbles', 'f marbles'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-a_cups', synonyms=['065-a_cups', 'a cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-b_cups', synonyms=['065-b_cups', 'b cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-c_cups', synonyms=['065-c_cups', 'c cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-d_cups', synonyms=['065-d_cups', 'd cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-e_cups', synonyms=['065-e_cups', 'e cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-f_cups', synonyms=['065-f_cups', 'f cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-g_cups', synonyms=['065-g_cups', 'g cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-h_cups', synonyms=['065-h_cups', 'h cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-i_cups', synonyms=['065-i_cups', 'i cups'], info='shape items', location='drawer_left_open', num=1),
            MyObject(name='065-j_cups', synonyms=['065-j_cups', 'j cups'], info='shape items', location='drawer_left_open', num=1),
            
            MyObject(name='019-a_pitcher_base', synonyms=['019-a_pitcher_base', 'a pitcher base'], info='kitchen items', location='container_1', num=1),
            MyObject(name='019-b_pitcher_base_v2', synonyms=['019-b_pitcher_base_v2', 'b pitcher base'], info='kitchen items', location='container_1', num=1),
            MyObject(name='020_pitcher_lid', synonyms=['020_pitcher_lid', 'pitcher lid'], info='kitchen items', location='container_1', num=1),
            MyObject(name='021_bleach_cleanser', synonyms=['021_bleach_cleanser', 'bleach cleanser'], info='kitchen items', location='container_1', num=1),
            MyObject(name='022_windex_bottle', synonyms=['022_windex_bottle', 'windex bottle'], info='kitchen items', location='container_1', num=1),
            MyObject(name='023_wine_glass', synonyms=['023_wine_glass', 'wine glass'], info='kitchen items', location='container_1', num=1),
            MyObject(name='024_bowl', synonyms=['024_bowl', 'bowl'], info='kitchen items', location='container_1', num=1),
            MyObject(name='025_mug', synonyms=['025_mug', 'mug'], info='kitchen items', location='container_1', num=1),
            MyObject(name='026_sponge', synonyms=['026_sponge', 'sponge'], info='kitchen items', location='container_1', num=1),
            MyObject(name='029_plate', synonyms=['029_plate', 'plate'], info='kitchen items', location='container_1', num=1),
            MyObject(name='030_fork', synonyms=['030_fork', 'fork'], info='kitchen items', location='container_2', num=1),
            MyObject(name='031_spoon', synonyms=['031_spoon', 'spoon'], info='kitchen items', location='container_2', num=1),
            MyObject(name='032_knife', synonyms=['032_knife', 'knife'], info='kitchen items', location='container_1', num=1),
            MyObject(name='033_spatula', synonyms=['033_spatula', 'spatula'], info='kitchen items', location='container_1', num=1)
            
            #GPSR
            ###MyObject(name='004_sugar_box', synonyms=['004_sugar_box', 'water', 'water bottle', 'evian', 'evian bottle'], info='drink item', location='chair_a', num=2),
            ###MyObject(name='002_master_chef_can', synonyms=['002_master_chef_can', 'milk tea', 'milk tea bottle', ], info='drink item', location='chair_b', num=2),
            ###MyObject(name='003_cracker_box', synonyms=['003_cracker_box', 'biscuits', 'biscuits box', 'cookies', 'cookies box'], info='food item', location='shelf_middle', num=2),
            ###MyObject(name='019-a_pitcher_base', synonyms=['019-a_pitcher_base', 'corn soup', 'cup soup', 'corn', 'soup'], info='food item', location='shelf_middle', num=2),
            ###MyObject(name='013_apple', synonyms=['013_apple', 'apple'], info='fruits item', location='tray_2', num=2),
            ###MyObject(name='014_lemon', synonyms=['014_lemon', 'lemon', 'lime'], info='fruits item', location='tray_2', num=2),
            ###MyObject(name='036_wood_blocks', synonyms=['036_wood_blocks', 'bowl', 'dish'], info='dishware', location='long_table_center', num=2),
            ###MyObject(name='019-b_pitcher_base_v2', synonyms=['019-b_pitcher_base_v2', 'mug', 'cup'], info='dishware', location='long_table_center', num=2)
            
        ]
    elif location == "robocup":
        objects_info = [
            MyObject(name='tuna', synonyms=['tuna', 'tuna'], info='food', location='pantry', num=1),
            MyObject(name='tomato_soup', synonyms=['tomato_soup', 'tomato_soup', 'tomato soup'], info='food', location='pantry', num=1),
            MyObject(name='spam', synonyms=['spam', 'spam'], info='food', location='pantry', num=1),
            MyObject(name='mustard', synonyms=['mustard', 'mustard'], info='food', location='pantry', num=1),
            MyObject(name='strawberry_jello', synonyms=['strawberry_jello', 'strawberry_jello', 'strawberry jello'], info='food', location='pantry', num=1),
            MyObject(name='chocolate_jello', synonyms=['chocolate_jello', 'chocolate_jello', 'chocolate jello'], info='food', location='pantry', num=1),
            MyObject(name='coffee_grounds', synonyms=['coffee_grounds', 'coffee_grounds', 'coffee grounds'], info='food', location='pantry', num=1),
            MyObject(name='sugar', synonyms=['sugar', 'sugar'], info='food', location='pantry', num=1),
            
            MyObject(name='banana', synonyms=['banana', 'banana'], info='fruits', location='couch_table', num=1),
            MyObject(name='strawberry', synonyms=['strawberry', 'strawberry'], info='fruits', location='couch_table', num=1),
            MyObject(name='apple', synonyms=['apple', 'apple'], info='fruits', location='couch_table', num=1),
            MyObject(name='lemon', synonyms=['lemon', 'lemon'], info='fruits', location='couch_table', num=1),
            MyObject(name='peach', synonyms=['peach', 'peach'], info='fruits', location='couch_table', num=1),
            MyObject(name='plum', synonyms=['plum', 'plum'], info='fruits', location='couch_table', num=1),
            MyObject(name='pear', synonyms=['pear', 'pear'], info='fruits', location='couch_table', num=1),
            MyObject(name='orange', synonyms=['orange', 'orange'], info='fruits', location='couch_table', num=1),
            
            MyObject(name='plate', synonyms=['plate', 'plate'], info='dishes', location='dinner_table', num=1),
            MyObject(name='bowl', synonyms=['bowl', 'bowl'], info='dishes', location='dinner_table', num=1),
            MyObject(name='fork', synonyms=['fork', 'fork'], info='dishes', location='dinner_table', num=1),
            MyObject(name='spoon', synonyms=['spoon', 'spoon'], info='dishes', location='dinner_table', num=1),
            MyObject(name='knife', synonyms=['knife', 'knife'], info='dishes', location='dinner_table', num=1),
            MyObject(name='cup', synonyms=['cup', 'cup'], info='dishes', location='dinner_table', num=1),
            
            MyObject(name='soccer_ball', synonyms=['soccer_ball', 'soccer_ball', 'soccer ball'], info='toys', location='desk', num=1),
            MyObject(name='baseball', synonyms=['baseball', 'baseball'], info='toys', location='desk', num=1),
            MyObject(name='tennis_ball', synonyms=['tennis_ball', 'tennis_ball', 'tennis ball'], info='toys', location='desk', num=1),
            MyObject(name='dice', synonyms=['dice', 'dice'], info='toys', location='desk', num=1),
            MyObject(name='rubiks_cube', synonyms=['rubiks_cube', 'rubiks_cube', 'rubiks cube'], info='toys', location='desk', num=1),
            
            MyObject(name='pringles', synonyms=['pringles', 'pringles'], info='snacks', location='snack_bin', num=1),
            MyObject(name='cornflakes', synonyms=['cornflakes', 'cornflakes'], info='snacks', location='snack_bin', num=1),
            MyObject(name='cheezit', synonyms=['cheezit', 'cheezit'], info='snacks', location='snack_bin', num=1),
            
            MyObject(name='red_wine', synonyms=['red_wine', 'red_wine', 'red wine'], info='drinks', location='side_table', num=1),
            MyObject(name='juice_pack', synonyms=['juice_pack', 'juice_pack', 'juice pack'], info='drinks', location='side_table', num=1),
            MyObject(name='orange_juice', synonyms=['orange_juice', 'orange_juice', 'orange juice'], info='drinks', location='side_table', num=1),
            MyObject(name='milk', synonyms=['milk', 'milk'], info='drinks', location='side_table', num=1),
            MyObject(name='tropical_juice', synonyms=['tropical_juice', 'tropical_juice', 'tropical juice'], info='drinks', location='side_table', num=1),
            MyObject(name='iced_tea', synonyms=['iced_tea', 'iced_tea', 'iced tea'], info='drinks', location='side_table', num=1),
            MyObject(name='cola', synonyms=['cola', 'cola'], info='drinks', location='side_table', num=1),
            
            MyObject(name='cleanser', synonyms=['cleanser', 'cleanser'], info='cleaning_supplies', location='small_shelf', num=1),
            MyObject(name='sponge', synonyms=['sponge', 'sponge'], info='cleaning_supplies', location='small_shelf', num=1),
            MyObject(name='unknown', synonyms=['unknown', 'unknown'], info='unknown', location='office_bin', num=1)
        ]
    else:
        objects_info = [
            MyObject(name='001_chips_can', synonyms=['001_chips_can', 'chips', 'chips can'], info='food items', location='large_table', num=1)
        ]
    
    return objects_info


#( item (type Human) (name nil) (room nil) (zone nil) (attributes nil) (objs nil) (num 1) )
def load_human_info(location="stream"):
    
    if location == "stream":
        human_info = [
            #DEMO
            #MyObject(name='person', synonyms=['person', 'person', 'human', 'anyone', 'someone', 'unknown', 'them', 'her', 'him'], location='office_unknown'),
            MyObject(name='paola', synonyms=['paola', 'paola', 'paula', 'polly'], location='office_unknown'),
            MyObject(name='luis', synonyms=['luis', 'luis', 'louis', 'lewis'], location='living_room_unknown'),
            MyObject(name='mary', synonyms=['mary', 'mary', 'marry', 'mary jane'], location='studio_unknown'),
            MyObject(name='mike', synonyms=['mike', 'mike', 'mark', 'mac'], location='dining_room_unknown'),
            ###
            
            MyObject(name='amelia', synonyms=['amelia', 'amelia', 'amely', 'amelie'], location='dining_room_unknown'),
            MyObject(name='ava', synonyms=['ava', 'ava'], location='dining_room_unknown'),
            MyObject(name='charlotte', synonyms=['charlotte', 'charlotte'], location='dining_room_unknown'),
            MyObject(name='olivia', synonyms=['olivia', 'olivia'], location='dining_room_unknown'),
            
            MyObject(name='angel', synonyms=['angel', 'angel', 'angela', 'angie'], location='dining_room_unknown'),
            MyObject(name='charlie', synonyms=['charlie', 'charlie'], location='dining_room_unknown'),
            MyObject(name='hunter', synonyms=['hunter', 'hunter'], location='dining_room_unknown'),
            MyObject(name='max', synonyms=['max', 'max'], location='dining_room_unknown'),
            MyObject(name='parker', synonyms=['parker', 'parker'], location='dining_room_unknown'),
            MyObject(name='sam', synonyms=['sam', 'sam'], location='dining_room_unknown'),
            
            MyObject(name='alex', synonyms=['alex', 'alex'], location='dining_room_unknown'),
            MyObject(name='steve', synonyms=['steve', 'steve'], location='dining_room_unknown'),
            MyObject(name='robert', synonyms=['robert', 'robert'], location='dining_room_unknown'),
            MyObject(name='winston', synonyms=['winston', 'winston'], location='dining_room_unknown'),
            MyObject(name='colin', synonyms=['colin', 'colin'], location='dining_room_unknown'),
            MyObject(name='bill', synonyms=['bill', 'bill'], location='dining_room_unknown'),
            MyObject(name='adam', synonyms=['adam', 'adam'], location='dining_room_unknown'),
            MyObject(name='jonathan', synonyms=['jonathan', 'jonathan'], location='dining_room_unknown'),
            MyObject(name='peter', synonyms=['peter', 'peter'], location='dining_room_unknown')
        ]
    elif location == "robocup":
        human_info = [
            MyObject(name='person', synonyms=['person', 'person', 'human', 'anyone', 'someone', 'unknown', 'them', 'me', 'her', 'him'], location='office_unknown'),
            MyObject(name='adel', synonyms=['adel', 'adele', 'abel'], location='dining_room_unknown'),
            MyObject(name='angel', synonyms=['angel', 'angel', 'angela', 'angie'], location='dining_room_unknown'),
            MyObject(name='axel', synonyms=['axel', 'axel', 'alex'], location='dining_room_unknown'),
            MyObject(name='charlie', synonyms=['charlie', 'charlie', 'charles'], location='dining_room_unknown'),
            MyObject(name='jane', synonyms=['jane', 'jane', 'joan', 'joanna'], location='dining_room_unknown'),
            MyObject(name='jules', synonyms=['jules', 'jules'], location='dining_room_unknown'),
            MyObject(name='morgan', synonyms=['morgan', 'morgan'], location='dining_room_unknown'),
            MyObject(name='paris', synonyms=['paris', 'paris'], location='dining_room_unknown'),
            MyObject(name='robin', synonyms=['robin', 'robin'], location='dining_room_unknown'),
            MyObject(name='simone', synonyms=['simone', 'simone'], location='dining_room_unknown'),
            
            MyObject(name='john', synonyms=['john', 'john', 'jonathan'], location='dining_room_unknown')
        ]
    else:
        human_info = [
            MyObject(name='paola', synonyms=['paola', 'paola', 'paula', 'polly'], location='office_unknown'),
            MyObject(name='luis', synonyms=['luis', 'luis', 'louis', 'lewis'], location='dining_room_unknown')
        ]
    
    return human_info

