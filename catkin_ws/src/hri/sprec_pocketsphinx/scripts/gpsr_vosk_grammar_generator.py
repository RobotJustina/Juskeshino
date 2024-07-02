#!/usr/bin/env python3
#This package is suppose to generate a vosk grammar base on the 2024 @home command generator
#But it is not working very well. Sorry.
import rospy
import rospkg
import time
import re
import numpy as np
import rospkg
from CompetitionTemplate.command_generator.gpsr_commands import CommandGenerator
from grammar.dict_builder import build_dict

def read_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    return data


def parse_names(data):
    parsed_names = re.findall(r'\|\s*([A-Za-z]+)\s*\|', data, re.DOTALL)
    parsed_names = [name.strip() for name in parsed_names]

    if parsed_names:
        return parsed_names[1:]
    else:
        warnings.warn("List of names is empty. Check content of names markdown file")
        return []


def parse_locations(data):
    parsed_locations = re.findall(r'\|\s*([0-9]+)\s*\|\s*([A-Za-z,\s, \(,\)]+)\|', data, re.DOTALL)
    parsed_locations = [b for (a, b) in parsed_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    parsed_placement_locations = [location for location in parsed_locations if location.endswith('(p)')]
    parsed_locations = [location.replace('(p)', '') for location in parsed_locations]
    parsed_placement_locations = [location.replace('(p)', '') for location in parsed_placement_locations]
    parsed_placement_locations = [location.strip() for location in parsed_placement_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    if parsed_locations:
        return parsed_locations, parsed_placement_locations
    else:
        warnings.warn("List of locations is empty. Check content of location markdown file")
        return []


def parse_rooms(data):
    parsed_rooms = re.findall(r'\|\s*(\w+ \w*)\s*\|', data, re.DOTALL)
    parsed_rooms = [rooms.strip() for rooms in parsed_rooms]

    if parsed_rooms:
        return parsed_rooms[1:]
    else:
        warnings.warn("List of rooms is empty. Check content of room markdown file")
        return []


def parse_objects(data):
    parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
    parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
    parsed_objects = [objects.strip() for objects in parsed_objects]

    parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsed_categories = [category.strip() for category in parsed_categories]
    parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
    parsed_categories_plural = [category[0] for category in parsed_categories]
    parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
    parsed_categories_singular = [category[1] for category in parsed_categories]
    parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

    if parsed_objects or parsed_categories:
        return parsed_objects, parsed_categories_plural, parsed_categories_singular
    else:
        warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
        return []

def main():
    print("INITIALIZING GPSR TEST Not in a Tenshily manner...")
    rospy.init_node("gpsr")
    rate = rospy.Rate(10)
    
    vosk_grammar = CommandGenerator.color_clothes_list + CommandGenerator.color_clothe_list + CommandGenerator.clothe_list
    vosk_grammar+= CommandGenerator.color_list + CommandGenerator.question_list + CommandGenerator.talk_list
    vosk_grammar+= CommandGenerator.object_comp_list + CommandGenerator.person_info_list + CommandGenerator.pose_person_plural_list
    vosk_grammar+= CommandGenerator.gesture_person_plural_list +  CommandGenerator.pose_person_list
    vosk_grammar+= CommandGenerator.gesture_person_list + CommandGenerator.connector_list
    for k in CommandGenerator.verb_dict:
        for w in CommandGenerator.verb_dict[k]:
            vosk_grammar.append(w)
    for k in CommandGenerator.prep_dict:
        for w in CommandGenerator.prep_dict[k]:
            vosk_grammar.append(w)

    pkg = rospkg.RosPack()
    act_pln_path = pkg.get_path("act_pln") + '/scripts/'
    names_file_path = act_pln_path + 'CompetitionTemplate/names/names.md'
    locations_file_path = act_pln_path + 'CompetitionTemplate/maps/location_names.md'
    rooms_file_path = act_pln_path + 'CompetitionTemplate/maps/room_names.md'
    objects_file_path = act_pln_path + 'CompetitionTemplate/objects/objects.md'
    cmudict = act_pln_path + 'grammar/cmudict.dic'

    names_data = read_data(names_file_path)
    names = parse_names(names_data)
    names = [n.lower() for n in names]
        

    locations_data = read_data(locations_file_path)
    location_names, placement_location_names = parse_locations(locations_data)

    rooms_data = read_data(rooms_file_path)
    room_names = parse_rooms(rooms_data)

    objects_data = read_data(objects_file_path)
    object_names, object_categories_plural, object_categories_singular = parse_objects(objects_data)

    vosk_grammar += names + location_names + placement_location_names + room_names
    vosk_grammar += object_names + object_categories_plural + object_categories_singular
    grammar = []
    for v in vosk_grammar:
        parts = v.split()
        grammar += parts
    
    build_dict(grammar, cmudict)
    
    while not rospy.is_shutdown():
        
        rate.sleep()


if __name__ == "__main__":
    main()
