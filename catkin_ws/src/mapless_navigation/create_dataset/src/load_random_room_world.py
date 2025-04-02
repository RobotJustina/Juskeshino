#! /usr/bin/env python3
import rospy
import rospkg
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--map", default="rnd_room_c2_01", type=str, help="Map to use, default: rnd_room_c2_01")

args = parser.parse_args(rospy.myargv()[1:])

def main():

    rospy.init_node("load_random_room_world")

    path = rospkg.RosPack().get_path("gazebo_envs")
    path += "/worlds/"
    src = path + "random_room.world"
    dst = path + "room_selected.world"

    rospy.loginfo("load_random_room_world Initialized")
    print("Replacing map to", args.map, "in", dst)
    
    with open(src, "rt") as fin:
        with open(dst, "wt") as fout:
            for line in fin:
                fout.write(line.replace('rnd_room_c2_01', args.map))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
