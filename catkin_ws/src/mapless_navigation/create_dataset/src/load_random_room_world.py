#! /usr/bin/env python3
import rospy
import rospkg
import argparse
import shutil

parser = argparse.ArgumentParser()
parser.add_argument("--map", default="rnd_room_c2_01", type=str, help="Map to use, default: rnd_room_c2_01")
parser.add_argument("--replace", default="False", type=str, help="If True replace room_selected.world, default: False")


args = parser.parse_args(rospy.myargv()[1:])

def main():
    rospy.init_node("load_random_room_world")
    path = rospkg.RosPack().get_path("gazebo_envs")
    path += "/worlds/"
    src = path + "random_room.world"
    dst = path + "room_selected.world"
    rospy.loginfo("load_random_room_world Initialized")
    print("Replacing map to", args.map, "in", dst)
    
    if eval(args.replace):
        src = path + args.map + '.world'
        print("----------------------------Replace map-----------------------------------------")
        print("src:", src)
        print("dst:", dst)
        try:
            shutil.copy2(src, dst)
        except Exception as e:
            str = "An error occurred:" + e
            rospy.logerr(str)        
            return None
        
    if eval(args.replace)== False:
        with open(src, "rt") as fin:
            with open(dst, "wt") as fout:
                for line in fin:
                    fout.write(line.replace('rnd_room_c2_01', args.map))

    else:
        print("No map changes")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
