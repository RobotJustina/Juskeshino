# Credit to Raphael Memmesheimer for creating this script

import os
import sys

rootdir = sys.argv[1]
os.system("find . -name '*.jpg' -execdir mogrify -resize 200x {} \;")
os.system("find . -name '*.png' -execdir mogrify -resize 200x {} \;")
skip_root = True
for subdir, dirs, files in os.walk(rootdir):
    if skip_root:
        skip_root = False
        continue
    print("# Class %s" % subdir[subdir.find("/")+1:])
    print("""
| Objectname               |  Image                   |
:-------------------------:|:-------------------------:""")
    for file in files:
        print("| %s  |  ![](%s) |" % (file[:file.find(".")], os.path.join(subdir, file)))
    print("\n")