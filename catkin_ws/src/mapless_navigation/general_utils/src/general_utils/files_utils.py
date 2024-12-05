#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
from pathlib import Path
import rospkg
import cv2

# Directory tools >>
class DirectoryUtils:
    def __init__(self):
        pass

    @staticmethod
    def existDir(directory_path="", verbose=False):
        if verbose:
            print("Verifying if directory path exist...")

        return os.path.exists(directory_path)

    @staticmethod
    def deleteDir(directory_path="", verbose=False):
        if verbose:
            print("rmdir: ", directory_path)
        if DirectoryUtils.existDir(directory_path, verbose):
            for root, dirs, files in os.walk(directory_path, topdown=False):
                for name in files:
                    file_path = os.path.join(root, name)
                    os.remove(file_path)
                for name in dirs:
                    dir_path = os.path.join(root, name)
                    os.rmdir(dir_path)
            try:
                os.rmdir(directory_path)
                if verbose:
                    print("Directory deleted")
                return True
            except OSError as error:
                print("Failed to delete directory!", error)
                return False
        else:
            if verbose:
                print("Directory not found!")
            return False

    @staticmethod
    def createDir(directory_path="", verbose=False):
        if verbose:
            print("make dirs: ", directory_path)
        if DirectoryUtils.existDir(directory_path, verbose):
            if verbose:
                print("Directory already exist!")
            return False
        else:
            try:
                os.makedirs(directory_path, exist_ok=True)
                print("Directory created")
                return True
            except OSError as error:
                print("Failed to create directory!", error)
                return False

    @staticmethod
    def replaceDir(directory_path, verbose=False):
        deleted = DirectoryUtils.deleteDir(directory_path, verbose)
        created = DirectoryUtils.createDir(directory_path)
        if not created:
            if verbose:
                print("Directory not replaced")
            return False

        if verbose:
            if not deleted:
                print("Directory successful created")
            else:
                print("Directory successful replaced")
        return True
    
    def getRelativePath(absolute_path):
        #Locally can use: Path(__file__).resolve()
        FILE = Path(absolute_path).resolve()
        ROOT = FILE.parents[0] 
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
        return ROOT
    
    def getRosPkgPath(pkg_name="general_utils"):
        return rospkg.RosPack().get_path(pkg_name)

# Directory tools <<

# File tools >>
class FileUtils:
    def __init__(self):
        pass

    @staticmethod
    def showImage(self, verbose=False):
        pass

    @staticmethod
    def resizeImage(img, height=480, keep_prop=True, verbose=False):
        if verbose: print("image shape: ", img.shape)

        if keep_prop:
            aspect_ratio = round(float(img.shape[0]) / float(img.shape[1]), 4)
            if verbose: print("aspect ratio:", aspect_ratio)
            width = int(height/aspect_ratio)
        else:
            width = height
        
        if verbose: print("new dimensions: ",width, height)
        new_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_LINEAR)
        return new_img
    
    def loadImage(image_path="image_path"):
        try:
            image = cv2.imread(image_path)
        except:
            print("Can not find ", image_path)
            return None
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    def saveImage(img, name=""):
        if name == "":
            print("Error! no image name specified")
            return None
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imwrite(name, img)
    
# File tools <<

def testFunction():
    rospy.loginfo(".py files_utils")
    print("test function is called.")
