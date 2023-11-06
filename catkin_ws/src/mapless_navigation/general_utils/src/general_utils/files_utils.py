#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os


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
            print("makedirs: ", directory_path)
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
# Directory tools <<

# File tools >>
class FileUtils:
    def __init__(self):
        pass

    @staticmethod
    def showImage(self, verbose=False):
        pass
# File tools <<

def testFunction():
    rospy.loginfo(".py files_utils")
    print("test function is called.")
