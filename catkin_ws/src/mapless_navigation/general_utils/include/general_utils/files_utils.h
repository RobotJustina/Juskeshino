#pragma once
#include <ros/ros.h>
#include <sys/stat.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

// Directory tools >>
class DirectoryUtils
{
private:
    DirectoryUtils() = delete;  // Disallow object creation
    ~DirectoryUtils() = delete; // Disallow object deletion
public:
    static bool existDir(std::string directory_path, bool verbose = false);
    static bool deleteDir(std::string directory_path, bool verbose = false);
    static bool createDir(std::string directory_path, bool verbose = false);
    static bool replaceDir(std::string directory_path, bool verbose = false);
};
// Directory tools <<

// File tools >>
class FileUtils
{
    private:
        FileUtils() = delete;  // Disallow object creation
        ~FileUtils() = delete; // Disallow object deletion
    public:
        static bool showImage(std::string file_path, bool verbose = false);
};
// File tools <<


void testFunction();