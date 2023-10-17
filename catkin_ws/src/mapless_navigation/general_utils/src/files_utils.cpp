#include "general_utils/files_utils.h"
// #include <ros/package.h>

// Directory tools >>
bool DirectoryUtils::existDir(std::string directory_path, bool verbose)
{
    if (verbose)
        ROS_INFO("Verifying if directory path exist...");

    struct stat sb;
    if (stat(directory_path.c_str(), &sb) == 0)
        return true;
    else
        return false;
}

bool DirectoryUtils::deleteDir(std::string directory_path, bool verbose)
{
    std::string command = "rm -r " + directory_path;
    const char *_command = command.c_str();
    int sys_value;

    if (verbose)
        ROS_WARN("Executing: %s", _command);
    if (DirectoryUtils::existDir(directory_path, verbose))
    {
        sys_value = system(_command); // Executing command
        if (sys_value == 0)
        {
            if (verbose)
                ROS_INFO("Directory deleted");
            return true;
        }
        else
        { // Some other error
            ROS_ERROR("Failed to delete directory!");
            return false;
        }
    }
    else
    {
        if (verbose)
            ROS_WARN("Directory not found!");
        return false;
    }
}

bool DirectoryUtils::createDir(std::string directory_path, bool verbose)
{
    std::string command = "mkdir " + directory_path;
    const char *_command = command.c_str();

    int sys_value;

    if (verbose)
        ROS_INFO("Executing: %s", _command);
    if (DirectoryUtils::existDir(directory_path, verbose))
    {
        if (verbose)
            ROS_WARN("Directory already exist!");
        return false;
    }
    else
    {
        sys_value = system(_command); // Executing command
        if (sys_value == 0)
        {
            if (verbose)
                ROS_INFO("Directory created");
            return true;
        }
        else
        { // Some other error
            ROS_ERROR("Failed to create directory!");
            return false;
        }
    }
}

bool DirectoryUtils::replaceDir(std::string directory_path, bool verbose)
{
    bool created, deleted;
    deleted = DirectoryUtils::deleteDir(directory_path, verbose);
    created = DirectoryUtils::createDir(directory_path);

    if (!created)
    {
        if (verbose)
            ROS_ERROR("Directory not replaced");
        return false;
    }

    if (!deleted)
    { // Directory was not deleted
        if (verbose)
            ROS_INFO("Directory successful created");
    }
    else
    { // Directory was deleted
        if (verbose)
            ROS_INFO("Directory successful replaced");
    }
    return true;
}
// Directory tools <<

// File tools >>
bool FileUtils::showImage(std::string file_path, bool verbose)
{
    cv::Mat img = cv::imread(file_path);
    if (img.empty())
    {
        ROS_ERROR("Could not read the image: %s", file_path.c_str());
        return false;
    }
    if (verbose)
        ROS_INFO("Image loaded: %s", file_path.c_str());

    cv::imshow("Display window", img);
    cv::waitKey(0); // Wait for a keystroke in the window
    return true;
}
// File tools <<

void testFunction()
{
    ROS_INFO(".h files_utils");
    std::cout << "test function is called." << std::endl;
}