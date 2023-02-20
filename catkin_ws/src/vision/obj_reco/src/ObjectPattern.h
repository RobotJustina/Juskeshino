#include "opencv2/opencv.hpp"

class ObjectPattern
{
public:
    ObjectPattern();
    ObjectPattern(std::string name, cv::Mat histogram);
    ~ObjectPattern();

    std::string name;
    cv::Mat histogram;
    float height;
};
