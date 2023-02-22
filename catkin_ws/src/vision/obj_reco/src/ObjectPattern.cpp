#include "ObjectPattern.h"

ObjectPattern::ObjectPattern()
{
    name = "";
    height = -1;
}

ObjectPattern::ObjectPattern(std::string name, cv::Mat histogram)
{
    this->name = name;
    this->histogram = histogram;
}

ObjectPattern::~ObjectPattern()
{
}
