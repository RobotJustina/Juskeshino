#include "Utils.h"

bool  Utils::debug = false;
float Utils::min_x =  0.3;
float Utils::min_y = -2.0;
float Utils::min_z =  0.3;
float Utils::max_x =  2.0;
float Utils::max_y =  2.0;
float Utils::max_z =  2.0;

void Utils::pointcloud_msg2_cv_mat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
    int offset_x = 0;
    int offset_y = 4;
    int offset_z = 8;
    int offset_bgr  = 12;
    for(int i=0; i < pc_msg.fields.size(); i++)
        if (pc_msg.fields[i].name == "x")
            offset_x = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "y")
            offset_y = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "z")
            offset_z = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgb")
            offset_bgr = pc_msg.fields[i].offset;
        else if (pc_msg.fields[i].name == "rgba")
            offset_bgr = pc_msg.fields[i].offset;

    bgr_dest = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_x];
            float* y = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_y];
            float* z = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_z];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 1];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + offset_bgr + 2];
        }
}


void Utils::transform_cloud_wrt_base(sensor_msgs::PointCloud2& cloud, cv::Mat& bgr_dest, cv::Mat& cloud_dest,
                                     tf::TransformListener* tf_listener)
{
    std::cout <<"ObjReco.->Point cloud frame: " << cloud.header.frame_id << std::endl;
    if(cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", cloud, cloud, *tf_listener);
    }
    Utils::pointcloud_msg2_cv_mat(cloud, bgr_dest, cloud_dest);
}


void Utils::filter_by_distance(cv::Mat& cloud, cv::Mat& img, cv::Mat& filtered_cloud, cv::Mat& filtered_img)
{
    // This function is intended to keep point only in a given bounding box, e.g., to remove floot and distant walls
    // The function DOES NOT return a smaller point cloud. It returns a cloud with all non valid points set to zero. 
    cv::Mat valid_points;
    cv::inRange(cloud, cv::Scalar(Utils::min_x, Utils::min_y, Utils::min_z),
                cv::Scalar(Utils::max_x, Utils::max_y, Utils::max_z), valid_points);
    filtered_cloud = cloud.clone();
    filtered_img   = img.clone();
    for(size_t i=0; i<img.rows; i++)
        for(size_t j=0; j<img.cols; j++)
            if(!valid_points.data[i*img.cols + j])
            {
                filtered_cloud.at<cv::Vec3f>(i,j) = cv::Vec3f(0,0,0);
                filtered_img.at<cv::Vec3b>(i,j)   = cv::Vec3b(0,0,0);
            }
    if(Utils::debug)
    {
        cv::imshow("Filtered by distance RGB", filtered_img);
        cv::imshow("Filtered by distance XYZ", filtered_cloud);
    }
}

float Utils::dist_point_to_segment(float px, float py, float pz, float x1, float y1, float z1, float x2, float y2, float z2)
{
    float ax = px - x1;
    float ay = py - y1;
    float az = pz - z1;
    float bx = x2 - x1;
    float by = y2 - y1;
    float bz = z2 - z1;
    float bm = sqrt(bx*bx + by*by + bz*bz);
    if(bm == 0) return sqrt(ax*ax + ay*ay + az*az);
    bx /= bm;
    by /= bm;
    bz /= bm;
    float projection = ax*bx + ay*by + az*bz;
    if(projection < 0) return sqrt(ax*ax + ay*ay + az*az);
    if(projection > 1) return sqrt((px-x2)*(px-x2) + (py-y2)*(py-y2) + (pz-z2)*(pz-z2));
    return sqrt(ax*ax + ay*ay + az*az - projection);
}

float Utils::dist_point_to_segment(float px, float py, float x1, float y1, float x2, float y2)
{
    float ax = px - x1;
    float ay = py - y1;
    float bx = x2 - x1;
    float by = y2 - y1;
    float bm = sqrt(bx*bx + by*by);
    if(bm == 0) return sqrt(ax*ax + ay*ay);
    bx /= bm;
    by /= bm;
    float projection = ax*bx + ay*by;
    if(projection < 0) return sqrt(ax*ax + ay*ay);
    if(projection > 1) return sqrt((px-x2)*(px-x2) + (py-y2)*(py-y2));
    return sqrt(ax*ax + ay*ay - projection*projection*bm*bm);
}

visualization_msgs::Marker Utils::get_lines_marker(std::vector<geometry_msgs::Point> lines)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obj_reco_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.6;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(10.0);
    for(size_t i=0; i < lines.size(); i++) marker.points.push_back(lines[i]);
    return marker;
}

std::vector<geometry_msgs::Point> Utils::get_line_msg(std::vector<cv::Vec3f> line)
{
    std::vector<geometry_msgs::Point> msg;
    for(size_t i=0; i< line.size(); i++)
    {
        geometry_msgs::Point msg_p;
        msg_p.x = line[i][0];
        msg_p.y = line[i][1];
        msg_p.z = line[i][2];
        msg.push_back(msg_p);
    }
    return msg;
}

void draw_lines(cv::Mat& img, std::vector<cv::Vec2f>& lines)
{
    for(size_t i=0; i< lines.size(); i++)
    {
        float d = lines[i][0], theta = lines[i][1];
        cv::Point p1, p2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*d, y0 = b*d;
        p1.x = round(x0 + 1000*(-b));
        p1.y = round(y0 + 1000*(a));
        p2.x = round(x0 - 1000*(-b));
        p2.y = round(y0 - 1000*(a));
        cv::line(img, p1, p2, cv::Scalar(0,255,0), 2, cv::LINE_AA);
    }
}

/* Draw a set of lines given as a set of pairs (r1,theta1),(r2,theta2),(r3,theta3),...
 * img  : Image to draw in
 * lines: set of lines in normal form
 */
void Utils::draw_lines(cv::Mat& img, std::vector<cv::Vec2f>& lines)
{
    for(size_t i=0; i< lines.size(); i++)
    {
        float d = lines[i][0], theta = lines[i][1];
        cv::Point p1, p2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*d, y0 = b*d;
        p1.x = round(x0 + 1000*(-b));
        p1.y = round(y0 + 1000*(a));
        p2.x = round(x0 - 1000*(-b));
        p2.y = round(y0 - 1000*(a));
        cv::line(img, p1, p2, cv::Scalar(0,255,0), 2, cv::LINE_AA);
    }
}


/* My own implementation of Hough Transform for detecting lines
 * Parameters:
 * - A binary image (commonly, the result of an edge detection process)
 * - A point cloud with xyz information for pixels in binary image
 * - d_min:  min distance to search lines. Recommended: d_step
 * - d_max:  max distance to search lines. Recommended: image diagonal length in pixels
 * - d_step: distance increment step. Recommended: something around (d_max-d_min)/100
 * - theta_min:    min angle to search lines. Recommended: -pi
 * - theta_max:    max angle to search lines. Recommended: pi
 * - theta_step:   angle increment step. Recommended: something around 0.017 (1 degree)
 * - threshold:    min number of votes to consider a line is found. Recommended: add a trackbar and test.
 * - A set of lines in image coordinates in the form (r1,theta1),(r2,theta2),(r3,theta3),...
 * Returns:
 * - A set of lines in cartesian coordinates given by a set of points of the form
 *   (x11,y11,z11),(x12,y12,z12), (x21,y21,z21),(x22,y22,z22), (x31,y31,z31),(x32,y32,z32), ...,
 */
std::vector<cv::Vec3f> Utils::hough_lines(cv::Mat img_bin, cv::Mat xyz, float d_min, float d_max, int d_step, float theta_min,
                                   float theta_max, float theta_step, int threshold, std::vector<cv::Vec2f>& lines_img)
{
    if(Utils::debug) std::cout << "ObjReco-Utils->Executing hough transform for finding lines..." << std::endl;
    /*
     * dist_n : number of values for quantized distance
     * theta_n: number of values for quantized angle
     */
    int dist_n  = (int)ceil((d_max - d_min)/d_step);
    int theta_n = (int)ceil((theta_max - theta_min)/theta_step);

    /*
     * votes     : 2D array to stores votes for each possible line in the Hough Space
     * pixel_acc : 2D array to store a list of pixel coordinates corresponding to each given vote
     * sines     : Precalculated sine values for each quantized angle
     * cosines   : Precalculated cosines values for each quantized angle
     */
    cv::Mat votes = cv::Mat::zeros(dist_n, theta_n, CV_32SC1);
    std::vector<std::vector<std::vector<cv::Vec2i> > > pixel_acc;
    pixel_acc.resize(dist_n);
    for(size_t i=0; i < pixel_acc.size(); i++) pixel_acc[i].resize(theta_n);
    float sines[theta_n], cosines[theta_n];

    //Precalculate sine and cosine for each quantized angle
    for(size_t k=0; k<theta_n; k++, sines[k]=sin(theta_min+theta_step*k), cosines[k]=cos(theta_min+theta_step*k));

    //Loop over all pixels in the binary image
    for(size_t i=0; i < img_bin.rows; i++)
        for(size_t j=0; j < img_bin.cols; j++)
            //For each non-zero pixel, calculate quantized distance for each quantized angle
            //given a cartesian point (i,j) in image coordinates
            if(img_bin.at<unsigned char>(i,j) != 0)
                for(size_t k=0; k<theta_n; k++)
                {
                    int d = (int)((j*cosines[k] + i*sines[k] - d_min)/d_step);
                    if(d >= 0 && d < dist_n)
                    {
                        votes.at<int>(d,k)++;
                        pixel_acc[d][k].push_back(cv::Vec2i(i,j));
                    }
                }

    std::vector<cv::Vec3f> lines;
    lines_img.clear();
    //std::vector<cv::Vec2f> lines_img; //Lines in image coordinates in the form rho, theta
    //Loop over all points in the Hough Space
    for(size_t i=0; i<dist_n; i++)
        for(size_t j=0; j<theta_n; j++)
            //IF A POINT IN HOUGH SPACE HAS A NUMBER OF VOTES GREATER THAN A GIVEN THRESHOLD
            //THEN, WE FOUND A LINE!
            if(votes.at<int>(i,j) > threshold)
            {
                lines_img.push_back(cv::Vec2f(i*d_step + d_min, j*theta_step + theta_min));
                cv::Mat points = cv::Mat(votes.at<int>(i,j), 3, CV_32F);
                for(size_t k=0; k < pixel_acc[i][j].size(); k++)
                {
                    cv::Vec2i idx = pixel_acc[i][j][k];
                    cv::Vec3f p = xyz.at<cv::Vec3f>(idx[0], idx[1]);
                    if(cv::norm(p) < 0.1) continue;
                    points.at<float>(k,0) = p[0];
                    points.at<float>(k,1) = p[1];
                    points.at<float>(k,2) = p[2];
                }
                std::vector<cv::Vec3f> line = Utils::line_by_pca(points);
                lines.push_back(line[0]);
                lines.push_back(line[1]);
            }
    if(Utils::debug) std::cout << "ObjReco-Utils->Found " << lines.size()/2 << " lines by hough transform." << std::endl;
    return lines;
}

std::vector<cv::Vec3f> Utils::line_by_pca(cv::Mat& points)
{
    std::vector<cv::Vec3f> line(2);
    cv::PCA pca(points, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if(Utils::debug)
    {
        std::cout << "ObjReco->PCA: number of points: " << points.rows << std::endl;
        std::cout << "ObjReco->PCA mean: " << pca.mean << std::endl;
        std::cout << "ObjReco->PCA vectors: " << pca.eigenvectors << std::endl;
        std::cout << "ObjReco->PCA eigenvalues: " << pca.eigenvalues << std::endl;
    }
    float line_mag = 2*sqrt(pca.eigenvalues.at<float>(0));
    line[0][0] = pca.mean.at<float>(0,0) + line_mag*pca.eigenvectors.at<float>(0,0);
    line[0][1] = pca.mean.at<float>(0,1) + line_mag*pca.eigenvectors.at<float>(0,1);
    line[0][2] = pca.mean.at<float>(0,2) + line_mag*pca.eigenvectors.at<float>(0,2);
    line[1][0] = pca.mean.at<float>(0,0) - line_mag*pca.eigenvectors.at<float>(0,0);
    line[1][1] = pca.mean.at<float>(0,1) - line_mag*pca.eigenvectors.at<float>(0,1);
    line[1][2] = pca.mean.at<float>(0,2) - line_mag*pca.eigenvectors.at<float>(0,2);
    return line;
}
