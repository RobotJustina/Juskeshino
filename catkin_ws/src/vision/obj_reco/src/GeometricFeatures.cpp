#include "GeometricFeatures.h"

cv::Mat GeometricFeatures::get_horizontal_normals(cv::Mat& cloud, float normal_min_z, bool debug)
{
    float mag;
    cv::Vec3f p, p_ne, p_se, p_nw, p_sw, v1, v2, normal;
    cv::Mat normals = cv::Mat::zeros(cv::Size(cloud.cols, cloud.rows), CV_32FC3);
    for(size_t i=1; i<cloud.rows-1; i++)
        for(size_t j=1; j<cloud.cols-1; j++)
        {
            p    = cloud.at<cv::Vec3f>(i,j);
            p_ne = cloud.at<cv::Vec3f>(i+1,j+1);
            p_nw = cloud.at<cv::Vec3f>(i+1,j-1);
            p_sw = cloud.at<cv::Vec3f>(i-1,j-1);
            p_se = cloud.at<cv::Vec3f>(i-1,j+1);
            if(p == cv::Vec3f(0,0,0) || p_ne == cv::Vec3f(0,0,0) || p_nw == cv::Vec3f(0,0,0) ||
               p_sw == cv::Vec3f(0,0,0) || p_se == cv::Vec3f(0,0,0))
                continue;
            normal = (p_ne - p_sw).cross(p_nw - p_se);
            mag = sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
            if(mag != 0) normal /= mag;
            if(normal[2] < 0) normal = -normal;
            if(normal[2] < normal_min_z)
                continue;
            normals.at<cv::Vec3f>(i,j) = normal;
        }
    if(debug)
        cv::imshow("Horizontal Normals", normals);
    return normals;
}

std::vector<cv::Vec3f> GeometricFeatures::get_horizontal_planes(cv::Mat& cloud, float normal_min_z, float dist_threshold, float min_area, 
                                                                cv::Mat& output_img_mask, bool debug)
{
    cv::Mat normals = GeometricFeatures::get_horizontal_normals(cloud, normal_min_z, debug);
    cv::cvtColor(normals, output_img_mask, cv::COLOR_BGR2GRAY);
    output_img_mask.convertTo(output_img_mask, CV_8UC1, 255);
    cv::threshold(output_img_mask, output_img_mask, 10, 255, cv::THRESH_BINARY);
    if(debug) cv::imshow("Binary image with normals", output_img_mask);
    std::vector<cv::Vec3f> inliers, outliers;
    std::vector<cv::Vec3f> plane = GeometricFeatures::plane_by_ransac(cloud, output_img_mask, normal_min_z, dist_threshold, min_area, inliers,
                                                                      outliers, debug);
    return plane;
}

std::vector<cv::Vec3f> GeometricFeatures::above_horizontal_plane(cv::Mat& cloud, std::vector<cv::Vec3f> plane, float dist_threshold,
                                                                 cv::Mat& output_img_mask, bool debug)
{
    std::vector<cv::Vec3f> points;
    if(plane.size()<2) return points;
    cv::Vec3f center = plane[0];
    cv::Vec3f normal = plane[1];
    if(normal[2] < 0) normal = -normal;
    output_img_mask = cv::Mat::zeros(cloud.rows, cloud.cols, CV_8UC1);
    for(size_t i=0; i<cloud.rows; i++)
        for(size_t j=0; j<cloud.cols; j++)
        {
            cv::Vec3f v = cloud.at<cv::Vec3f>(i,j) - center;
            float dist = normal[0]*v[0] + normal[1]*v[1] + normal[2]*v[2];
            if(dist > dist_threshold)
            {
                output_img_mask.at<char>(i,j) = 255;
                points.push_back(cloud.at<cv::Vec3f>(i,j));
            }
        }
    if(debug) cv::imshow("Points above plane mask", output_img_mask);
    if(points.size() < 10) std::cout << "ObjReco.->WARNING! Too few points above a plane. Not enough for object recognition" << std::endl;
    return points;
}

/*
 * Returns a plane given by a set of 2 xyz vectors: [center, normal].
 * Normal is returned as a unitary vector
 */
std::vector<cv::Vec3f> GeometricFeatures::plane_from_points(cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3)
{
    std::vector<cv::Vec3f> plane(2);
    plane[0] = (p1 + p2 + p3)/3; //Center point
    plane[1] = (p2 - p1).cross(p3 - p1); //Normal vector
    float mag = cv::norm(plane[1]);
    if(mag != 0) plane[1] /= mag;
    if(plane[1][2] < 0) plane[1] = -plane[1];
    return plane;
}

/*
 * Returns a plane given by a set of 6 xyz vectors: [center, normal and four bounding points].
 * Yes, it is assumed only a planar segment with a rectangular shape. Sorry for that limitation.
 */
std::vector<cv::Vec3f> GeometricFeatures::plane_by_pca(cv::Mat points, float& approx_area, bool debug)
{
    std::vector<cv::Vec3f> plane(6);
    cv::PCA pca(points, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if(debug)
    {
        std::cout << "ObjReco->Plane PCA: number of points: " << points.rows << std::endl;
        std::cout << "ObjReco->Plane PCA mean: " << pca.mean << std::endl;
        std::cout << "ObjReco->Plane PCA vectors: " << pca.eigenvectors << std::endl;
        std::cout << "ObjReco->Plane PCA eigenvalues: " << pca.eigenvalues << std::endl;
    }
    float width  = 2*sqrt(pca.eigenvalues.at<float>(0));
    float height = 2*sqrt(pca.eigenvalues.at<float>(1));
    plane[0] = pca.mean.at<cv::Vec3f>(0);
    plane[1] = pca.eigenvectors.at<cv::Vec3f>(0).cross(pca.eigenvectors.at<cv::Vec3f>(1));
    plane[2] = plane[0] + width*pca.eigenvectors.at<cv::Vec3f>(0) + height*pca.eigenvectors.at<cv::Vec3f>(1);
    plane[3] = plane[0] - width*pca.eigenvectors.at<cv::Vec3f>(0) + height*pca.eigenvectors.at<cv::Vec3f>(1);
    plane[4] = plane[0] - width*pca.eigenvectors.at<cv::Vec3f>(0) - height*pca.eigenvectors.at<cv::Vec3f>(1);
    plane[5] = plane[0] + width*pca.eigenvectors.at<cv::Vec3f>(0) - height*pca.eigenvectors.at<cv::Vec3f>(1);
    approx_area = 4*width*height;
    return plane;
}

/*
 * Returns the first found plane with more than 'min_area' inliers
 */
std::vector<cv::Vec3f> GeometricFeatures::plane_by_ransac(cv::Mat cloud, cv::Mat mask, float normal_min_z, float dist_threshold, float min_area,
                                                          std::vector<cv::Vec3f>& inliers, std::vector<cv::Vec3f>& outliers, bool debug)
{
    std::vector<cv::Point> indices;
    cv::findNonZero(mask, indices);
    cv::Mat points = cv::Mat(indices.size(), 3, CV_32F);
    for(int i=0; i<indices.size(); i++)
        points.at<cv::Vec3f>(i) = cloud.at<cv::Vec3f>(indices[i]);
    
    std::cout << "ObjReco-GeometricFeatures.->Executing ransac with " << points.rows << " points"  << std::endl;
    std::vector<cv::Vec3f> planes, plane;
    if(points.rows < 3) return plane;
    cv::RNG rng(time(NULL));
    //Get first three random points until get a horizontal candidate plane.
    cv::Vec3f p1, p2, p3;
    do{
        p1 = points.at<cv::Vec3f>(rng.next()%points.rows, rng.next()%points.cols);
        p2 = points.at<cv::Vec3f>(rng.next()%points.rows, rng.next()%points.cols);
        p3 = points.at<cv::Vec3f>(rng.next()%points.rows, rng.next()%points.cols);
        plane = GeometricFeatures::plane_from_points(p1, p2, p3);
    }while(plane[1][2] < normal_min_z);

    //Find inliers
    cv::Vec3f center = plane[0];
    cv::Vec3f normal = plane[1];
    inliers.clear();
    outliers.clear();
    if(debug) std::cout << "ObjReco-GeometricFeatures.->Checking candidate plane: " << center << "  " << normal << std::endl;
    for(size_t i=0; i < cloud.rows; i++)
        for(size_t j=0; j < cloud.cols; j++)
        {
            cv::Vec3f v = cloud.at<cv::Vec3f>(i,j) - center;
            float dist = fabs(normal[0]*v[0] + normal[1]*v[1] + normal[2]*v[2]);
            if (dist < dist_threshold)
                inliers.push_back(cloud.at<cv::Vec3f>(i,j));
            else
            {
                outliers.push_back(cloud.at<cv::Vec3f>(i,j));
                mask.at<char>(i,j) = 0;
            }
        }
    float approx_area;
    plane = GeometricFeatures::plane_by_pca(cv::Mat(inliers).reshape(1), approx_area, debug);
    if(debug)
    {
        std::cout << "ObjReco-GeometricFeatures.->Candidate plane had " << inliers.size() << " inliers and approx area: " << approx_area << std::endl;
        cv::imshow("Plane inliers", mask);
    }
    if(approx_area < min_area) plane.clear();
    return plane;
}

float GeometricFeatures::dist_point_to_line_segment(float px, float py, float pz, float x1, float y1, float z1, float x2, float y2, float z2)
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

std::vector<cv::Vec3f> GeometricFeatures::line_by_pca(cv::Mat& points)
{
    std::vector<cv::Vec3f> line(2);
    cv::PCA pca(points, cv::Mat(), cv::PCA::DATA_AS_ROW);
    float line_mag = 2*sqrt(pca.eigenvalues.at<float>(0));
    line[0][0] = pca.mean.at<float>(0,0) + line_mag*pca.eigenvectors.at<float>(0,0);
    line[0][1] = pca.mean.at<float>(0,1) + line_mag*pca.eigenvectors.at<float>(0,1);
    line[0][2] = pca.mean.at<float>(0,2) + line_mag*pca.eigenvectors.at<float>(0,2);
    line[1][0] = pca.mean.at<float>(0,0) - line_mag*pca.eigenvectors.at<float>(0,0);
    line[1][1] = pca.mean.at<float>(0,1) - line_mag*pca.eigenvectors.at<float>(0,1);
    line[1][2] = pca.mean.at<float>(0,2) - line_mag*pca.eigenvectors.at<float>(0,2);
    return line;
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
std::vector<cv::Vec3f> GeometricFeatures::hough_lines(cv::Mat img_bin, cv::Mat xyz, float d_min, float d_max, int d_step, float theta_min,
                                                      float theta_max, float theta_step, int threshold, cv::Mat& output_bgr, bool debug)
{
    /*
     * dist_n : number of values for quantized distance
     * theta_n: number of values for quantized angle
     */
    int dist_n  = (int)ceil((d_max - d_min)/d_step);
    int theta_n = (int)ceil((theta_max - theta_min)/theta_step);

    /*
     * pixel_acc : 2D array to store a list of pixel coordinates corresponding to each given vote. Each element in 2d-array is a vector of Vec2i.
     * sines     : Precalculated sine values for each quantized angle
     * cosines   : Precalculated cosines values for each quantized angle
     */
    std::vector<std::vector<std::vector<cv::Vec2i> > > pixel_acc(dist_n);
    for(size_t i=0; i < pixel_acc.size(); i++) pixel_acc[i].resize(theta_n);
    float sines[theta_n], cosines[theta_n];

    //Precalculate sine and cosine for each quantized angle
    for(size_t k=0; k<theta_n; k++, sines[k]=sin(theta_min+theta_step*k), cosines[k]=cos(theta_min+theta_step*k));

    //Loop over all pixels in the binary image
    for(size_t i=0; i < img_bin.rows; i++)      //For each non-zero pixel, calculate quantized distance for each quantized angle
        for(size_t j=0; j < img_bin.cols; j++)  //given a cartesian point (i,j) in image coordinates
            if(img_bin.at<unsigned char>(i,j) != 0)
                for(size_t k=0; k<theta_n; k++)
                {
                    int d = (int)((j*cosines[k] + i*sines[k] - d_min)/d_step);
                    if(d >= 0 && d < dist_n)
                        pixel_acc[d][k].push_back(cv::Vec2i(i,j));
                }

    std::vector<cv::Vec3f> lines;
    //Loop over all points in the Hough Space
    for(size_t i=0; i<dist_n; i++)         //IF A POINT IN HOUGH SPACE HAS A NUMBER OF VOTES GREATER THAN A GIVEN THRESHOLD
        for(size_t j=0; j<theta_n; j++)    //THEN, WE FOUND A LINE!
            if(pixel_acc[i][j].size() > threshold)
            {
                //lines_img.push_back(cv::Vec2f(i*d_step + d_min, j*theta_step + theta_min)); //Lines in image coordinates in the form rho-theta
                cv::Mat points = cv::Mat(pixel_acc[i][j].size(), 3, CV_32F);
                for(size_t k=0; k < pixel_acc[i][j].size(); k++)
                {
                    cv::Vec3f p = xyz.at<cv::Vec3f>(pixel_acc[i][j][k][0], pixel_acc[i][j][k][1]);
                    if(cv::norm(p) < 0.1) continue;
                    points.at<cv::Vec3f>(k) = p;
                }
                std::vector<cv::Vec3f> line = GeometricFeatures::line_by_pca(points);
                lines.push_back(line[0]);
                lines.push_back(line[1]);
                if(debug)
                {
                    cv::Point p1, p2;
                    double a = cos(j*theta_step + theta_min), b = sin(j*theta_step + theta_min);
                    double x0 = a*(i*d_step + d_min), y0 = b*(i*d_step + d_min);
                    p1.x = round(x0 + 1000*(-b));
                    p1.y = round(y0 + 1000*(a));
                    p2.x = round(x0 - 1000*(-b));
                    p2.y = round(y0 - 1000*(a));
                    cv::line(output_bgr, p1, p2, cv::Scalar(0,255,0), 2, cv::LINE_AA);
                }
            }
    if(debug)
        cv::imshow("Hough Lines", output_bgr);
    return lines;
}

std::vector<cv::Vec3f> GeometricFeatures::find_table_edge(cv::Mat& cloud, float normal_min_z, int canny_threshold1, int canny_threshold2,
                                                          int canny_window_size, int hough_min_rho, int hough_max_rho, int hough_step_rho,
                                                          float hough_min_theta, float hough_max_theta, float hough_step_theta, float hough_threshold,
                                                          cv::Mat& output_bgr, bool debug)
{
    cv::Mat grayscale_normals, borders;

    cv::Mat normals = GeometricFeatures::get_horizontal_normals(cloud, normal_min_z, debug);
    cv::cvtColor(normals, grayscale_normals, cv::COLOR_BGR2GRAY);
    grayscale_normals.convertTo(grayscale_normals, CV_8UC1, 255);
    cv::Canny(grayscale_normals, borders, canny_threshold1, canny_threshold2, canny_window_size);    
    std::vector<cv::Vec3f> lines = GeometricFeatures::hough_lines(borders, cloud,  hough_min_rho, hough_max_rho, hough_step_rho, hough_min_theta,
                                                                  hough_max_theta, hough_step_theta, hough_threshold, output_bgr, debug);
    float min_dist = 1000;
    std::vector<cv::Vec3f> nearest_line(2);
    for(int i=0; i<lines.size(); i+=2)
    {
        float d = GeometricFeatures::dist_point_to_line_segment(0, 0, 0, lines[i][0], lines[i][1], lines[i][2], lines[i+1][0], lines[i+1][1], lines[i+1][2]);
        if(d < min_dist)
        {
            min_dist = d;
            nearest_line[0] = lines[i];
            nearest_line[1] = lines[i+1];
        }
    }
    if(min_dist == 1000) nearest_line.clear();
    if(debug)
        cv::imshow("Canny borders", borders);
    return nearest_line;
}
