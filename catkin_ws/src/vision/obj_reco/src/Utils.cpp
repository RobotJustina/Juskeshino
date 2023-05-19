#include "Utils.h"

bool  Utils::debug = false;

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
    if(Utils::debug) std::cout <<"ObjReco.->Point cloud frame: " << cloud.header.frame_id << std::endl;
    if(cloud.header.frame_id != "base_link")
    {
        if(Utils::debug) std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", cloud, cloud, *tf_listener);
    }
    Utils::pointcloud_msg2_cv_mat(cloud, bgr_dest, cloud_dest);
}


void Utils::filter_by_distance(cv::Mat& cloud, cv::Mat& img, float min_x, float min_y, float min_z, float max_x,
                                   float max_y, float max_z, cv::Mat& filtered_cloud, cv::Mat& filtered_img)
{
    // This function is intended to keep point only in a given bounding box, e.g., to remove floor and distant walls
    // The function DOES NOT return a smaller point cloud. It returns a cloud with all non valid points set to zero. 
    cv::Mat valid_points;
    cv::inRange(cloud, cv::Scalar(min_x, min_y, min_z),
                cv::Scalar(max_x, max_y, max_z), valid_points);
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


visualization_msgs::Marker Utils::get_lines_marker(std::vector<cv::Vec3f> lines)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obj_reco_markers";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.9;
    marker.lifetime = ros::Duration(10.0);
    for(size_t i=0; i < lines.size(); i++)
    {
        geometry_msgs::Point msg_p;
        msg_p.x = lines[i][0];
        msg_p.y = lines[i][1];
        msg_p.z = lines[i][2];
        marker.points.push_back(msg_p);
    }
    return marker;
}

std::vector<geometry_msgs::Point> Utils::get_lines_msg(std::vector<cv::Vec3f> line)
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

visualization_msgs::Marker Utils::get_plane_marker(std::vector<cv::Vec3f> plane)
{
    //Intended only for horizontal planes
    visualization_msgs::Marker marker;
    if(plane.size() < 6) return marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "obj_reco_markers";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = plane[0][0];
    marker.pose.position.y = plane[0][1];
    marker.pose.position.z = plane[0][2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = fabs(plane[2][0] - plane[4][0]);
    marker.scale.y = fabs(plane[2][1] - plane[4][1]);
    marker.scale.z = 0.1;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(10.0);
    return marker;
}

vision_msgs::RecognizeObjects::Response Utils::get_recog_objects_response(std::vector<cv::Mat>& objects_bgr, std::vector<cv::Mat>& objects_xyz,
                                                                          std::vector<cv::Mat>& objects_masks, std::vector<std::string>& labels,
                                                                          std::vector<double>& confidences, cv::Mat& result_img, std::string frame_id)
{
    vision_msgs::RecognizeObjects::Response resp;
    if(objects_bgr.size() < 1) return resp;
    
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = ros::Time::now();
    cv_img.image = result_img;
    cv_img.toImageMsg(resp.image);

    resp.recog_objects.resize(objects_bgr.size());
    for(size_t i=0; i < objects_bgr.size(); i++)
        resp.recog_objects[i] = Utils::get_vision_object_msg(objects_bgr[i], objects_xyz[i], objects_masks[i], labels[i], confidences[i], frame_id);
    
    return resp;    
}

vision_msgs::VisionObject Utils::get_vision_object_msg(cv::Mat& obj_bgr, cv::Mat& obj_xyz, cv::Mat& obj_mask, std::string label,
                                                       double confidence, std::string frame_id)
{
    vision_msgs::VisionObject msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.id = label;
    msg.confidence = confidence;

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = ros::Time::now();
    cv_img.image = obj_bgr;
    cv_img.toImageMsg(msg.image);

    cv::Scalar p = cv::mean(obj_xyz, obj_mask);
    msg.pose.position.x = p[0];
    msg.pose.position.y = p[1];
    msg.pose.position.z = p[2];
    msg.pose.orientation.w = 1.0;

    cv::Mat obj_x_y_z[3];
    double min_x, min_y, min_z, max_x, max_y, max_z;
    int min_idx, max_idx;
    cv::split(obj_xyz, obj_x_y_z);
    cv::minMaxIdx(obj_x_y_z[0], &min_x, &max_x, &min_idx, &max_idx, obj_mask);
    cv::minMaxIdx(obj_x_y_z[1], &min_y, &max_y, &min_idx, &max_idx, obj_mask);
    cv::minMaxIdx(obj_x_y_z[2], &min_z, &max_z, &min_idx, &max_idx, obj_mask);
    msg.size.x = max_x - min_x;
    msg.size.y = max_y - min_y;
    msg.size.z = max_z - min_z;

    cv::Scalar c = cv::mean(obj_bgr, obj_mask);
    msg.color_rgba.r = c[2];
    msg.color_rgba.g = c[1];
    msg.color_rgba.b = c[0];
    msg.color_rgba.a = 1.0;
    
    msg.graspable = true;
    return msg;
}

visualization_msgs::MarkerArray Utils::get_objects_markers(std::vector<vision_msgs::VisionObject>& objs)
{
    visualization_msgs::MarkerArray msg;
    std::vector<visualization_msgs::Marker> markers;
    markers.resize(objs.size());
    for(size_t i=0; i<objs.size(); i++)
    {
        markers[i].header.frame_id = objs[i].header.frame_id;
        markers[i].header.stamp = ros::Time();
        markers[i].ns = "obj_reco_markers";
        markers[i].id = i;
        markers[i].type = visualization_msgs::Marker::CUBE;
        markers[i].action = visualization_msgs::Marker::ADD;
        markers[i].pose = objs[i].pose;
        markers[i].scale = objs[i].size;
        markers[i].color = objs[i].color_rgba;
        markers[i].color.a = 0.6;
        markers[i].lifetime = ros::Duration(10.0);
    }
    msg.markers = markers;
    return msg;
}
