///*
// * TMapping.cpp
// */

#include "TMapping.h"


TMapping::TMapping(ros::NodeHandle& nodeHandle)
: nodeHandle_(nodeHandle)
{
    readParameters();

    map_ = new semantic_bki::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);

    mapPub_ = new semantic_bki::MarkerArrayPub(nodeHandle, mapTopic_, 0.1f);

    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nodeHandle_, imageTopic_, 50);
    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nodeHandle_, depthTopic_, 50);
    info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo> (nodeHandle_, cameraTopic_, 50);

    sync_ = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(500), *image_sub_, *depth_sub_, *info_sub_);
    sync_->registerCallback(boost::bind(&TMapping::imagesCallback, this, _1, _2, _3));

}

    TMapping::~TMapping()
    {
    }


    void TMapping::readParameters()
    {
        // Read parameters for image subscriber.
        nodeHandle_.param("map_topic", mapTopic_, std::string("/occupied_cells_vis_array"));
        nodeHandle_.param("image_topic", imageTopic_, std::string("/camera/color/image_raw"));
        nodeHandle_.param("depth_topic", depthTopic_, std::string("/camera/aligned_depth_to_color/image_raw"));
        nodeHandle_.param("camera_topic", cameraTopic_, std::string("/camera/color/camera_info"));
        nodeHandle_.param("camera_base", cameraBase_, std::string("camera_color_optical_frame"));
        nodeHandle_.param("pointcloud_topic", pointCloudTopic_, std::string("/velodyne_points"));
        nodeHandle_.param("velodyne_topic", velodyneTopic_, std::string("velodyne_actual"));
    
        nodeHandle_.param("label_data_dir", labelDir_, std::string("/home/youngji/workspace/catkin_ws_mapping/labels/"));

        block_depth = 4;
        sf2 = 1.0;
        ell = 1.0;
        prior = 1.0f;
        var_thresh = 1.0f;
        free_thresh = 0.3;
        occupied_thresh = 0.7;
        resolution = 1.0;
        num_class = 2;
        free_resolution = 100;
        ds_resolution = 0.1;
        scan_num = 100;
        max_range = 30.0;

        nodeHandle_.param<int>("block_depth", block_depth, block_depth);
        nodeHandle_.param<double>("sf2", sf2, sf2);
        nodeHandle_.param<double>("ell", ell, ell);
        nodeHandle_.param<float>("prior", prior, prior);
        nodeHandle_.param<float>("var_thresh", var_thresh, var_thresh);
        nodeHandle_.param<double>("free_thresh", free_thresh, free_thresh);
        nodeHandle_.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
        nodeHandle_.param<double>("resolution", resolution, resolution);
        nodeHandle_.param<int>("num_class", num_class, num_class);
        nodeHandle_.param<double>("free_resolution", free_resolution, free_resolution);
        nodeHandle_.param<double>("ds_resolution", ds_resolution, ds_resolution);
        nodeHandle_.param<int>("scan_num", scan_num, scan_num);
        nodeHandle_.param<double>("max_range", max_range, max_range);

    }

    void TMapping::imagesCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr & infomsg){

        //* process msgs *//
        
        //read label_img
        std::string label_img_name(labelDir_ + std::to_string(color_msg->header.stamp.toNSec()) + ".png.png");
    	cv::Mat label_img = cv::imread(label_img_name, CV_LOAD_IMAGE_GRAYSCALE);
        if (label_img.empty())return;
        

        // Get transform
        tf::StampedTransform transform;
        try {
            transformListener_.lookupTransform("/map", cameraBase_, color_msg->header.stamp, transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // depth & label img to pcl::PointCloud<pcl::PointXYZL>
        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        cloud_msg->header = color_msg->header;
        cloud_msg->height = color_msg->height;
        cloud_msg->width  = color_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;
        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "label", 1, sensor_msgs::PointField::UINT32);

        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(infomsg);
        Eigen::Affine3d pose;
        tf::transformTFToEigen (transform, pose);
        semantic_bki::point3f origin;
        convert_to_cloud<uint16_t>(label_img, depth_msg, cloud_msg, camModel, pose, origin);
        pcl::PointCloud<pcl::PointXYZL> input;
        pcl::fromROSMsg(*cloud_msg, input);

        //* mapping *//
        map_->insert_pointcloud(input, origin, resolution, free_resolution, max_range);

        ROS_INFO_STREAM("Map inserted ");

        mapPub_->clear_map(resolution);
        for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
          if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
            semantic_bki::point3f p = it.get_loc();
            mapPub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 1);
          }
        }
        
        ROS_INFO_STREAM("Map published ");
        mapPub_->publish();
        ROS_INFO_STREAM("Mapping finished ");

    }

