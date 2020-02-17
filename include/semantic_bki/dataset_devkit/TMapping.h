#ifndef __TMAPPING_H_
#define __TMAPPING_H_
 
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h> 

// cv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/calib3d.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>


#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>

// STD
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

// Mapping
#include "bkioctomap.h"
#include "markerarray_pub.h"

class TMapping {
    
    public:
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
      TMapping(ros::NodeHandle& nodeHandle);
      ~TMapping();
    private:
      void readParameters();

      //! ROS node handle.
      ros::NodeHandle& nodeHandle_;

      message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
      message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> *info_sub_;
      message_filters::Synchronizer<MySyncPolicy> *sync_;

      std::string mapTopic_;
      std::string depthTopic_;
      std::string imageTopic_;
      std::string cameraTopic_;
      std::string cameraBase_;
      std::string pointCloudTopic_;
      std::string velodyneTopic_;
      std::string labelDir_;

      //! TF listener.
      tf::TransformListener transformListener_;

      // Callbacks.
      void imagesCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::CameraInfoConstPtr & infomsg);

      // Mapping
      semantic_bki::SemanticBKIOctoMap* map_;
      semantic_bki::MarkerArrayPub* mapPub_;
      int block_depth;
      double sf2;
      double ell;
      float prior;
      float var_thresh;
      double free_thresh;
      double occupied_thresh;
      double resolution;
      int num_class;
      double free_resolution;
      double ds_resolution;
      int scan_num;
      double max_range;

      template<typename T>
      void convert_to_cloud(cv::Mat label_image,
                            const sensor_msgs::ImageConstPtr& depth_msg,
                            sensor_msgs::PointCloud2::Ptr& cloud_msg,
                            const image_geometry::PinholeCameraModel& model,
                            Eigen::Affine3d pose,
                            semantic_bki::point3f& origin,
                            double range_max = 0.0){
        // Use correct principal point from calibration
        float center_x = model.cx();
        float center_y = model.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
        float constant_x = unit_scaling / model.fx();
        float constant_y = unit_scaling / model.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_label(*cloud_msg, "label");
        const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
//        const T* label_row = reinterpret_cast<const T*>(&label_msg->data[0]);
        int row_step = depth_msg->step / sizeof(T);
        for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
        {
         for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_label)
         {
            if(v>(int)(cloud_msg->height/2)){
            T depth = depth_row[u];

            // Missing points denoted by NaNs
            if (!depth_image_proc::DepthTraits<T>::valid(depth))
            {
             if (range_max != 0.0)
             {
               depth = depth_image_proc::DepthTraits<T>::fromMeters(range_max);
             }
             else
             {
               *iter_x = *iter_y = *iter_z = bad_point;
               continue;
             }
            }

            // Fill in XYZ
            Eigen::Vector4d xyz((u - center_x) * depth * constant_x, (v - center_y) * depth * constant_y, depth_image_proc::DepthTraits<T>::toMeters(depth), 1.0);
            xyz = pose.matrix()*xyz;
            *iter_x = xyz(0);
            *iter_y = xyz(1);
            *iter_z = xyz(2);
            //           *iter_x = (u - center_x) * depth * constant_x;
            //           *iter_y = (v - center_y) * depth * constant_y;
            //           *iter_z = depth_image_proc::DepthTraits<T>::toMeters(depth);

            if(label_image.at<unsigned char>(v,u)>100) *iter_label = 1;
            else *iter_label = 2;
            //           *iter_label = 1.0;//label_image.at<unsigned char>(v,u);
            //           *iter_label = label_row[u];
            }
            }
            origin.x() = pose.matrix()(0,3);
            origin.y() = pose.matrix()(1,3);
            origin.z() = pose.matrix()(2,3);
            }
      }

      
};//class

#endif
