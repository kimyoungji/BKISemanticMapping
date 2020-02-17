#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <string>

namespace semantic_bki {

    double interpolate( double val, double y0, double x0, double y1, double x1 );
    double base( double val );
    double red( double gray );
    double green( double gray );
    double blue( double gray );
    std_msgs::ColorRGBA JetMapColor(float gray);
    std_msgs::ColorRGBA SemanticMapColor(int c);
    std_msgs::ColorRGBA SemanticKITTISemanticMapColor(int c);
    std_msgs::ColorRGBA NCLTSemanticMapColor(int c);
    std_msgs::ColorRGBA KITTISemanticMapColor(int c);
    std_msgs::ColorRGBA heightMapColor(double h);

    class MarkerArrayPub {
        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType> PointCloud;
    public:
        MarkerArrayPub(ros::NodeHandle nh, std::string topic, float resolution);

        void insert_point3d(float x, float y, float z, float min_z, float max_z, float size);

        void clear_map(float size);

        void insert_point3d_semantics(float x, float y, float z, float size, int c, int dataset);

        void insert_point3d_variance(float x, float y, float z, float min_v, float max_v, float size, float var);

        void insert_point3d(float x, float y, float z, float min_z, float max_z);

        void insert_point3d(float x, float y, float z);

        void insert_color_point3d(float x, float y, float z, double min_v, double max_v, double v);

        void clear();

        void publish() const;

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        visualization_msgs::MarkerArray::Ptr msg;
        std::string markerarray_frame_id;
        std::string topic;
        float resolution;
    };

}
