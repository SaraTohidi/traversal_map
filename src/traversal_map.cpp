#include <list>
#include <chrono>
#include <limits>
#include <math.h>
#include <iostream>
#include <vector>
#include <tf/tf.h>
#include "ros/ros.h"
#include <algorithm>
#include <opencv/cv.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <traversal_map/traversal_map.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

PLUGINLIB_EXPORT_CLASS(traversal_map::TraversalArea, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using namespace std;
using namespace pcl;

namespace traversal_map{

TraversalArea::TraversalArea() : nh_("~/")
{
    initializeTraversableData();
}

void TraversalArea::initializeTraversableData()
{
    //Setting config
    nh_.param("threshold", threshold_, 2.5);
    nh_.param("map_lenght_x", map_lenght_x_, 6.5);
    nh_.param("map_lenght_y", map_lenght_y_, 6.5);
    nh_.param("map_resolution", octomap_resolution_, 0.1);
    nh_.param<std::string>("base_frame", base_frame_, "/odom");
    nh_.param("x", lenght_x_, 1.0);
    nh_.param("y", lenght_y_, 1.0);
    nh_.param<std::string>("point_cloud", point_cloud_, "/octomap_point_cloud_centers");

    dsrv_ = new dynamic_reconfigure::Server<traversal_map::TraversalAreaConfig>(nh_);
    dynamic_reconfigure::Server<traversal_map::TraversalAreaConfig>::CallbackType tc =
            boost::bind(&TraversalArea::reconfigureCBTAC, this, _1, _2);
    dsrv_->setCallback(tc);

    //Subscribe topics
    octomap_sub_ = nh_.subscribe<boost::shared_ptr<sensor_msgs::PointCloud2> >(point_cloud_ , 1, &TraversalArea::octomapCallBack, this);

    //Publishing the Occupancy Grid
    occipancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/nav_msgs/OccupancyGrid" , 10);

    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    ros::waitForShutdown();
}

///////////////////////////
/// \brief TraversalArea::onInitialize
/// \param config
/// \param level
///
void TraversalArea::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_c_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &TraversalArea::reconfigureCB, this, _1, _2);
    dsrv_c_->setCallback(cb);
}

void TraversalArea::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void TraversalArea::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void TraversalArea::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                 double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;

    double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
    }

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
}

void TraversalArea::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                int max_j)
{
    if (!enabled_)
        return;

    for (int j = min_j; j < max_j; j++)
    {
        for (int i = min_i; i < max_i; i++)
        {
            int index = getIndex(i, j);
            if (costmap_[index] == NO_INFORMATION)
                continue;
            master_grid.setCost(i, j, costmap_[index]);
        }
    }
}



/////////////////////////

void TraversalArea::reconfigureCBTAC(traversal_map::TraversalAreaConfig &config, uint32_t level)
{
    //Giving the config data to parameters
    threshold_ = config.threshold;
    map_lenght_x_ = config.map_lenght_x;
    map_lenght_y_ = config.map_lenght_y;
    octomap_resolution_ = config.map_resolution;
    base_frame_ = config.base_frame;
    lenght_x_ = config.x;
    lenght_y_ = config.y;
}
///////////////////////////
void TraversalArea::octomapCallBack(const boost::shared_ptr<sensor_msgs::PointCloud2> octomap_data)
{
    ROS_ERROR_ONCE("First Cloud Received: width = %d, height = %d\n ", octomap_data.get()->width, octomap_data.get()->height);
    auto start = std::chrono::steady_clock::now();

    //Converting PointCloud2d to PointCloud with x, y, and z coordinates
    PointCloud<pcl::PointXYZ> points;
    fromROSMsg(*octomap_data, points);

    int lenght_x, lenght_y;
    const vector<vector<double> > matrix = initializeTraversalMap(points, lenght_x, lenght_y);

    searchingNodesByBFS(matrix, lenght_x, lenght_y);
    ROS_WARN_STREAM("searchingNodesByBFS elapsed time : " << getElapsedTime(start, true));

    //ROS_INFO("Publishing occupancy grid");
    publishOccupancyGrid(occupancy_grid_);
}

const vector<vector<double> > TraversalArea::initializeTraversalMap(const PointCloud<PointXYZ> octopoints_data, int &lenght_x, int & lenght_y)
{
    //Giving the length of the map frame to parametes
    double lenght_size_x = (map_lenght_x_*2);
    double lenght_size_y = (map_lenght_y_*2);
    double resolution = octomap_resolution_;

    //Creating a lenght of matrix by resolution of the Octomap
    lenght_x = lenght_size_x/resolution;
    lenght_y = lenght_size_y/resolution;

    //Creating a matrix size by multiply length of x and y coordinates
    int matrix_size = (lenght_x * lenght_y) ;

    vector<vector<double> > matrix;
    vector<vector<int> > vector_count;

    //Initialize a vector by NAN value and size of the matrix
    vector<double> vec((matrix_size),NAN);
    vector<int> vec_2((matrix_size),0);

    //Converting vector to a matrix
    for (int i = 0 ; i < matrix_size; i = i + lenght_x)
    {
        vector_count.push_back(std::vector<int>(&vec_2[i], &vec_2[i + lenght_x]));
        matrix.push_back(std::vector<double>(&vec[i], &vec[i + lenght_x]));
    }

    int x,y;
    int center = matrix.size()/2;

    //Converting points of octomap to matrix
    for(int i = 0; i < octopoints_data.size(); i++)
    {
        Point points;
        points.x = octopoints_data.at(i).x;
        points.y = octopoints_data.at(i).y;
        points.z = octopoints_data.at(i).z;

        x = center - int((points.x /resolution));
        y = center - int((points.y /resolution));

        if(isnan(matrix.at(x).at(y)))
        {
            vector_count.at(x).at(y) = 1;
            matrix.at(x).at(y) = (points.z /resolution);
        }
        else
        {
            matrix.at(x).at(y)  = ( ( (matrix.at(x).at(y) ) * vector_count.at(x).at(y) ) + ( (points.z / resolution) ) );
            vector_count.at(x).at(y) = vector_count.at(x).at(y) + 1;
            matrix.at(x).at(y) = matrix.at(x).at(y) / vector_count.at(x).at(y);
        }
    }

    //creating occupancy grid
    occupancy_grid_.info.map_load_time= ros::Time::now();
    occupancy_grid_.info.width = lenght_x;
    occupancy_grid_.info.height = lenght_y;
    occupancy_grid_.info.origin.position.x = -lenght_size_x/2;
    occupancy_grid_.info.origin.position.y = -lenght_size_y/2;
    occupancy_grid_.info.origin.position.z = 0;
    occupancy_grid_.info.origin.orientation.x = 0;
    occupancy_grid_.info.origin.orientation.y = 0;
    occupancy_grid_.info.origin.orientation.z = 0;
    occupancy_grid_.info.resolution = resolution;
    occupancy_grid_.data.resize(matrix_size);

    return matrix;
}

double TraversalArea::computingTraversalElevation(Node child, Node parent)
{
    double elevation,dis,value;

    if(!isnan(parent.elevation))
    {
        elevation = abs(child.elevation - parent.elevation);
        int diff_x = (child.id_x - parent.id_x);
        int diff_y = (child.id_y - parent.id_y);
        dis = abs(sqrt( (pow(diff_x,2) ) + (pow(diff_y,2) ) ) );
        value = ((elevation) / (dis) );
    }
    else
    {
        //If a parent has nan value give the child value to be able to compute
        parent.elevation = child.elevation;
        elevation = abs(child.elevation - parent.elevation);
        int diff_x = (child.id_x - parent.id_x);
        int diff_y = (child.id_y - parent.id_y);
        dis = abs(sqrt( (pow(diff_x,2) ) + (pow(diff_y,2) ) ) );
        value = ((elevation) / (dis) );
    }
    ROS_DEBUG_STREAM("Value " << value);

    return value;
}

void TraversalArea::publishOccupancyGrid(nav_msgs::OccupancyGrid occupancy_grid)
{
    occupancy_grid.header.frame_id = base_frame_;

    //publishing occupancy grid
    occipancy_grid_pub_.publish(occupancy_grid);
}

bool TraversalArea::checkForValidationRegion(const int index_i,const int index_j,const int lenght_x,const int lenght_y)
{
    //Check if they are in rigion of the map or not
    if ( (index_i > -1 ) && (index_j > -1) )
    {
        if ( (index_i < lenght_x) && ( index_j < lenght_y) )
        {
            return true;
        }
        else
            return false;
    }
    else
    {
        return false;
    }

}

bool TraversalArea::validateValue(const double value)
{
    //Check if the value is acceptable as traversal or not
    if(value < threshold_)
        return true;
    else
        return false;
}

inline bool TraversalArea::checkVisitedNodes(const vector<Visited>& visited_queue, const Node &child)
{
    Visited node;

    //Check if the node has been visited or not
    for(int i = 0 ; i < visited_queue.size(); i++)
    {
        node = visited_queue.at(i);
        if(node.parent_x_id == child.id_x && node.parent_y_id == child.id_y)
            return true;
    }
    return false;

}

bool TraversalArea::setFreeFootprintRegion(const Node child, const int center)
{
    /** @todo Change the value of the footprint region to fit your robot footprint */
    //Check if the node has in the rigion of the robot's footprint or not
    if((child.id_x <(center + (int(lenght_x_/2))+1)) && (child.id_x > (center - (int(lenght_x_/2))-1))
            && (child.id_y < (center + (int(lenght_y_/2))+1)) && (child.id_y > (center - (int(lenght_y_/2)-1)) ) )
        return true;
    else
        return false;
}

void TraversalArea::searchingNodesByBFS(const vector<vector<double> > matrix,const int lenght_x,const int lenght_y)
{
    std::chrono::steady_clock::time_point totalTime = std::chrono::steady_clock::now();
    double value;

    //Setting the center of the matrix which has to have a same value for both i and j index
    const int center = ((matrix.size()) / 2);

    Node root;
    root.elevation = matrix.at(center).at(center);
    root.id_x = center;
    root.id_y = center;

    Node child;
    Node parent;
    vector<Visited> visited_queue;
    Visited visited;

    vector<Node> queue;
    queue.push_back(root);
    parent = queue.at(0);
    visited.parent_x_id = parent.id_x;
    visited.parent_y_id = parent.id_y;
    visited_queue.push_back(visited);

    int count = 0;
    //The Breadth First algorithm

    double checkVisitedNodesReslutElasped = 0;
    double checkForValidationRegionResultElasped = 0;
    double setFreeFootprintRegionResultElasped = 0;
    double computingTraversalElevationElasped = 0;
    double validateValueElasped = 0;
    while(!queue.empty())
    {

        ++count;

        for(int i = -1 ; i < 2; i++)
        {
            for(int j = -1; j < 2; j++)
            {
                child.id_x = parent.id_x + i;
                child.id_y = parent.id_y + j;
                std::chrono::steady_clock::time_point startFunction = std::chrono::steady_clock::now();

                bool checkVisitedNodesReslut = checkVisitedNodes(visited_queue, child);
                checkVisitedNodesReslutElasped += getElapsedTime(startFunction, true);
                if(!checkVisitedNodesReslut)
                {
                    startFunction = std::chrono::steady_clock::now();
                    bool checkForValidationRegionResult = checkForValidationRegion(child.id_x, child.id_y, lenght_x, lenght_y);
                    checkForValidationRegionResultElasped += getElapsedTime(startFunction, true);

                    if( checkForValidationRegionResult )
                    {
                        child.elevation = matrix.at(child.id_x).at(child.id_y);

                        //Set indexes as they've been seen
                        visited.parent_x_id = child.id_x;
                        visited.parent_y_id = child.id_y;
                        visited_queue.push_back(visited);

                        startFunction = std::chrono::steady_clock::now();
                        bool setFreeFootprintRegionResult = setFreeFootprintRegion(child, center);
                        setFreeFootprintRegionResultElasped += getElapsedTime(startFunction, true);

                        if(setFreeFootprintRegionResult)
                        {
                            queue.push_back(child);

                            //occupancy_grid is 0 -TRAVERESABLE AREA
                            setOccupancyValue(lenght_x, child, FREE, center);
                        }
                        else
                        {
                            //for calculation do these steps
                            if(isnan(matrix.at(child.id_x).at(child.id_y)))
                            {
                                //occupancy_grid is -1 -NOT SEEN
                                setOccupancyValue(lenght_x, child, UNKNOWN, center);
                            }
                            else
                            {
                                //ready to compute
                                startFunction = std::chrono::steady_clock::now();
                                value = computingTraversalElevation(child, parent);
                                computingTraversalElevationElasped += getElapsedTime(startFunction, true);

                                startFunction = std::chrono::steady_clock::now();
                                if(validateValue(value))
                                {
                                    queue.push_back(child);

                                    //occupancy_grid is 0 -TRAVERESABLE AREA
                                    setOccupancyValue(lenght_x, child, FREE, center);
                                }
                                else
                                {
                                    //occupancy_grid is 100 -OBSTACLE AREA
                                    setOccupancyValue(lenght_x, child, OBSTACLE, center);
                                }
                                validateValueElasped += getElapsedTime(startFunction, true);


                            }
                        }
                    }
                }
            }
        }
        queue.erase(queue.begin());
        if(!queue.empty())
            parent = queue.at(0);
    }

}

void TraversalArea::setOccupancyValue(const int lenght_x,const Node child,const int cost,const int center)
{
    int index_x =0,index_y=0;

    //Do these step to find out the converted index for occupancy grid
    if(child.id_x == center)
        index_x = child.id_x;
    if(child.id_y == center)
        index_y = child.id_y;

    if(child.id_x > center)
    {
        index_x = child.id_x - center;
        index_x = center - index_x;
    }
    if(child.id_y > center)
    {
        index_y = child.id_y - center;
        index_y = center - index_y;
    }
    if(child.id_x < center)
    {
        index_x = center - child.id_x;
        index_x = center + index_x;
    }
    if(child.id_y < center)
    {
        index_y = center - child.id_y;
        index_y = center + index_y;
    }

    if(cost == 1)
    {
        occupancy_grid_.data.at((index_y * lenght_x) + index_x) = costmap_2d::NO_INFORMATION;
    }
    if(cost == 2)
    {
        occupancy_grid_.data.at((index_y * lenght_x) + index_x) = costmap_2d::FREE_SPACE;
    }
    if(cost == 3)
    {
        occupancy_grid_.data.at((index_y * lenght_x) + index_x) = costmap_2d::LETHAL_OBSTACLE;
    }


}

TraversalArea::~TraversalArea(){ }

} // end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traversable_map_computer");

    traversal_map::TraversalArea computing_traversable_map ;

    return 0;
}
