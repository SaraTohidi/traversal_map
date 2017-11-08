#ifndef TRAVERSAL_LAYER_H_
#define TRAVERSAL_LAYER_H_

#include <chrono>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <costmap_2d/layer.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose2D.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <traversal_map/TraversalAreaConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace geometry_msgs;
using namespace pcl;


namespace traversal_map {

enum OccuapancyGridValue{
    OBSTACLE=3,
    FREE=2,
    UNKNOWN=1
};

//* Visited
/**
* The basic class description, it does:
* 1. Creating a description of visited node
* which has 2 value X and Y
*
*/

struct Visited
{
    Visited(){}
    int parent_x_id;
    int parent_y_id;
};

//* Node
/**
* The basic class description, it does:
* 1. Creating a discription of every point from the cleaned input data as a node
* which has values as X,Y and Z or elevation
*
*/

struct Node
{
    Node() {}
    int id_x;
    int id_y;
    double elevation;
};

//* TraversableComputer
/**
* The basic class description, it does:
* 1. Creating an elevation of pointcloud data from octomap data
* 2. Computing the traversable area by Breadth-First-Traversal algorithm
* 3. Publishing an accupancy grid of traversable area
*
*/


class TraversalArea : public costmap_2d::Layer, public costmap_2d::Costmap2D
{

public:

    TraversalArea();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                               double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    bool isDiscretized()
    {
      return true;
    }

    virtual void matchSize();

    /*!
    * \brief Initialize the datas for Class.
    *
    * Set the subscribers and poblishers
    */
    void initializeTraversableData();
    /*!
    * \brief reconfigure function gives parameter a pre-defined value.
    */
    void reconfigureCBTAC(traversal_map::TraversalAreaConfig &config , uint32_t level);
    /*!
    * \brief Called when subscribed any data from the Map frame.
    *
    * Get the origin information data of the map frame
    * \param map_data       This is a data of the map which has been subscribed
    */
    void mapCallBack(nav_msgs::OccupancyGrid map_data);
    /*!
    * \brief This function convert pointcloud2 to pointcloud data.
    *
    * Convert the octomap data to pointcloud data to be ready for computation of the elevation creator
    * \param octomap_data       This is a pointcloud data which comes from octomap topic
    */
    void octomapCallBack(const boost::shared_ptr<sensor_msgs::PointCloud2> octomap_data);
    /*!
    * \brief Initialize the primary data for computing traversable area.
    *
    * Create a matrix of input points and initialize the occupancy grid and publish the occupancy grid.
    * \param elevation_points       This is a vector of points with singular elevation
    */
    const vector<vector<double> > initializeTraversalMap(const PointCloud<PointXYZ> octopoints_data, int &lenght_x, int & lenght_y);
    /*!
    * \brief Compute a traversable or none traversable area value.
    *
    * Compute to get a value by following an algorithm even if there be a NAN value for the parent.
    * \param child       This is a child node which is around a parent
    * \param parent       This is a parent node which has a childeren around itself
    */
    double computingTraversalElevation(Node child, Node parent);
    /*!
    * \brief Gives the boolean for checking if nodes are in region.
    *
    * Check if every nodes has a valid indexs and sitting in regions.
    * \param index_i , index_j       These are the indexes of every node
    * \param lenght_x , lenght_y       These are the lenghts of regions
    */
    bool checkForValidationRegion(int index_i, int index_j, int lenght_x, int lenght_y);
    /*!
    * \brief Gives a boolean if an input is valid.
    *
    * Check if the input value is in the threshold or not.
    * \param value       This is a value which traversalComputing gives
    */
    bool validateValue(const double value);
    /*!
    * \brief Gives the boolean if there is any repetitious node.
    *
    * Check if the targer node has been visited or not.
    * \param visited_queue       This is vector of nodes which have been visited
    * \param child       This is a node that will be check
    */
    bool checkVisitedNodes(const vector<Visited>& visited_queue, const Node &child);
    /*!
    * \brief Searching by one algorithm to find out traversable area.
    *
    * Do the Breadth-First-Traversal algorithm to find out which places are traversable.
    * \param matrix       This is a matrix which created befor by pointcloud
    * \param lenght_x, lenght_x       This is a size of the regions
    */
    void searchingNodesByBFS(const vector<vector<double> > matrix, const int lenght_x, const int lenght_y);
    /*!
    * \brief Set the values to occupancy grid.
    *
    * Set one of these values "NO_INFORMATION, FREE_SPACE, LETHAL_OBSTACLE" to occupancy grid by cool param "what".
    * \param lenght_x       This is a size of the region
    * \param child       This is a node of the tree
    * \param what       This is a cool signe for determining different places
    * \param center       This is a center index of the matrix
    */
    void setOccupancyValue(const int lenght_x,const Node child,const int cost,const int center);
    /*!
    * \brief Set all of the footprint region of the robot as free space.//optional
    *
    * Set all of the childeren around the center with raduis as footprint free for occupancy grid.
    * \param child       This is a child node
    * \param center       This is a center of the matrix
    */
    bool setFreeFootprintRegion(const Node child, const int center);
    /*!
    * \brief TPublish the occupancy grid.
    *
    * Publish the occupancy grid by giving it a frame id.
    * \param occupancy_grid       This is a occupancy grid which has been initialized before
    */
    void publishOccupancyGrid(nav_msgs::OccupancyGrid occupancy_grid);

    ~TraversalArea();

    ros::NodeHandle nh_;
    nav_msgs::OccupancyGrid occupancy_grid_;

    double map_lenght_x_,map_lenght_y_,octomap_resolution_;
    double lenght_x_, lenght_y_;
    double_t threshold_;
    string base_frame_, point_cloud_;

private:

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    double mark_x_, mark_y_;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_c_;

    double getElapsedTime(std::chrono::steady_clock::time_point& startRefrence, const bool resetTimer = false) {
//        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - startRefrence;
//        std::chrono::duration<std::chrono::milliseconds> elapsed = std::chrono::high_resolution_clock::now() - startRefrence;
//        std::chrono::duration<std::chrono::milliseconds> elapsed = std::chrono::steady_clock::now() - startRefrence;
//        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - std::chrono::steady_clock::now());
//        if (resetTimer)
//        {
//            startRefrence = std::chrono::high_resolution_clock::now();
//        }
//        return elapsed.count();

//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        double elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - startRefrence).count();
        if (resetTimer)
        {
            startRefrence = std::chrono::steady_clock::now();
        }
        return elapsedTime;
    }


    //ros stuff
    ros::Subscriber map_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher occipancy_grid_pub_;
    ros::Subscriber pointcloud_sub_;
    ros::Time begin;
    ros::Time end;
    ros::Duration duration;
    //dynamic reconfigure stuff
    dynamic_reconfigure::Server<traversal_map::TraversalAreaConfig> *dsrv_; /**< A dynamic reconfigure(it generate every pre-defined). */

};

}

#endif
