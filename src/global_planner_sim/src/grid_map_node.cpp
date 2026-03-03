#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/PoseStamped.h>
#include<cstdlib>
#include<ctime>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh;

    int grid_size= 100;
    double resolution= 0.05;

    ros::Publisher map_pub= nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::Publisher start_pub= nh.advertise<geometry_msgs::PoseStamped>("/start", 1, true);
    ros::Publisher goal_pub= nh.advertise<geometry_msgs::PoseStamped>("/goal",1,true);

    nav_msgs::OccupancyGrid map;

    map.header.frame_id = "map";
    map.info.resolution = resolution;
    map.info.width  = grid_size;
    map.info.height = grid_size;

    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data.resize(grid_size * grid_size, 0);

    srand(time(0));

    for (int i = 0;i < map.data.size();i++) 
  {
    if (rand() % 5 == 0)
        map.data[i] = 100;
  }

    map_pub.publish(map);
    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";

    int sx, sy; 
    do { 
        sx = rand() % grid_size; 
        sy = rand() % grid_size; 
    } 
    
    while (map.data[sy * grid_size + sx] != 0); 

    start.pose.position.x = (sx+0.5) * resolution; 
    start.pose.position.y = (sy+0.5) * resolution; 
    start.pose.orientation.w = 1.0; 
    start_pub.publish(start);

    geometry_msgs::PoseStamped goal; 
    goal.header.frame_id = "map"; 
    int gx, gy; 
    
    do 
    { 
        gx = rand() % grid_size; 
         gy = rand() % grid_size; 
    }
    
    while (map.data[gy * grid_size + gx] != 0); 
    goal.pose.position.x = (gx+0.5) * resolution;
    goal.pose.position.y = (gy+0.5) * resolution;
    goal.pose.orientation.w = 1.0;
    goal_pub.publish(goal); 
       
    ros::spin();
 }