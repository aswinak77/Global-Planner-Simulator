#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>

nav_msgs::OccupancyGrid map_data;
geometry_msgs::PoseStamped start_pose, goal_pose;

bool map_received = false;
bool start_received = false;
bool goal_received = false;

int width, height;
double resolution;

int toIndex(int x, int y)
{
    return y * width + x;
}

bool isFree(int x, int y)
{
    return map_data.data[toIndex(x, y)] == 0;
}

double heuristic(int x1, int y1, int x2, int y2)
{
    return hypot(x1 - x2, y1 - y2);
}

struct Node
{
    int x, y;
    double cost;
};

struct Compare
{
    bool operator()(Node a, Node b)
    {
        return a.cost > b.cost;
    }
};

nav_msgs::Path runAStar()
{
    nav_msgs::Path path;
    path.header.frame_id = "map";

    int sx = start_pose.pose.position.x / resolution;
    int sy = start_pose.pose.position.y / resolution;

    int gx = goal_pose.pose.position.x / resolution;
    int gy = goal_pose.pose.position.y / resolution;

    std::priority_queue<Node, std::vector<Node>, Compare> open;

    std::unordered_map<int, int> parent;
    std::unordered_map<int, double> g_cost;

    open.push({sx, sy, 0});
    g_cost[toIndex(sx, sy)] = 0;

    int dx[8] = {1, -1, 0, 0, 1 , 1, -1, -1};
    int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty())
    {
        Node current = open.top();
        open.pop();

        if (current.x == gx && current.y == gy)
            break;

        for (int i = 0; i < 8; i++)
        {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                continue;

            if (!isFree(nx, ny))
                continue;

            if (dx[i] != 0 && dy[i] != 0)
            {
            if (!isFree(current.x + dx[i], current.y) ||
            !isFree(current.x, current.y + dy[i]))
            continue;
            }


            double move_cost = (dx[i] == 0 || dy[i] == 0) ? 1.0 : std::sqrt(2.0);
            double new_cost = g_cost[toIndex(current.x, current.y)] + move_cost;

            if (!g_cost.count(toIndex(nx, ny)) || new_cost < g_cost[toIndex(nx, ny)])
            {
                g_cost[toIndex(nx, ny)] = new_cost;

                double f = new_cost + heuristic(nx, ny, gx, gy);

                open.push({nx, ny, f});
                parent[toIndex(nx, ny)] = toIndex(current.x, current.y);
            }
        }
    }

    int current = toIndex(gx, gy);

    while (parent.count(current))
    {
        int x = current % width;
        int y = current / width;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x * resolution;
        pose.pose.position.y = y * resolution;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);

        current = parent[current];
    }

    return path;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data = *msg;
    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    map_received = true;
}

void startCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    start_pose = *msg;
    start_received = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose = *msg;
    goal_received = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber start_sub = nh.subscribe("/start", 1, startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/goal", 1, goalCallback);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1, true);

    ros::Rate rate(1);

    while (ros::ok())
    {
        ros::spinOnce();

        if (map_received && start_received && goal_received)
        {
            nav_msgs::Path path = runAStar();
            path_pub.publish(path);
        }

        rate.sleep();
    }
}