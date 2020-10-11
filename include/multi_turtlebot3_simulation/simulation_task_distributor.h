#ifndef MULTI_TURTLEBOT3_SIMULATION_SIMULATION_TASK_DISTRIBUTOR
#define MULTI_TURTLEBOT3_SIMULATION_SIMULATION_TASK_DISTRIBUTOR

#include <ros/ros.h>
#include <ros/console.h>
#include <map>
#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace multi_turtlebot3_simulation
{

// Base interface
class NavigationAlgorithm
{
public:
    virtual ~NavigationAlgorithm() {}
    virtual std::multimap<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, const std::vector<geometry_msgs::PoseStamped> goals) = 0;
};

// Using strategy design pattern

// Strategy 1
class NavigationAlgorithmSimple : public NavigationAlgorithm
{
public:
    std::multimap<std::string, geometry_msgs::PoseStamped> run(std::vector<std::string> robots, 
                                                          const std::vector<geometry_msgs::PoseStamped> goals) override
    {
        std::multimap<std::string, geometry_msgs::PoseStamped> results;

        int loop_size = robots.size(); // hardcode
        for (int n = 0; n < loop_size; ++n)
        {

            // calculate for the first robot
            SimulationNavigator navigator(*robots.begin(), "base_footprint", "map", "move_base/NavfnROS/make_plan");

            double distance = 0;
            while (distance == 0) distance = navigator.getDistance(goals.at(n)); 
            std::string robot = robots.at(0);

            ROS_WARN("TaskDistributor run at Goal #%d", n);
            ROS_WARN("TaskDistributor initialize %s with distance %f", robot.c_str(), distance);

            for (std::vector<std::string>::const_iterator it_robot = robots.begin() + 1; it_robot != robots.end(); ++it_robot)
            {
                SimulationNavigator navigator(*it_robot, "base_footprint", "map", "move_base/NavfnROS/make_plan");

                double c_distance = 0;
                while (c_distance == 0) c_distance = navigator.getDistance(goals.at(n));
                ROS_WARN("TaskDistributor calculate %s with distance %f", (*it_robot).c_str(), c_distance);
                if (c_distance < distance)
                {
                    distance = c_distance;
                    robot = *it_robot;
                    ROS_WARN("TaskDistributor switch to %s with distance %f", robot.c_str(), distance);
                }
            }

            results.insert(std::pair<std::string, geometry_msgs::PoseStamped>(robot,goals.at(n)));
            robots.erase(std::remove(robots.begin(), robots.end(), robot), robots.end()); 
            ROS_WARN("TaskDistributor assaigned %s with Goal #%d", robot.c_str(), n);
        }
        return results;
    }
};

// Strategy 2
class NavigationAlgorithmExhastive : public NavigationAlgorithm
{
    std::multimap<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, 
                                                          const std::vector<geometry_msgs::PoseStamped> goals) override
    {
        

        return std::multimap<std::string, geometry_msgs::PoseStamped>();
    }
};

class SimulationTaskDistributor
{ 
    public: 
        SimulationTaskDistributor(NavigationAlgorithm *s_algorithm)
        {
            this->algorithm = s_algorithm;
        }

        void setAlgorithm(NavigationAlgorithm *s_algorithm)
        {
            delete this->algorithm;
            this->algorithm = s_algorithm;
        }
        
        std::multimap<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, 
                                                            const std::vector<geometry_msgs::PoseStamped> goals)
        {
            return this->algorithm->run(robots, goals);
        }

    private:
        NavigationAlgorithm *algorithm;
};

}

#endif