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
    virtual std::map<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, const std::vector<geometry_msgs::PoseStamped> goals) = 0;
};

// Using strategy design pattern

// Strategy 1
class NavigationAlgorithmSimple : public NavigationAlgorithm
{
public:
    std::map<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, 
                                                          const std::vector<geometry_msgs::PoseStamped> goals) override
    {
        std::map<std::string, geometry_msgs::PoseStamped> results;

        int loop_size = robots.size(); // hardcode
        for (n = 0; n < loop_size; ++n)
        {
            // calculate for the first robot
            double distance = navigator(*robots.begin(), "base_footprint", "map", "move_base/NavfnROS/make_plan").getDistance(goals.at(n)); 
            std::string robot = robots.at(0);

            for (std::vector<td::string>::iterator it_robot = robots.begin() + 1 ; it_robot != robots.end(); ++it_robot)
            {
                SimulationNavigator navigator(*it_robot, "base_footprint", "map", "move_base/NavfnROS/make_plan");
                c_distance = navigator.getDistance(goals.at(n));

                if (c_distance < distance)
                {
                    distance = c_distance;
                    robot = *it_robot;
                }
            }

            results[robot] = goals.at(n);
        }
        return result;
    }
};

// Strategy 2
class NavigationAlgorithmExhastive : public NavigationAlgorithm
{
    std::map<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, 
                                                          const std::vector<geometry_msgs::PoseStamped> goals) override
    {
        return std::map<std::string, geometry_msgs::PoseStamped>();
    }
};

class SimulationTaskDistributor
{ 
    public: 
        SimulationTaskDistributor(NavigationAlgorithm *s_algorithm)
        {
            this->algorithm = s_algorithm;
        }

        ~SimulationTaskDistributor()
        {
            delete this->algorithm;
        }

        void setAlgorithm(NavigationAlgorithm *s_algorithm)
        {
            delete this->algorithm;
            this->algorithm = s_algorithm;
        }
        
        std::map<std::string, geometry_msgs::PoseStamped> run(const std::vector<std::string> robots, 
                                                            const std::vector<geometry_msgs::PoseStamped> goals)
        {
            return this->algorithm->run(robots, goals);
        }

    private:
        NavigationAlgorithm *algorithm;
};

}

#endif