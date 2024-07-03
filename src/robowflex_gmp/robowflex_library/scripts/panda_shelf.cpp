/* Author: Davina Sanghera */

#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/detail/franka.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/io/broadcaster.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

using namespace robowflex;

/* \file panda_shelf.cpp
 * Motion planning with the Franka combined with visualization.
 */

static const std::string ARM = "panda_arm";
static const std::string GRIPPER = "panda_hand";

int main(int argc, char **argv)
{

    // Startup ROS
    ROS ros(argc, argv);

    // Create the default Panda robot.
    auto panda = std::make_shared<FrankaRobot>();
    panda->initialize();

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by default.
    IO::RVIZHelper rviz(panda);
  
    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    usleep(5000);

    // Load a scene from a YAML file.
    auto scene = std::make_shared<Scene>(panda);
    scene->fromYAMLFile("package://robowflex_resources/panda.yml");

    // Visualize the scene.
    rviz.updateScene(scene);

    // Open results file
    std::ofstream resultsfile;
    std::string resultspath = "/home/davinasanghera/test_franka_gmp/test_franka_gmp_simulation_bringup/src/results/results.txt"; // CHANGE TO YOUR OWN PATH
    resultsfile.open(resultspath, std::ios::out | std::ofstream::trunc);

    // Open planning time file
    std::ofstream timingfile;
    std::string timingpath = "/home/davinasanghera/test_franka_gmp/test_franka_gmp_simulation_bringup/src/results/timings.txt"; // CHANGE TO YOUR OWN PATH
    timingfile.open(timingpath, std::ios::out | std::ofstream::trunc);

    // Open pose file
    std::fstream posesfile;
    std::string posespath = "/home/davinasanghera/test_franka_gmp/test_franka_gmp_simulation_bringup/src/results/poses.txt"; // CHANGE TO YOUR OWN PATH
    posesfile.open(posespath, std::ios::in);
    if (posesfile.is_open()){
        std::string line;
        while(getline(posesfile, line)){

            std::stringstream linestream(line);
            std::string currentpose;
            std::vector< double > arr;

            // Save current pose to array
            while(getline(linestream, currentpose, ','))
            {   
                std::stringstream valuestream(currentpose);
                std::string value;
                while(getline(valuestream, value, ' '))
                {   
                    double val = std::stod(value);
                    arr.push_back(val);
                }
            }

            // Create the default planner for the Panda.
            auto planner = std::make_shared<OMPL::FrankaOMPLPipelinePlanner>(panda, "panda");
            planner->initialize();
        
            // Create a motion planning request with a pose goal.
            MotionRequestBuilder request(planner, ARM);
            panda->setGroupState(ARM, {1.6226691970685083, -1.4696587055637407, -1.875046522904153, -2.8552946888576027, -2.852911180500942, 3.4276649380615902, 1.7688121670587176});
            request.setConfig("RRTConnect");
            request.setStartConfiguration(panda->getScratchState());

            std::cout << arr[0] << "\n";
            std::cout << arr[1] << "\n";
            std::cout << arr[2] << "\n";
            std::cout << arr[3] << "\n";
            std::cout << arr[4] << "\n";
            std::cout << arr[5] << "\n";
            std::cout << arr[6] << "\n";

            const auto &goal_pose = TF::createPoseQ(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6]);
            request.setGoalPose("panda_link8", "world", goal_pose);
            
            rviz.addGoalMarker("goal", request);
            rviz.updateMarkers();

            // Do motion planning!
            planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
            if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                resultsfile << 0 << std::endl;
                timingfile << 5.0 << std::endl;
                continue;
            }
            else {
                resultsfile << 1 << std::endl;
                timingfile << res.planning_time << std::endl;
                rviz.updateTrajectory(res);
            }

        }
        posesfile.close();
        resultsfile.close();
        timingfile.close();

        RBX_INFO("Press enter to remove goal and scene.");
        std::cin.get();
    
        rviz.removeMarker("goal");
        rviz.updateMarkers();
        rviz.removeScene();
        
        RBX_INFO("Press enter to exit.");
        std::cin.get();
    
        return 0;
    }

}

