/* Author: Furkan Duman */

#include <robowflex_library/detail/franka.h>
#include <robowflex_library/io.h>
#include <robowflex_library/log.h>

using namespace robowflex;

const std::string  //
    FrankaRobot::DEFAULT_URDF{"package://franka_description/robots/panda/panda.urdf.xacro"};
const std::string  //
    FrankaRobot::DEFAULT_SRDF{"package://panda_moveit_config/config/panda.srdf.xacro"};
const std::string  //
    FrankaRobot::DEFAULT_LIMITS{"package://panda_moveit_config/config/joint_limits.yaml"};
const std::string  //
    FrankaRobot::DEFAULT_KINEMATICS{"package://panda_moveit_config/config/kinematics.yaml"};
const std::string  //
    OMPL::FrankaOMPLPipelinePlanner::DEFAULT_CONFIG{
        "package://robowflex_resources/panda/config/ompl_planning.yaml"  //
    };

const std::string  //
    FrankaRobot::RESOURCE_URDF{"package://robowflex_resources/panda/urdf/panda.urdf"};
const std::string  //
    FrankaRobot::RESOURCE_SRDF{"package://robowflex_resources/panda/config/panda.srdf"};
const std::string  //
    FrankaRobot::RESOURCE_LIMITS{"package://robowflex_resources/panda/config/joint_limits.yaml"};
const std::string  //
    FrankaRobot::RESOURCE_KINEMATICS{"package://robowflex_resources/panda/config/kinematics.yaml"};
const std::string  //
    OMPL::FrankaOMPLPipelinePlanner::RESOURCE_CONFIG{
        "package://robowflex_resources/panda/config/ompl_planning.yaml"  //
    };

FrankaRobot::FrankaRobot() : Robot("panda")
{
}

bool FrankaRobot::initialize()
{
    bool success = false;

    // First attempt the `robowflex_resources` package, then attempt the "actual" resource files.
    if (IO::resolvePackage(RESOURCE_URDF).empty() or IO::resolvePackage(RESOURCE_SRDF).empty())
    {
        RBX_INFO("Initializing Panda with `panda_{description, moveit_config}`");
        success = Robot::initialize(DEFAULT_URDF, DEFAULT_SRDF, DEFAULT_LIMITS, DEFAULT_KINEMATICS);
    }
    else
    {
        RBX_INFO("Initializing Panda with `robowflex_resources`");
        success = Robot::initialize(RESOURCE_URDF, RESOURCE_SRDF, RESOURCE_LIMITS, RESOURCE_KINEMATICS);
    }

    loadKinematics("panda_arm");
    // loadKinematics("panda_manipulator");
    loadKinematics("hand");

    FrankaRobot::openGripper();

    return success;
}

void FrankaRobot::openGripper()
{
    const std::map<std::string, double> angles = {{"panda_finger_joint1", 0.04},
                                                  {"panda_finger_joint2", 0.04}};

    Robot::setState(angles);
}

void FrankaRobot::closeGripper()
{
    const std::map<std::string, double> angles = {{"panda_finger_joint1", 0.0},
                                                  {"panda_finger_joint2", 0.0}};

    Robot::setState(angles);
}

void FrankaRobot::setBasePose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
{
    // Check if the virtual joints exist in the robot model
    if (hasJoint("virtual_joint/trans_x") && hasJoint("virtual_joint/trans_y") && hasJoint("virtual_joint/trans_z") &&
        hasJoint("virtual_joint/rot_x") && hasJoint("virtual_joint/rot_y") && hasJoint("virtual_joint/rot_z") && hasJoint("virtual_joint/rot_w"))
    {
        // Create a map with the joint names and desired positions
        const std::map<std::string, double> pose = {
            {"virtual_joint/trans_x", x},
            {"virtual_joint/trans_y", y},
            {"virtual_joint/trans_z", z},
            {"virtual_joint/rot_x", q_x},
            {"virtual_joint/rot_y", q_y},
            {"virtual_joint/rot_z", q_z},
            {"virtual_joint/rot_w", q_w}};

        // Set the joint positions in the robot state
        scratch_->setVariablePositions(pose);
        scratch_->update(); // Update the robot state
    }
    else
    {
        // If the virtual joints don't exist, log a warning message
        RBX_WARN("Virtual joints do not exist, cannot move base! Make sure they are defined in your URDF.");
    }
}


OMPL::FrankaOMPLPipelinePlanner::FrankaOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name)
  : OMPLPipelinePlanner(robot, name)
{
}

bool OMPL::FrankaOMPLPipelinePlanner::initialize(const Settings &settings,
                                              const std::vector<std::string> &adapters)
{
    if (IO::resolvePackage(RESOURCE_CONFIG).empty())
        return OMPLPipelinePlanner::initialize(DEFAULT_CONFIG, settings, DEFAULT_PLUGIN, adapters);
    else
        return OMPLPipelinePlanner::initialize(RESOURCE_CONFIG, settings, DEFAULT_PLUGIN, adapters);
}