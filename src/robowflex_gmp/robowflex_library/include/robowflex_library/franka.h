/* Author: Furkan Duman */

#ifndef ROBOWFLEX_FRANKA_
#define ROBOWFLEX_FRANKA_

#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(FrankaRobot);
    /* \endcond */

    /** \class robowflex::FrankaRobotPtr
        \brief A shared pointer wrapper for robowflex::FrankaRobot. */

    /** \class robowflex::FrankaRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::FrankaRobot. */

    /** \brief Convenience class that describes the default setup for Panda (with robotiq gripper and load cell)
     *  Will first attempt to load configuration and description from the robowflex_resources package.
     *  See https://github.com/KavrakiLab/robowflex_resources for this package.
     *  If this package is not available, then franka_description / robots packages will be used.
     */
    class FrankaRobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        FrankaRobot();

        /** \brief Initialize the robot with manipulator kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

        /** \brief Sets the base pose of the Panda robot (a virtual planar joint)
         *  \param[in] x The x position.
         *  \param[in] y The y position.
         *  \param[in] z The z position.
         *  \param[in] q_x,q_y,q_z,q_w The angles.
         */
        void setBasePose(double x, double y, double z, double q_x, double q_y, double q_z, double q_w);

        /** \brief Opens the Panda's gripper.
         */
        void openGripper();

        /** \brief Closes the Panda's gripper.
         */
        void closeGripper();

    private:
        static const std::string DEFAULT_URDF;        ///< Default URDF
        static const std::string DEFAULT_SRDF;        ///< Default SRDF
        static const std::string DEFAULT_LIMITS;      ///< Default Limits
        static const std::string DEFAULT_KINEMATICS;  ///< Default kinematics

        static const std::string RESOURCE_URDF;        ///< URDF from robowflex_resources
        static const std::string RESOURCE_SRDF;        ///< SRDF from robowflex_resources
        static const std::string RESOURCE_LIMITS;      ///< Limits from robowflex_resources
        static const std::string RESOURCE_KINEMATICS;  ///< kinematics from robowflex_resources
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(FrankaOMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::FrankaOMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::FrankaOMPLPipelinePlanner. */

        /** \class robowflex::OMPL::FrankaOMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::FrankaOMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for Panda.
         */
        class FrankaOMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            FrankaOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const Settings &settings = Settings(),
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string DEFAULT_CONFIG;   ///< Default planning configuration.
            static const std::string RESOURCE_CONFIG;  ///< Planning configuration from robowflex_resources.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
