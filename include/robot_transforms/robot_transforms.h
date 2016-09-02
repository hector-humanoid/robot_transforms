#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_tools {
     /**
     * @brief Stores the state of the robot (joint positions and root transformation) and computes forward kinematics.
     */
    class RobotTransforms {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Default constructor
         */
        RobotTransforms() :
            initialized_(false)
        {}

        /**
         * @brief Initializes RobotTransforms
         * @param robot_description_param_name (optional) Specifies the location of the robot
         *                                     description on the parameter server.
         */
        void init(const std::string& robot_description_param_name = "/robot_description");

        /**
         * @brief Updates multiple joint states at once
         * @param state_map Map between joint name and joint position
         */
        void updateState(const std::map<std::string, double>& state_map);
        /**
         * @brief Updates a single joint position
         * @param joint_name Name of the joint
         * @param state Position of the joint
         */
        void updateState(const std::string joint_name, double state);

        /**
         * @brief Returns the position of a joint
         * @param joint_name Name of the joint
         * @return Position of the joint
         */
        double getJointPosition(const std::string joint_name);

        /**
         * @brief Updates the transformation to the root link
         * @param transform Transform (rotation and translation) to the root link in world coordinates
         */
        void updateRootTransform(const Eigen::Affine3d& transform);
        /**
         * @brief Returns the Transform to the root link in world coordinates
         * @return Transform (rotation and translation) to root link
         */
        const Eigen::Affine3d& getRootTransform();

        /**
         * @brief Calculates forward kinematics from the root link to a given link
         * @param link_name Name of the link
         * @return Transformation matrix from root to link_name
         */
        Eigen::Affine3d getTransform(std::string link_name);
        /**
         * @brief Calculates forward kinematics between frames
         * @param base_frame Reference frame
         * @param target_frame Target frame
         * @return Transformation matrix from base_frame to target_frame
         */
        Eigen::Affine3d getTransform(std::string base_frame, std::string target_frame);
    private:
        /**
         * @brief Loads the model (robot_description, robot_description_semantic) from parameter server
         */
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        /**
         * @brief Stores the robot model
         */
        robot_model::RobotModelPtr robot_model_;
        /**
         * @brief Stores the state of the robot and computes FK
         */
        robot_state::RobotStatePtr robot_state_ptr_;

        /**
         * @brief Stores transform to root link
         */
        Eigen::Affine3d root_transform_;

        /**
         * @brief True if init() has been called
         */
        bool initialized_;

    };
}


#endif
