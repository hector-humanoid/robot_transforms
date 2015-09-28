#ifndef robot_transforms_H
#define robot_transforms_H

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_tools {
    class RobotTransforms {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RobotTransforms() :
            initialized_(false)
        {}

        void init(const std::string& robot_description_param_name = "/robot_description");

        void updateState(const std::map<std::string, double>& state_map);
        void updateState(const std::string joint_name, double state);

        double getJointPosition(const std::string joint_name);

        void updateRootTransform(const Eigen::Affine3d& transform);
        const Eigen::Affine3d& getRootTransform();

        Eigen::Affine3d getTransform(std::string link_name);
        Eigen::Affine3d getTransform(std::string base_frame, std::string target_frame);
    private:
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        robot_model::RobotModelPtr robot_model_;
        robot_state::RobotStatePtr robot_state_ptr_;

        Eigen::Affine3d root_transform_;

        bool initialized_;

    };
}


#endif
