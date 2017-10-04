#include<robot_transforms/robot_transforms.h>

namespace robot_tools {

void RobotTransforms::init(const std::string &robot_description_param_name) {
    ros::NodeHandle nh;
    std::string robot_description;
    if (!nh.getParam(robot_description_param_name, robot_description)) {
        ROS_ERROR_STREAM("RobotTransforms couldn't find robot_description on " << nh.getNamespace() << "/" << robot_description_param_name <<". Init failed.");
        return;
    }
    std::string robot_semantics;
    if (!nh.getParam(robot_description_param_name + "_semantic", robot_semantics)) {
        ROS_ERROR_STREAM("RobotTransforms couldn't find robot_description_semantic on " << nh.getNamespace() << "/" << robot_description_param_name << "_semantic" <<". Init failed.");
        return;
    }

    robot_model_loader::RobotModelLoader::Options options(robot_description, robot_semantics);
    options.load_kinematics_solvers_ = false;
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(options));

    robot_model_ptr_ = robot_model_loader_->getModel();
    robot_state_ptr_.reset(new robot_state::RobotState(robot_model_ptr_));
    robot_state_ptr_->setToDefaultValues();
    root_transform_ = Eigen::Affine3d::Identity();
    initialized_ = true;
}

void RobotTransforms::updateState(const std::map<std::string, double>& state_map) {
    if (!initialized_) return;
    robot_state_ptr_->setVariablePositions(state_map);
}

void RobotTransforms::updateState(const std::string joint_name, double state) {
    if (!initialized_) return;
    try {
        robot_state_ptr_->setVariablePosition(joint_name, state);
    } catch (moveit::Exception e) {
        ROS_ERROR_STREAM_THROTTLE(1, "Couldn't update state for joint '" << joint_name << "'. Exception: " << e.what());
    }
}

double RobotTransforms::getJointPosition(const std::string joint_name) {
    if (!initialized_) return 0.0;
    double position = 0;
    try {
        position = robot_state_ptr_->getVariablePosition(joint_name);
    } catch (moveit::Exception e) {
        ROS_ERROR_STREAM_THROTTLE(1, "Couldn't get state for joint '" << joint_name << "'. Exception: " << e.what());
    }
    return position;
}

void RobotTransforms::updateRootTransform(const Eigen::Affine3d& transform) {
    root_transform_ = transform;
}

const Eigen::Affine3d& RobotTransforms::getRootTransform() {
   return root_transform_;
}

Eigen::Affine3d RobotTransforms::getTransform(std::string link_name) {
    if (!initialized_) return Eigen::Affine3d::Identity();
    try {
        const Eigen::Affine3d& transform = robot_state_ptr_->getGlobalLinkTransform(link_name);
        return transform;
    } catch (moveit::Exception e) {
        ROS_ERROR_STREAM_THROTTLE(1, "Couldn't get transform for link '" << link_name << "'. Exception: " << e.what());
        return Eigen::Affine3d::Identity();
    }
}

Eigen::Affine3d RobotTransforms::getTransform(std::string base_frame, std::string target_frame) {
    if (!initialized_) return Eigen::Affine3d::Identity();
    return robot_state_ptr_->getGlobalLinkTransform(base_frame).inverse() * robot_state_ptr_->getGlobalLinkTransform(target_frame);
}

std::string RobotTransforms::getChildLink(std::string joint_name) {
  moveit::core::JointModel * joint = robot_model_ptr_->getJointModel(joint_name);
  if (joint == NULL) {
    ROS_ERROR_STREAM("[robot_transforms] Can't find joint with name '" << joint_name << "'.");
    return "";
  }

  const moveit::core::LinkModel * link = joint->getChildLinkModel();
  if (link == NULL) {
    ROS_ERROR_STREAM("[robot_transforms] joint had no child link (shouldn't never happen).");
    return "";
  }

  return link->getName();
}

std::string RobotTransforms::getParentLink(std::string joint_name) {
  moveit::core::JointModel * joint = robot_model_ptr_->getJointModel(joint_name);
  if (joint == NULL) {
    ROS_ERROR_STREAM("[robot_transforms] Can't find joint with name '" << joint_name << "'.");
    return "";
  }

  const moveit::core::LinkModel * link = joint->getParentLinkModel();
  if (link == NULL) {
    ROS_ERROR_STREAM("[robot_transforms] joint had no parent link (shouldn't never happen).");
    return "";
  }

  return link->getName();
}


}
