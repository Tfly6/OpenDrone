#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>

#include <opendrone_gazebo_plugins/ContactPulse.h>

namespace gazebo {

class OpenDroneContactPlugin final : public ModelPlugin {
 public:
  OpenDroneContactPlugin() = default;

  ~OpenDroneContactPlugin() override {
    contact_subscriber_.reset();
    if (contact_manager_ != nullptr && !filter_name_.empty() &&
        contact_manager_->HasFilter(filter_name_)) {
      contact_manager_->RemoveFilter(filter_name_);
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    if (!model_) {
      gzerr << "[opendrone_contact] model is null\n";
      return;
    }
    if (!ros::isInitialized()) {
      gzerr << "[opendrone_contact] ROS is not initialized; load gazebo_ros_api_plugin first\n";
      return;
    }

    const std::string robot_namespace =
        ReadString(sdf, "robotNamespace", "/flight_eval");
    const std::string topic_name =
        ReadString(sdf, "topicName", "contact_pulse");
    ignored_collision_substring_ =
        ReadString(sdf, "ignoredCollisionSubstring", "ground_plane");
    const double recontact_gap =
        ReadDouble(sdf, "recontactGap", 0.1);
    recontact_gap_ = common::Time(std::max(0.0, recontact_gap));

    std::map<std::string, physics::CollisionPtr> own_collisions;
    for (const auto& link : model_->GetLinks()) {
      for (const auto& collision : link->GetCollisions()) {
        own_collisions.emplace(collision->GetScopedName(), collision);
      }
    }
    if (own_collisions.empty()) {
      gzwarn << "[opendrone_contact] model " << model_->GetName()
             << " has no collisions\n";
      return;
    }

    own_collision_names_.clear();
    for (const auto& item : own_collisions) {
      own_collision_names_.emplace(item.first, true);
    }

    contact_manager_ =
        model_->GetWorld()->Physics()->GetContactManager();
    filter_name_ = model_->GetScopedName() + "::opendrone_contact";
    const std::string filtered_topic =
        contact_manager_->CreateFilter(filter_name_, own_collisions);

    transport_node_.reset(new transport::Node());
    transport_node_->Init(model_->GetWorld()->Name());
    contact_subscriber_ = transport_node_->Subscribe(
        filtered_topic, &OpenDroneContactPlugin::OnContacts, this);

    ros_node_.reset(new ros::NodeHandle(robot_namespace));
    contact_publisher_ = ros_node_->advertise<
        opendrone_gazebo_plugins::ContactPulse>(topic_name, 1, false);

    gzmsg << "[opendrone_contact] monitoring " << own_collisions.size()
          << " collisions on " << robot_namespace << "/" << topic_name
          << "; ignoring '" << ignored_collision_substring_ << "'\n";
  }

 private:
  static std::string ReadString(
      const sdf::ElementPtr& sdf, const std::string& key,
      const std::string& fallback) {
    return sdf && sdf->HasElement(key)
        ? sdf->Get<std::string>(key)
        : fallback;
  }

  static double ReadDouble(
      const sdf::ElementPtr& sdf, const std::string& key, double fallback) {
    return sdf && sdf->HasElement(key)
        ? sdf->Get<double>(key)
        : fallback;
  }

  bool IsOwnCollision(const std::string& name) const {
    return own_collision_names_.find(name) != own_collision_names_.end();
  }

  bool IsIgnored(const std::string& name) const {
    return !ignored_collision_substring_.empty() &&
           name.find(ignored_collision_substring_) != std::string::npos;
  }

  void OnContacts(ConstContactsPtr& contacts) {
    if (!contacts || !contact_publisher_) {
      return;
    }
    const common::Time now = model_->GetWorld()->SimTime();
    for (int index = 0; index < contacts->contact_size(); ++index) {
      const auto& contact = contacts->contact(index);
      const bool first_is_own = IsOwnCollision(contact.collision1());
      const bool second_is_own = IsOwnCollision(contact.collision2());
      if (first_is_own == second_is_own) {
        continue;
      }

      const std::string& self =
          first_is_own ? contact.collision1() : contact.collision2();
      const std::string& other =
          first_is_own ? contact.collision2() : contact.collision1();
      if (IsIgnored(other)) {
        continue;
      }

      const std::string pair_key = self + '\n' + other;
      const auto previous = last_contact_time_.find(pair_key);
      const bool continuous_contact =
          previous != last_contact_time_.end() &&
          now >= previous->second &&
          now - previous->second <= recontact_gap_;
      last_contact_time_[pair_key] = now;
      if (continuous_contact) {
        continue;
      }

      opendrone_gazebo_plugins::ContactPulse pulse;
      pulse.header.stamp = ros::Time(now.sec, now.nsec);
      pulse.self_collision = self;
      pulse.other_collision = other;
      pulse.point_count = static_cast<std::uint16_t>(
          std::min(contact.position_size(), 65535));
      contact_publisher_.publish(pulse);
    }
  }

  physics::ModelPtr model_;
  physics::ContactManager* contact_manager_{nullptr};
  transport::NodePtr transport_node_;
  transport::SubscriberPtr contact_subscriber_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher contact_publisher_;
  std::string filter_name_;
  std::string ignored_collision_substring_;
  common::Time recontact_gap_;
  std::map<std::string, bool> own_collision_names_;
  std::unordered_map<std::string, common::Time> last_contact_time_;
};

GZ_REGISTER_MODEL_PLUGIN(OpenDroneContactPlugin)

}  // namespace gazebo
