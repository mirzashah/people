#ifndef SOCIAL_COSTMAP_PLUGIN_H_
#define SOCIAL_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <people_velocity_tracker/PersonPositionAndVelocity.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <social_costmap_plugin/SocialCostmapConfig.h>

namespace social_costmap_plugin
{
  class SocialCostmapPlugin : public costmap_2d::CostmapPluginROS
  {
    public:
      SocialCostmapPlugin() { layered_costmap_ = NULL; }

      virtual void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      void activate() {}
      void deactivate() {}    
      bool isDiscretized() { return false; }
      void matchSize() {}

    private:
      void peopleCallback(const people_velocity_tracker::PersonPositionAndVelocity& person);
      void configure(SocialCostmapConfig &config, uint32_t level);
      ros::Subscriber people_sub_;
      std::list<people_velocity_tracker::PersonPositionAndVelocity> people_list_;
      std::list<people_velocity_tracker::PersonPositionAndVelocity> transformed_people_;
      double cutoff_, amplitude_, covar_, factor_;
      ros::Duration people_keep_time_;
      boost::recursive_mutex lock_;
      tf::TransformListener tf_;
      dynamic_reconfigure::Server<SocialCostmapConfig>* server_;
      dynamic_reconfigure::Server<SocialCostmapConfig>::CallbackType f_;
  };
};
#endif

