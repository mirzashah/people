#include <social_costmap_plugin/social_costmap_plugin.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(social_costmap_plugin::SocialLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx), 
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var){
    return sqrt(-2*var * log(cutoff/A) );
}


namespace social_costmap_plugin
{
    void SocialLayer::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
    {
        ros::NodeHandle nh("~/" + name), g_nh;
        layered_costmap_ = costmap;
        name_ = name;
        current_ = true;

        server_ = new dynamic_reconfigure::Server<SocialCostmapConfig>(nh);
        f_ = boost::bind(&SocialLayer::configure, this, _1, _2);
        server_->setCallback(f_);

        people_sub_ = nh.subscribe("/people", 1, &SocialLayer::peopleCallback, this);
    }
    
    
    void SocialLayer::peopleCallback(const people_velocity_tracker::PersonPositionAndVelocity& person) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(people_list_.size()>0 && (*people_list_.begin()).header.stamp!=person.header.stamp)
          people_list_.clear();

        people_list_.push_front(person);
      }


    void SocialLayer::update_bounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){
        boost::recursive_mutex::scoped_lock lock(lock_);
        
        // clear old people
        ros::Duration time_diff = ros::Time::now() - (*people_list_.begin()).header.stamp;
        if(time_diff > people_keep_time_){
          people_list_.clear();
          return;
        }
        
        std::list<people_velocity_tracker::PersonPositionAndVelocity>::iterator p_it;
        std::string global_frame = layered_costmap_->getGlobalFrameID();
        transformed_people_.clear();
        
        for(p_it = people_list_.begin(); p_it != people_list_.end(); ++p_it){
            people_velocity_tracker::PersonPositionAndVelocity person = *p_it;
            people_velocity_tracker::PersonPositionAndVelocity tpt;
            geometry_msgs::PointStamped pt, opt;
            

            try{
              pt.point.x = person.position.x;
              pt.point.y = person.position.y;
              pt.point.z = person.position.z;
              pt.header.frame_id = person.header.frame_id;
              tf_.transformPoint(global_frame, pt, opt);
              tpt.position.x = opt.point.x;
              tpt.position.y = opt.point.y;
              tpt.position.z = opt.point.z;

              pt.point.x += person.velocity.x;
              pt.point.y += person.velocity.y;
              pt.point.z += person.velocity.z;
              tf_.transformPoint(global_frame, pt, opt);
              
              tpt.velocity.x = tpt.position.x - opt.point.x;
              tpt.velocity.y = tpt.position.y - opt.point.y;
              tpt.velocity.z = tpt.position.z - opt.point.z;
              
              transformed_people_.push_back(tpt);
              
              double mag = sqrt(pow(tpt.velocity.x,2) + pow(person.velocity.y, 2));
              double factor = 1.0 + mag * factor_;
              double point = get_radius(cutoff_, amplitude_, covar_ * factor );
              
              *min_x = std::min(*min_x, tpt.position.x - point);
              *min_y = std::min(*min_y, tpt.position.y - point);
              *max_x = std::max(*max_x, tpt.position.x + point);
              *max_y = std::max(*max_y, tpt.position.y + point);
              
            }
            catch(tf::LookupException& ex) {
              ROS_ERROR("No Transform available Error: %s\n", ex.what());
              continue;
            }
            catch(tf::ConnectivityException& ex) {
              ROS_ERROR("Connectivity Error: %s\n", ex.what());
              continue;
            }
            catch(tf::ExtrapolationException& ex) {
              ROS_ERROR("Extrapolation Error: %s\n", ex.what());
              continue;
            }
        }
    }
    
    void SocialLayer::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( people_list_.size() == 0 )
          return;
        if( cutoff_ >= amplitude_)
            return;
        
        std::list<people_velocity_tracker::PersonPositionAndVelocity>::iterator p_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        
        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            people_velocity_tracker::PersonPositionAndVelocity person = *p_it;
            double angle = atan2(person.velocity.y, person.velocity.x);
            double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));
            double factor = 1.0 + mag * factor_;
            double base = get_radius(cutoff_, amplitude_, covar_);
            double point = get_radius(cutoff_, amplitude_, covar_ * factor );
            
            unsigned int width = std::max(1, int( (base + point) / res )),
                          height = std::max(1, int( (base + point) / res ));
                          
            double cx = person.position.x, cy = person.position.y;

            double ox, oy;
            if(sin(angle)>0)
                oy = cy - base;
            else
                oy = cy + (point-base) * sin(angle) - base;

            if(cos(angle)>=0)
                ox = cx - base;
            else
                ox = cx + (point-base) * cos(angle) - base;


            int dx, dy;
            costmap->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y=0, end_x=width, end_y = height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap->getSizeInCellsX())
                end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

            if((int)(start_x+dx) < min_i)
                start_x = min_i - dx;
            if((int)(end_x+dx) > max_i)
                end_x = max_i - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap->getSizeInCellsY())
                end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

            if((int)(start_y+dy) < min_j)
                start_y = min_j - dy;
            if((int)(end_y+dy) > max_j)
                end_y = max_j - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
            for(int i=start_x;i<end_x;i++){
                for(int j=start_y;j<end_y;j++){
                  unsigned char old_cost = costmap->getCost(i+dx, j+dy);
                  if(old_cost == costmap_2d::NO_INFORMATION)
                    continue;

                  double x = bx+i*res, y = by+j*res;
                  double ma = atan2(y-cy,x-cx);
                  double diff = angles::shortest_angular_distance(angle, ma);
                  double a;
                  if(fabs(diff)<M_PI/2)
                      a = gaussian(x,y,cx,cy,amplitude_,covar_*factor,covar_,angle);
                  else
                      a = gaussian(x,y,cx,cy,amplitude_,covar_,       covar_,0);

                  if(a < cutoff_)
                    continue;
                  unsigned char cvalue = (unsigned char) a;
                  costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));

              }
            } 

            
        }
    }

    void SocialLayer::configure(SocialCostmapConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        people_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
    }


};
