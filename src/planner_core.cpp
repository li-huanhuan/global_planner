#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/tf.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
  costmap_(nullptr), initialized_(false), allow_unknown_(true)
{}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
  costmap_(nullptr), initialized_(false), allow_unknown_(true)
{
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner()
{
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
    if (orientation_filter_)
      delete orientation_filter_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
        {
            AStarExpansion* as = new AStarExpansion(p_calc_, cx, cy);
            planner_ = as;
        }

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        potential_array_ = nullptr;

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }
    else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level)
{
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my)
{
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& set_start,
                             const geometry_msgs::PoseStamped& set_goal,
                             double tolerance,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    geometry_msgs::PoseStamped start_new;
    geometry_msgs::PoseStamped goal_new;

    this->getNearFreePoint(set_start,start_new,tolerance);
    this->getNearFreePoint(set_goal,goal_new,tolerance);

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal_new.header.frame_id != frame_id_)
    {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", frame_id_.c_str(), goal_new.header.frame_id.c_str());
        return false;
    }

    if (start_new.header.frame_id != frame_id_)
    {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", frame_id_.c_str(), start_new.header.frame_id.c_str());
        return false;
    }

    double wx = start_new.pose.position.x;
    double wy = start_new.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
    {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_)
    {
        start_x = start_x_i;
        start_y = start_y_i;
    }
    else
    {
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal_new.pose.position.x;
    wy = goal_new.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
    {
        ROS_WARN_THROTTLE(1.0,"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_)
    {
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }
    else
    {
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start_new, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    try
    {
      potential_array_ = new float[nx * ny];
    }
    catch (...)
    {
      ROS_ERROR("global planner new float[nx * ny] error.");
      return false;
    }

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    uint8_t plan_timer = 0;
    this->planner_->setSafeControl(true);
    while(plan_timer < 2 && plan.empty())
    {
        //计算可行点矩阵，距离机器人越近值越小
        bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(),
                                                         start_x, start_y, goal_x, goal_y,
                                                         nx * ny * 2, potential_array_);

        if(!old_navfn_behavior_)
            planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
        if(publish_potential_)
        {
            publishPotential();
        }

        if (found_legal)
        {
            //extract the plan
            //从可行点矩阵提取路径
            if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal_new, plan))
            {
                //make sure the goal we push on has the same timestamp as the rest of the plan
                geometry_msgs::PoseStamped goal_copy = goal_new;
                goal_copy.header.stamp = ros::Time::now();
                plan.push_back(goal_copy);
            }
            else
            {
              if(plan_timer == 1)
                ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
        }
        else
        {
          if(plan_timer == 1)
            ROS_INFO("Failed to get a plan.");
        }
        plan_timer++;
        this->planner_->setSafeControl(false);
    }
    if(!plan.empty())
    {
      // optimization path
      this->optimizationPath(plan,M_PI/10);
      // add orientations if needed
      optimizationOrientation(plan);
      //orientation_filter_->processPath(start_new, plan);
    }
    if(potential_array_)
    {
      delete[] potential_array_;
      potential_array_ = nullptr;
    }
    //publish the plan for visualization purposes
    publishPlan(plan);
    return !plan.empty();
}

int GlobalPlanner::optimizationPath(std::vector<geometry_msgs::PoseStamped>& plan,double movement_angle_range)
{
  if(plan.empty())
    return 0;
  size_t pose_size = plan.size() - 1;
  double px,py,cx,cy,nx,ny,a_p,a_n;
  bool is_run = false;
  int ci = 0;
  for(ci=0;ci<1000;ci++)
  {
    is_run = false;
    for(size_t i=1;i<pose_size;i++)
    {
      px = plan[i-1].pose.position.x;
      py = plan[i-1].pose.position.y;

      cx = plan[i].pose.position.x;
      cy = plan[i].pose.position.y;

      nx = plan[i+1].pose.position.x;
      ny = plan[i+1].pose.position.y;

      a_p = normalizeAngle(atan2(cy-py,cx-px),0,2*M_PI);
      a_n = normalizeAngle(atan2(ny-cy,nx-cx),0,2*M_PI);

      if(std::max(a_p,a_n)-std::min(a_p,a_n) > movement_angle_range)
      {
        plan[i].pose.position.x = (px + nx)/2;
        plan[i].pose.position.y = (py + ny)/2;
        is_run = true;
      }
    }
    if(!is_run)
      return ci;
  }
  return ci;
}

void GlobalPlanner::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
{
  size_t num = plan.size()-1;
  if(num < 1)
    return;
  for(size_t i=0;i<num;i++)
  {
    plan[i].pose.orientation = tf::createQuaternionMsgFromYaw( atan2( plan[i+1].pose.position.y - plan[i].pose.position.y,
                                                               plan[i+1].pose.position.x - plan[i].pose.position.x ) );
  }
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y,
                                         double goal_x, double goal_y,
                                         const geometry_msgs::PoseStamped& goal,
                                         std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path))
    {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    int path_size_num = path.size() -1;
    for (int i = path_size_num; i>=0; i--)
    {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        if(i != path_size_num)
        {
          double cx,cy,px,py;
          cx = pose.pose.position.x;
          cy = pose.pose.position.y;
          px = plan.back().pose.position.x;
          py = plan.back().pose.position.y;
          if( sqrt( (cx-px)*(cx-px) + (cy-py)*(cy-py) ) > 0.05)
          {
            geometry_msgs::PoseStamped pose_insert = pose;
            pose_insert.pose.position.x = (cx+px)/2;
            pose_insert.pose.position.y = (cy+py)/2;
            plan.push_back(pose_insert);
          }
        }
        plan.push_back(pose);
    }
    if(old_navfn_behavior_)
    {
        plan.push_back(goal);
    }
    return !plan.empty();
}

double GlobalPlanner::distance(double x1,double y1,double x2,double y2)
{
  return sqrt( (x1-x2) * (x1-x2) + (y1-y2) * (y1-y2) );
}

bool GlobalPlanner::isAroundFree(unsigned int mx, unsigned int my)
{
  if(mx <= 1 || my <= 1 || mx >= this->costmap_->getSizeInCellsX()-1 || my >= this->costmap_->getSizeInCellsY()-1)
    return false;
  int x,y;
  for(int i=-1;i<=1;i++)
  {
    for(int j=-1;j<=1;j++)
    {
      x = static_cast<int>(mx) + i;
      y = static_cast<int>(my) + j;
      if(this->costmap_->getCost(static_cast<unsigned int>(x),static_cast<unsigned int>(y)) != costmap_2d::FREE_SPACE)
        return false;
    }
  }
  return true;
}

void GlobalPlanner::getNearFreePoint(const geometry_msgs::PoseStamped in,
                                     geometry_msgs::PoseStamped& out,
                                     double tolerance)
{
  out = in;
  unsigned int grid_size = static_cast<unsigned int>(tolerance/costmap_->getResolution() + 0.5);
  if(grid_size<1)
  {
    out = in;
    return;
  }

  unsigned int mx0,my0;
  if(costmap_->worldToMap(in.pose.position.x,in.pose.position.y,mx0,my0))
  {
    if(this->isAroundFree(mx0,my0))
      return;
    unsigned int minx,maxx,miny,maxy;
    double wx = 0.0,wy = 0.0;
    double min_move_cost = 10000000.0;
    minx = mx0-grid_size>0?mx0-grid_size:0;
    maxx = mx0+grid_size<costmap_->getSizeInCellsX()?mx0+grid_size:costmap_->getSizeInCellsX();
    miny = my0-grid_size>0?my0-grid_size:0;
    maxy = my0+grid_size<costmap_->getSizeInCellsY()?my0+grid_size:costmap_->getSizeInCellsY();
    for(unsigned int i=minx;i<=maxx;i++)
    {
      for(unsigned int j=miny;j<=maxy;j++)
      {
        costmap_->mapToWorld(i,j,wx,wy);
        double current_move_cost = this->distance(in.pose.position.x,in.pose.position.y,wx,wy);
        if(!this->isAroundFree(i,j) || current_move_cost > tolerance)
          continue;
        if(min_move_cost > current_move_cost)
        {
          min_move_cost = current_move_cost;
          out.pose.position.x = wx;
          out.pose.position.y = wy;
        }
      }
    }
  }
}

void GlobalPlanner::publishPotential()
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        float potential = potential_array_[i];
        if (potential < POT_HIGH)
        {
            if (potential > max)
            {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        if (potential_array_[i] >= POT_HIGH)
        {
            grid.data[i] = -1;
        }
        else
        {
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
        }
    }
    potential_pub_.publish(grid);
}

} //end namespace global_planner
