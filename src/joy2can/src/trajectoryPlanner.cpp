#include "trajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(): posVelAccTime(1, 16)
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

void TrajectoryPlanner::addPoints(float x, float z)
{
    pt_x.push_back(x);
    pt_z.push_back(z);
}

// Prints the current list of points
void TrajectoryPlanner::printPoints()
{
    std::cout << "Trajectory points: \n[ | ";
    for (float i: pt_x){
    std::cout << i << " | ";
    }
    std::cout << "]" << std::endl;
    std::cout << "[ | ";
    for (float j: pt_z){
    std::cout << j << " | ";
    }
    std::cout << "]" << std::endl;
}

// Adds all points to the pt vectors to run the example path
void TrajectoryPlanner::plan()
{
    // Starting path
    addPoints(x_offset_, z_offset_);
    addPoints(x_offset_+ramp_dist_, z_offset_);
    addPoints(x_offset_+ramp_dist_+wall_width_, z_offset_);

    // All paths in-between start and ending path
    while(totalHeight_+wall_vStep_ <= wall_height_)
    {
        floor_ = floor_+1;
        totalHeight_ = totalHeight_ + wall_vStep_;

        if(totalHeight_+2*wall_vStep_ > wall_height_)
        {
            break;
        }

        addPoints(x_offset_+ramp_dist_+wall_width_, z_offset_+floor_*wall_vStep_);
        addPoints(x_offset_+ramp_dist_, z_offset_+floor_*wall_vStep_);

        floor_ = floor_+1;
        totalHeight_ = totalHeight_ + wall_vStep_;
        if(totalHeight_+2*wall_vStep_ > wall_height_)
        {
            break;
        }
        addPoints(x_offset_+ramp_dist_, z_offset_+floor_*wall_vStep_);
        addPoints(x_offset_+ramp_dist_+wall_width_, z_offset_+floor_*wall_vStep_);
    }
    
    // Ending path
    addPoints(x_offset_+ramp_dist_+wall_width_, z_offset_+floor_*wall_vStep_);
    addPoints(x_offset_+ramp_dist_, z_offset_+floor_*wall_vStep_);
    addPoints(x_offset_, z_offset_+floor_*wall_vStep_);
}

void TrajectoryPlanner::calcCartesianPosVelAcc()
{
    
    N = pt_x.size()-1;
    posVelAccTime.resize(N, 16);

    for (int i = 0; i < N; i++)
    {
        x0_ = pt_x.at(i);
        x1_ = pt_x.at(i+1);
        z0_ = pt_z.at(i);
        z1_ = pt_z.at(i+1);

    if(i == 0) // First trajectory (from stationary) will need zero initial velocity
    {
        v0_x_ = 0;
        v1_x_ = paint_vel_;
        t1_ = ramp_time_;
        sprayStatus_ = 0;
    }else if(i >= N-1) // Last trajectory (to stationary) will need zero final velocity
    {
        v0_x_ = -paint_vel_; // Also assumes that robot stops on same side as it started (negative initial velocity)
        v1_x_ = 0;
        t1_ = ramp_time_;
        sprayStatus_ = 0;   
    }else
    {
        if(pt_x.at(i) == pt_x.at(i+1)) // If both x-points for the path is equal it is a turn curve
        {
        // Check which way the turn is supposed to go
        if (pt_x.at(i) == x_offset_+ramp_dist_) // Indicates right turn
        {
            v0_x_ = -paint_vel_;
            v1_x_ = paint_vel_;
            t1_ = turn_time_;
            sprayStatus_ = 0;
        }else                                   // Indicates left turn
        {
            v0_x_ = paint_vel_;
            v1_x_ = -paint_vel_;
            t1_ = turn_time_;
            sprayStatus_ = 0;
        }
        
        }else if (pt_x.at(i) < pt_x.at(i+1)) // Last kind of trajectory is the straight lines
        {
            // Left to right movement
            v0_x_ = paint_vel_;
            v1_x_ = paint_vel_;
            t1_ = wall_width_/paint_vel_; // Calculates time based on specified wall width and painting velocity
            sprayStatus_ = 1;
        }else
        {
            // Right to left movement
            v0_x_ = -paint_vel_;
            v1_x_ = -paint_vel_;
            t1_ = wall_width_/paint_vel_; // Calculates time based on specified wall width and painting velocity
            sprayStatus_ = 1;
        }
        
    }

    if(i == 0)
    {
        t_sum_ = t1_;
    }else
    {
        t_sum_ = posVelAccTime(i-1,14)+t1_;
    }
    
    posVelAccTime(i,0) = x0_;
    posVelAccTime(i,1) = x1_;
    posVelAccTime(i,2) = z0_;
    posVelAccTime(i,3) = z1_;
    posVelAccTime(i,4) = v0_x_;
    posVelAccTime(i,5) = v1_x_;
    posVelAccTime(i,6) = v0_z_;
    posVelAccTime(i,7) = v1_z_;
    posVelAccTime(i,8) = a0_x_;
    posVelAccTime(i,9) = a1_x_;
    posVelAccTime(i,10) = a0_z_;
    posVelAccTime(i,11) = a1_z_;
    posVelAccTime(i,12) = t0_;
    posVelAccTime(i,13) = t1_;
    posVelAccTime(i,14) = t_sum_;
    posVelAccTime(i,15) = sprayStatus_;
    }
std::cout << posVelAccTime << std::endl;

}

// Getters
float TrajectoryPlanner::get_x_offset(){return x_offset_;}
float TrajectoryPlanner::get_z_offset(){return z_offset_;}
float TrajectoryPlanner::get_wall_width(){return wall_width_;}
float TrajectoryPlanner::get_wall_height(){return wall_height_;}
float TrajectoryPlanner::get_ramp_dist(){return ramp_dist_;}
float TrajectoryPlanner::get_outer_frame_width(){return outer_frame_width_;}
float TrajectoryPlanner::get_outer_frame_height(){return outer_frame_height_;}

// Setters
void TrajectoryPlanner::set_x_offset(float x_offset){x_offset_ = x_offset;}
void TrajectoryPlanner::set_z_offset(float z_offset){z_offset_ = z_offset;}
void TrajectoryPlanner::set_wall_width(float wall_width){wall_width_ = wall_width;}
void TrajectoryPlanner::set_wall_height(float wall_height){wall_height_ = wall_height;}
void TrajectoryPlanner::set_outer_frame_width(float outer_wall_width){outer_frame_width_ = outer_wall_width;}
void TrajectoryPlanner::set_outer_frame_height(float outer_wall_height){outer_frame_height_ = outer_wall_height;}
void TrajectoryPlanner::set_vStep(float wall_vStep){wall_vStep_ = wall_vStep;}

// Reset
void TrajectoryPlanner::reset()
{
    N = 0;
    floor_ = 0;
    totalHeight_ = 0;

    posVelAccTime = MatrixXf::Zero(1,16);
    pt_x.clear();
    pt_z.clear();
}