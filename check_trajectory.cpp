#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include "franka/circle_trajectory.h"

int main() {
    double radius = 0.1;
    double line_time = 10.0;
    double circle_time = 20.0;
    double time_step = 0.001;

    try {
        franka::CircleTrajectory traj = franka::generateCircleTrajectory(radius, line_time, circle_time, time_step);
        
        std::cout << "Generated " << traj.points.size() << " points." << std::endl;

        if (traj.points.empty()) return 0;

        double max_vel_diff = 0.0;
        double max_acc_diff = 0.0;
        double max_pos_jump = 0.0;

        for (size_t i = 1; i < traj.points.size(); ++i) {
            const auto& p_prev = traj.points[i-1];
            const auto& p_curr = traj.points[i];

            // Real Time Delta (from trajectory)
            // Note: In the robot loop, we effectively assume dt = 0.001
            // So we should check the delta based on 0.001 step
            
            double dx = p_curr.position[0] - p_prev.position[0];
            double dy = p_curr.position[1] - p_prev.position[1];
            double dz = p_curr.position[2] - p_prev.position[2];
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            double velocity = dist / time_step; // Effective velocity if played at 1kHz

            // Check if there are duplicate points (dt=0 in playback)
            // But verify if the points themselves have correct timestamps
            double dt_stored = p_curr.time - p_prev.time;

            if (dt_stored < 1e-9) {
                std::cout << "WARNING: Duplicate or Zero-dt point at index " << i 
                          << " Time: " << p_curr.time << std::endl;
            }
            
            // Analyze Jumps
            // Discontinuity implies a sudden change in derivatives
            // We approximate derivatives using finite difference of Position over fixed control step
            
            if (dist > max_pos_jump) max_pos_jump = dist;
            
            // Check implicit velocity vs stored velocity
            // This checks if the trajectory generation math matches the discrete steps
            double v_x = dx / time_step;
            double v_y = dy / time_step;
            double v_z = dz / time_step;
            
            // TODO: Calculate acceleration from velocity changes
        }
        
        // Let's do a stricter check: Finite Difference Acceleration
        // A_eff = (Pos[i] - 2*Pos[i-1] + Pos[i-2]) / dt^2
        
        double max_acc_norm = 0.0;
        int max_acc_idx = 0;

        for (size_t i = 2; i < traj.points.size(); ++i) {
             const auto& p0 = traj.points[i-2];
             const auto& p1 = traj.points[i-1];
             const auto& p2 = traj.points[i];
             
             std::array<double, 3> acc;
             for(int k=0; k<3; ++k) {
                 double vel1 = (p1.position[k] - p0.position[k]) / time_step;
                 double vel2 = (p2.position[k] - p1.position[k]) / time_step;
                 acc[k] = (vel2 - vel1) / time_step;
             }
             
             double acc_norm = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
             if (acc_norm > max_acc_norm) {
                 max_acc_norm = acc_norm;
                 max_acc_idx = i;
             }
        }
        
        std::cout << "Max Effective Acceleration: " << max_acc_norm << " m/s^2 at index " << max_acc_idx << std::endl;
        std::cout << "Time at max acc: " << traj.points[max_acc_idx].time << " (Approx " << max_acc_idx * 0.001 << "s)" << std::endl;
        
        // Print around transition
        // Linear end is at 10.0s -> Index 10000 approx
        // Let's inspect indices around 10000
        size_t transition_idx = static_cast<size_t>(line_time / time_step);
        std::cout << "\n--- Transition Analysis (around t=" << line_time << ") ---" << std::endl;
        for (size_t i = transition_idx - 5; i < transition_idx + 5 && i < traj.points.size(); ++i) {
            std::cout << "Idx " << i << " T=" << traj.points[i].time 
                      << " Pos X=" << traj.points[i].position[0] << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
