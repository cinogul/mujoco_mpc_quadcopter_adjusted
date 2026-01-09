// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/quadrotor/quadrotor.h"

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
std::string Quadrotor::XmlPath() const {
  return GetModelPath("quadrotor/task.xml");
}
std::string Quadrotor::Name() const { return "Quadrotor"; }

// --------------- Residuals for quadrotor task ---------------
//   Number of residuals: 4
//     Residual (0): position - goal position
//     Residual (1): linear velocity - goal linear velocity
//     Residual (2): angular velocity - goal angular velocity
//     Residual (3): control - hover control
//   Number of parameters: 6
// ------------------------------------------------------------
void Quadrotor::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residuals) const {
  // ---------- Residual (0) ----------
  double* position = SensorByName(model, data, "position");
  double* goal_position = SensorByName(model, data, "goal_position");
  mju_sub(residuals, position, goal_position, 3);

  // ---------- Residual (1) ----------
  double* linear_velocity = SensorByName(model, data, "linear_velocity");
  mju_copy(residuals + 3, linear_velocity, 3);

  // ---------- Residual (2) ----------
  double* angular_velocity = SensorByName(model, data, "angular_velocity");
  mju_copy(residuals + 6, angular_velocity, 3);

  // ---------- Residual (3) ----------
  double thrust = (model->body_mass[0] + model->body_mass[1]) *
                  mju_norm3(model->opt.gravity) / model->nu;
  for (int i = 0; i < model->nu; i++) {
    residuals[9 + i] = data->ctrl[i] - thrust;
  }

  // // ---------- Residual (4): Penalize moving (heading) toward fixed point ----------
  // // Compute angle between velocity vector and vector to obstacle; penalize
  // // when the angle is small (i.e., drone is heading toward the obstacle).
  // double obstacle_pos[3] = {2.5, 0.0, 0.75};
  // double obs_vec[3];
  // mju_sub3(obs_vec, obstacle_pos, position);  // vector from drone -> obstacle
  // double obs_dist = mju_norm3(obs_vec);

  // // Parameters
  // const double influence_distance = 3.0;  // start penalizing inside this radius
  // const double penalty_scale = 5.0;       // overall weight
  // const double eps = 1e-6;

  // double angle_penalty = 0.0;
  // if (obs_dist > eps) {
  //   double vel_norm = mju_norm3(linear_velocity);
  //   if (vel_norm > eps && obs_dist < influence_distance) {
  //     // cosine of angle between velocity and obs_vec
  //     double cos_theta = mju_dot3(linear_velocity, obs_vec) / (vel_norm * obs_dist + eps);
  //     // only penalize when cos_theta > 0 (i.e., velocity has a component toward obstacle)
  //     double toward = mju_max(0.0, cos_theta);
  //     // influence falls off with distance
  //     double influence = (influence_distance - obs_dist) / influence_distance;
  //     influence = mju_max(0.0, mju_min(1.0, influence));
  //     // penalize more if moving faster toward the obstacle
  //     angle_penalty = penalty_scale * toward * vel_norm * influence;

  //     // --- Side bias: encourage passing to one side ---
  //     // side_sign: +1 => prefer left (positive Y), -1 => prefer right (negative Y)
  //     const double side_sign = 1.0;         // tweak to choose side
  //     const double side_gain = 1.0;         // how strongly to bias to chosen side
  //     // perpendicular direction to obs_vec in XY plane
  //     double perp_x = -obs_vec[1];
  //     double perp_y = obs_vec[0];
  //     double perp_norm = sqrt(perp_x * perp_x + perp_y * perp_y) + eps;
  //     perp_x /= perp_norm;
  //     perp_y /= perp_norm;
  //     // velocity component along perpendicular (lateral motion)
  //     double lateral_vel = linear_velocity[0] * perp_x + linear_velocity[1] * perp_y;
  //     // side bias reduces the residual when the drone has lateral motion in desired direction
  //     double side_bias = side_gain * influence * side_sign * lateral_vel;
  //     // subtract bias so moving in desired lateral direction lowers the penalty
  //     angle_penalty -= side_bias;
  //   }
  // }
  // residuals[9 + model->nu] = angle_penalty;
}


// ----- Reset for quadrotor task -----
void Quadrotor::ResetLocked(const mjModel* model) {
  current_mode_ = 0;
}

// ----- Transition for quadrotor task -----
void Quadrotor::TransitionLocked(mjModel* model, mjData* data) {
   // set mode to GUI selection
  if (mode > 0) {
    current_mode_ = mode - 1;
  } else {
    // Only advance through keyframes if not at the last one
    if (current_mode_ < model->nkey - 1) {
      // goal position
      const double* goal_position = data->mocap_pos;

      // system's position
      double* position = SensorByName(model, data, "position");

      // position error
      double position_error[3];
      mju_sub3(position_error, position, goal_position);
      double position_error_norm = mju_norm3(position_error);

      if (position_error_norm <= 5.0e-1) {
        // update task state to move to next waypoint
        current_mode_ += 1;
      }
      
      // Only print when reaching the final waypoint
      if (current_mode_ == model->nkey - 1 && position_error_norm <= 5.0e-1) {
        if (!goal_reached_) {
          goal_reached_ = true;
          mjtNum sim_time = data->time;
          mju_printMat(&sim_time, 1, 1);
        }
      }
    }
    // If at the last keyframe, current_mode_ stays the same (hover around final goal)
  }

  // set goal
  mju_copy3(data->mocap_pos, model->key_mpos + 3 * current_mode_);
  mju_copy4(data->mocap_quat, model->key_mquat + 4 * current_mode_);
}

}  // namespace mjpc
