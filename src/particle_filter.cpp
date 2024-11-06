/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <angles/angles.h>

#include <dt_ndt_mcl/particle_filter.hpp>
#include <random>
#include <omp.h>

namespace ndt_2d {

ParticleFilter::ParticleFilter(size_t min_particles, size_t max_particles,
                               MotionModelPtr& motion_model)
    : gen_(random_()),
      motion_model_(motion_model),
      min_particles_(min_particles),
      max_particles_(max_particles),
      mean_(Eigen::Vector3d::Zero()),
      pose_cov(Eigen::Matrix<double, 6, 6>::Zero()),
      cov_(Eigen::Matrix3d::Zero()),
      kd_tree_(0.5, 0.5, 0.2671, max_particles),
      corrected_pose_(0.0) {
  particles_.reserve(max_particles_);
  weights_.reserve(max_particles_);
  particles_.assign(min_particles_, Particle(0.0, 0.0, 0.0));
  weights_.assign(min_particles_, 1.0 / min_particles_);
  updateStatistics();
}

void ParticleFilter::init(const double x, const double y, const double theta,
                          const double sigma_x, const double sigma_y,
                          const double sigma_theta) {
  std::normal_distribution<float> sample_x(x, sigma_x);
  std::normal_distribution<float> sample_y(y, sigma_y);
  std::normal_distribution<float> sample_theta(theta, sigma_theta);

  for (auto& particle : particles_) {
    particle(0) = sample_x(gen_);
    particle(1) = sample_y(gen_);
    particle(2) = angles::normalize_angle(sample_theta(gen_));
  }

  weights_.assign(particles_.size(), 1.0 / particles_.size());
  updateStatistics();
}
void ParticleFilter::setCorrectedPose(Eigen::Vector3d &pose, double weight){
    std::random_device rd;                             // Seed for randomness
    std::mt19937 gen(rd());                            // Mersenne Twister engine
    std::uniform_int_distribution<> distrib(0, min_particles_ - 1);

    // Select a random index
    int random_index = distrib(gen);

    corrected_pose_ = random_index;


    std::cout<<"random_index: "<<random_index<<std::endl;

    particles_[random_index] = pose;
}
void ParticleFilter::update(const double dx, const double dy,
                            const double dth) {
  // Apply control update
  motion_model_->sample(dx, dy, dth, particles_);
  updateStatistics();
}

double ParticleFilter::computeESS() {
  double ess = 0.0;
  for (auto& w : weights_) {
    ess += w * w;
  }
  return 1.0 / ess;
}

double ParticleFilter::computeVar(){
  double sum_weight = 0.0;
  for (auto& w : weights_) {
    sum_weight += w;
  }
  double mean_weight = sum_weight / weights_.size();
  double var_weight = 0.0;
  for (auto& w : weights_) {
    var_weight += (w - mean_weight) * (w - mean_weight);
  }
  var_weight /= weights_.size();
  return var_weight;

}

bool ParticleFilter::initiateResampling(double threshold, size_t ess_or_var){
  if (ess_or_var == 0){
    return computeESS() < particles_.size();
  }
  else{
    return computeVar() > threshold;
  }
  
}
Eigen::Vector3d ParticleFilter::generateRandomParticle(double delta_x, double delta_y, double delta_theta){
  std::uniform_real_distribution<double> uniform_dist_x_(mean_(0) - delta_x, mean_(0) + delta_x);
  std::uniform_real_distribution<double>uniform_dist_y_(mean_(1) - delta_y, mean_(1) + delta_y);
  std::uniform_real_distribution<double>uniform_dist_theta_(mean_(2) - delta_theta, mean_(2) + delta_theta);

  Eigen::Vector3d p;
  p(0) = uniform_dist_x_(gen_);
  p(1) = uniform_dist_y_(gen_);
  p(2) = uniform_dist_theta_(gen_);
  return p;
}
void ParticleFilter::injectRandomParticles(std::vector<Particle> resampled, std::vector<double> resampled_weights){
  std::cout<<"Injecting random particles"<<std::endl;
  int num_particles = resampled.size();
  int num_random_particles = static_cast<int>(num_particles * 0.1);
  for(int i =0 ; i < num_random_particles; i++){
    Eigen::Vector3d p;
    p = generateRandomParticle(0.25, 0.25, 0.1);
    resampled.push_back(p);
    resampled_weights.push_back(1.0 / num_particles);

  }
  particles_ = resampled;
  weights_ = resampled_weights;
}
void ParticleFilter::measure(const ScanMatcherPtr& matcher,
                             const ScanPtr& scan) {
  #pragma omp parallel for 
  for (size_t i = 0; i < particles_.size(); ++i) {
    // Pose of this particle in NDT format
    Pose2d pose(particles_[i](0), particles_[i](1), particles_[i](2));
    // Compute the score, ignoring the scan->pose
    weights_[i] = matcher->scorePoints(scan->getPoints(), pose);
  }
  updateStatistics();
}

void ParticleFilter::resample(const double kld_err, const double kld_z, bool inject_particles) {
  // Sampling particles based on current weights
  std::discrete_distribution<size_t> d(weights_.begin(), weights_.end());

  // Re-initialize all of the bins to empty
  kd_tree_.clear();

  std::vector<Particle> resampled;
  resampled.reserve(max_particles_);

  std::vector<double> resampled_weights;
  resampled_weights.reserve(max_particles_);

  // This will be the crazy equation from Probabilistic Robotics
  size_t Mx = max_particles_;

  while (resampled.size() < std::max(min_particles_, Mx)) {
    // Sample a particle - NOTE: KLD paper says we should apply motion model &
    // update here?
    size_t p = d(gen_);
    kd_tree_.insert(particles_[p], weights_[p]);
    resampled.push_back(particles_[p]);
    resampled_weights.push_back(weights_[p]);

    // Recompute crazy equation
    size_t k = kd_tree_.getLeafCount();
    if (k > 1) {
      double a = (k - 1) / (2.0 * kld_err);
      double b = 2.0 / (9.0 * (k - 1));
      double c = 1.0 - b + std::sqrt(b) * kld_z;
      Mx = a * c * c * c;
    }

    // Never exceed max particles
    if (resampled.size() >= max_particles_) {
      break;
    }
    // injectRandomParticles();
  }


  if(inject_particles){
    injectRandomParticles(resampled, resampled_weights);
  }
  else{
    particles_ = resampled;
    weights_ = resampled_weights;
  }

  updateStatistics();
}

Eigen::Vector3d ParticleFilter::getMean() { return mean_; }

Eigen::Matrix3d ParticleFilter::getCovariance() { return cov_; }

Eigen::Matrix<double, 6, 6> ParticleFilter::getPoseCovariance() { return pose_cov; }

void ParticleFilter::getMsg(geometry_msgs::msg::PoseArray & msg) {
  msg.poses.reserve(particles_.size());
  for (auto& particle : particles_) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particle(0);
    pose.position.y = particle(1);
    pose.orientation.z = sin(particle(2) / 2.0);
    pose.orientation.w = cos(particle(2) / 2.0);
    msg.poses.push_back(pose);
  }
}

void ParticleFilter::updateStatistics() {
  // Recompute weights to sum to 1.0
  double sum_weight = 0.0;
  for (auto& w : weights_) {
    sum_weight += w;
  }
  for (auto& w : weights_) {
    w /= sum_weight;
  }

  // Temporary mean and correlation
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d corr = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 6,6> pose_covariance;
  pose_covariance.setZero();
  // Use circular mean for theta
  double sum_cos_th = 0.0, sum_sin_th = 0.0;

  for (size_t i = 0; i < particles_.size(); ++i) {
    mean += weights_[i] * particles_[i];
    sum_cos_th += weights_[i] * cos(particles_[i](2));
    sum_sin_th += weights_[i] * sin(particles_[i](2));

    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = j; k < 2; ++k) {
        corr(j, k) += weights_[i] * particles_[i](j) * particles_[i](k);
      }
    }
  }

  // Mean is already normalized, since weights were normalized
  mean_(0) = mean(0);
  mean_(1) = mean(1);
  mean_(2) = atan2(sum_sin_th, sum_cos_th);

  // Compute covariance for x/y
  for (size_t j = 0; j < 2; ++j) {
    for (size_t k = j; k < 2; ++k) {
      cov_(j, k) = corr(j, k) - mean(j) * mean(k);
      cov_(k, j) = cov_(j, k);
    }
  }
  cov_(2, 2) = 0.0;

  // Compute covariance for theta
  for (size_t i = 0; i < particles_.size(); ++i) {
    double d = angles::shortest_angular_distance(particles_[i](2), mean_(2));
    cov_(2, 2) += weights_[i] * d * d;
  }

for (size_t i = 0; i < particles_.size(); ++i) {
  double dx = particles_[i](0)  - mean(0);
  double dy = particles_[i](1) - mean(1);
  pose_covariance(0, 0) += weights_[i] * dx * dx;
  pose_covariance(1, 1) += weights_[i] * dy * dy;
  pose_covariance(0, 1) += weights_[i] * dx * dy;
  pose_covariance(1, 0) = pose_covariance(0, 1);  // Symmetric covariance matrix

  // Theta differences
  double dtheta = angles::shortest_angular_distance(particles_[i](2) , mean(2));
  pose_covariance(2, 2) += weights_[i] * dtheta * dtheta;

  // Cross-covariance terms for theta (x-theta, y-theta)
  pose_covariance(0, 2) += weights_[i] * dx * dtheta;
  pose_covariance(1, 2) += weights_[i] * dy * dtheta;
  pose_covariance(2, 0) = pose_covariance(0, 2);  // Symmetric
  pose_covariance(2, 1) = pose_covariance(1, 2);  // Symmetric
}

pose_cov = pose_covariance;
}

}  // namespace ndt_2d