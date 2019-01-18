#include <white_line_estimator/particle_filter.h>
#include <ros/ros.h>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

ParticleFilter::ParticleFilter(int dimensions, int num_particles, Eigen::VectorXd init_value) {
  using namespace Eigen;
  std::srand((unsigned int)time(0));
  dimensions_ = dimensions;
  num_partcles_ = num_particles;
  Eigen::MatrixXd random_values = (MatrixXd::Random(dimensions_, num_partcles_).cwiseAbs() -
                                   MatrixXd::Ones(dimensions_, num_partcles_) * 0.5) *
                                  0.1;
  states_ = MatrixXd::Ones(dimensions_, num_partcles_);
  for (int i = 0; i < num_partcles_; i++) {
    states_.block(0, i, dimensions_, 1) = init_value;
  }
  states_ = states_ + random_values;
  weights_ = VectorXd::Ones(num_particles) / num_particles;
}

ParticleFilter::ParticleFilter(int dimensions, int num_particles) {
  using namespace Eigen;
  std::srand((unsigned int)time(0));
  dimensions_ = dimensions;
  num_partcles_ = num_particles;
  states_ = MatrixXd::Random(dimensions_, num_partcles_).cwiseAbs();
  weights_ = VectorXd::Ones(num_particles) / num_particles;
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::setWeights(Eigen::VectorXd weights) { weights_ = weights / weights.sum(); }

Eigen::VectorXd ParticleFilter::get_state() {
  using namespace Eigen;
  VectorXd normalized_state = VectorXd::Zero(dimensions_);
  for (int i = 0; i < num_partcles_; i++) {
    VectorXd state = states_.block(0, i, dimensions_, 1);
    normalized_state = normalized_state + state * weights_(i);
  }
  return normalized_state;
}

void ParticleFilter::addSystemNoise(double variance) {
  using namespace Eigen;
  MatrixXd system_noise = MatrixXd::Zero(dimensions_, num_partcles_);
  getNormalDistributionRandomNumbers(system_noise, 0, variance);
  states_ = states_ + system_noise;
}

void ParticleFilter::addSystemNoise(Eigen::VectorXd &control_input, double variance) {
  using namespace Eigen;
  MatrixXd system_noise = MatrixXd::Zero(dimensions_, num_partcles_);
  getNormalDistributionRandomNumbers(system_noise, 0, variance);
  MatrixXd control_input_matrix = MatrixXd::Zero(dimensions_, num_partcles_);
  for (int i = 0; i < num_partcles_; i++) {
    control_input_matrix.block(0, i, dimensions_, 1) = control_input;
  }
  states_ = states_ + system_noise + control_input_matrix;
}

void ParticleFilter::resample(double threshold) {
  if (getEss() <= threshold) {
    using namespace Eigen;
    MatrixXd updated_state = MatrixXd::Zero(dimensions_, num_partcles_);
    VectorXd random_values = VectorXd::Zero(num_partcles_);
    getNormalDistributionRandomNumbers(random_values, 0, 1);
    std::vector<int> indexes(num_partcles_);
    for (int i = 0; i < num_partcles_; i++) {
      double total_weights = 0;
      for (int j = 0; j < weights_.size(); j++) {
        total_weights += weights_(j);
        if (total_weights >= random_values[i]) {
          indexes[i] = i;
        }
      }
    }
    for (int i = 0; i < num_partcles_; i++) {
      updated_state.block(0, i, dimensions_, 1) = states_.block(0, indexes[i], dimensions_, 1);
    }
    states_ = updated_state;
  }
}

void ParticleFilter::getNormalDistributionRandomNumbers(Eigen::MatrixXd &target,
                                                             double average,
                                                             double variance) {
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm(average, variance);
  for (int i = 0; i < target.rows(); i++) {
    for (int j = 0; j < target.cols(); j++) {
      target(i, j) = norm(mt);
    }
  }
}

void ParticleFilter::getNormalDistributionRandomNumbers(Eigen::VectorXd &target,
                                                             double average,
                                                             double variance) {
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm(average, variance);
  for (int i = 0; i < target.size(); i++) {
    target(i) = norm(mt);
  }
}

void ParticleFilter::getUniformDistribution(Eigen::VectorXd &target, double max, double min) {
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_int_distribution<> random_values(0, 10000);
  for (int i = 0; i < target.size(); i++) {
    target(i) = ((double)random_values(mt) / 10000) * (max - min) + min;
  }
}

void ParticleFilter::getUniformDistribution(Eigen::MatrixXd &target, double max, double min) {
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_int_distribution<> random_values(0, 10000);
  for (int i = 0; i < target.rows(); i++) {
    for (int j = 0; j < target.cols(); j++) {
      target(i, j) = ((double)random_values(mt) / 10000) * (max - min) + min;
    }
  }
}