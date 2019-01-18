#ifndef PARTICLE_FILTER_H_INCLUDED
#define PARTICLE_FILTER_H_INCLUDED

//headers in Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//headers in STL
#include <functional>

class ParticleFilter {
 public:
  ParticleFilter(int dimensions, int num_particles, Eigen::VectorXd init_value);
  ParticleFilter(int dimensions, int num_particles);
  ~ParticleFilter();
  Eigen::VectorXd getState();
  Eigen::MatrixXd getStates() { return states_; };
  void setWeights(Eigen::VectorXd weights);
  void addSystemNoise(Eigen::VectorXd &control_input, double variance);
  void addSystemNoise(double variance);
  void resample(double threshold);
 private:
  double getEss() { return 1 / weights_.cwiseAbs2().sum(); };
  int dimensions_;
  int num_partcles_;
  Eigen::MatrixXd states_;
  Eigen::VectorXd weights_;
  void getNormalDistributionRandomNumbers(Eigen::MatrixXd &target, double average, double variance);
  void getNormalDistributionRandomNumbers(Eigen::VectorXd &target, double average, double variance);
  void getUniformDistribution(Eigen::MatrixXd &target, double max, double min);
  void getUniformDistribution(Eigen::VectorXd &target, double max, double min);
};
#endif // PARTICLE_FILTER_H_INCLUDED