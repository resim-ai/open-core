// This file is a companion for the Gaussian documentation at
// https://docs.resim.ai/math/gaussian/

#include <fmt/core.h>

#include <Eigen/Dense>
#include <fstream>

#include "resim/math/multivariate_gaussian.hh"

int main(int argc, char **argv) {
  using resim::math::Gaussian;
  using Vec = Gaussian::Vec;
  using Mat = Gaussian::Mat;

  // Set up our Gaussian sampler
  Vec mean = Vec::Zero(2);
  mean << 20., 2.;

  Mat covariance = Mat::Zero(2, 2);
  covariance << 4.8, -0.96, -0.96, 0.25;

  Gaussian gaussian{mean, covariance};

  // Sample the distribution:
  Gaussian::SamplesMat samples = gaussian.samples(1000);

  // Write our data out
  std::ofstream output;
  output.open("gaussian_samples.csv");
  for (const auto &sample : samples.rowwise()) {
    output << fmt::format("{0}, {1}", sample.x(), sample.y()) << std::endl;
  }
  output.close();
  return EXIT_SUCCESS;
}
