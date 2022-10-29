#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace resim::geometry {

//
// This class represents a simple wireframe represented by a vector of points
// and a vector of point index pairs representing edges in the wireframe. We may
// eventually upgrade this to include trigs as well and be a full 3D Mesh
// representation.
// TODO (https://app.asana.com/0/1202178773526279/1203257064495359/f) Add trig
// faces
//
class Wireframe {
 public:
  using Point = Eigen::Matrix<double, 3, 1>;
  using Edge = std::array<std::size_t, 2>;

  // Construct a wireframe from points and edges.
  // @parma[in] points - a vector of points present in the wireframe.
  // @param[in] edges - a vector of edges present in the wireframe.
  Wireframe(std::vector<Point> points, std::vector<Edge> edges);

  // Default constructor
  Wireframe() = default;

  // Check whether this wireframe is valid meaning that every edge is
  // between two distinct points in the vector of points.
  bool is_valid() const;

  // Adders
  void add_point(const Point &point);
  void add_edge(const Edge &edge);

  // Getters
  const std::vector<Point> &points() const;
  const std::vector<Edge> &edges() const;

 private:
  std::vector<Point> points_;
  std::vector<Edge> edges_;
};

}  // namespace resim::geometry
