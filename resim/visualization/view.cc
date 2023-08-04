// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/view.hh"

#include <glog/logging.h>

#include <memory>

#include "resim/actor/state/trajectory.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/d_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/status.hh"
#include "resim/visualization/client/view_client.hh"
#include "resim/visualization/view_update.hh"

namespace resim::visualization {

namespace {
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;
using FramedVector = transforms::FramedVector<3>;
constexpr auto UNKNOWN_FILE = "Unknown file";
}  // namespace

View::View() {
  client_ = std::make_unique<ViewClient>("https://api.resim.ai/v1");
}

// We need this since we're using the PIMPL idiom so this must appear after
// ViewClient is declared.
// NOLINTNEXTLINE(modernize-use-equals-default)
View::~View() {}

View &View::get_instance() {
  // Meyer's Singleton
  static View view;
  return view;
}

template <typename T>
View &View::operator<<(const T &subject) {
  // Generate a ViewObject without a name, then view it. Since we don't use a
  // macro, we cannot supply an accurate file name and line number.
  const ViewObject<T> &the_view_object{subject, UNKNOWN_FILE, 0};
  view_object(the_view_object);
  return *this;
}

template View &View::operator<< <Frame>(const Frame &subject);

template View &View::operator<< <SE3>(const SE3 &subject);

template View &View::operator<< <SO3>(const SO3 &subject);

template View &View::operator<< <curves::DCurve<SE3>>(
    const curves::DCurve<SE3> &subject);

template View &View::operator<< <curves::TCurve<SE3>>(
    const curves::TCurve<SE3> &subject);

template View &View::operator<< <actor::state::Trajectory>(
    const actor::state::Trajectory &subject);

template View &View::operator<< <FramedVector>(const FramedVector &subject);

template <typename T>
void View::view_object(ViewObject<T> view_object) {
  {
    // In a separate scope since flush() needs this lock
    std::lock_guard<std::mutex> guard{primitives_mutex_};

    // We simply push the primitivies in here without packing
    // them. The whole ViewUpdate will be packed by the ViewClient.
    // The user defined name is an std::optional so the existence
    // of a value i.e. a custom name is not cared about here.
    primitives_.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = view_object.the_object,
        .user_defined_name = view_object.user_defined_name,
        .file_name = view_object.file_name,
        .line_number = view_object.line_number,
    });
  }
  // Currently, we pack each primitive into its own update and send eagerly to
  // the view server.
  flush();
}

void View::flush() {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  REASSERT(client_ != nullptr, "No client interface set!");
  Status send_status = client_->send_view_update(ViewUpdate{
      .primitives = std::move(primitives_),
  });
  CHECK_STATUS_OK(send_status);
  primitives_.clear();
}

void View::set_client(std::unique_ptr<ViewClientInterface> &&client) {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  primitives_.clear();
  client_ = std::move(client);
}

// Implementation for ViewObject
template <typename T>
ViewObject<T>::ViewObject(T object, const char *file_name, int line_number)
    : the_object(std::move(object)),
      file_name(file_name),
      line_number(line_number) {}

template <typename T>
ViewObject<T>::ViewObject(
    T object,
    std::string name,
    const char *file_name,
    int line_number)
    : the_object(std::move(object)),
      user_defined_name(std::move(name)),
      file_name(file_name),
      line_number(line_number) {
  // Eagerly view the object, since we have all the required information
  resim::view.view_object(*this);
}

// Implementation of the ViewObject steaming operator
template <typename T>
void ViewObject<T>::operator<<(const std::string &name) {
  user_defined_name = name;
  resim::view.view_object(*this);
}

template struct ViewObject<transforms::Frame<3>>;
template struct ViewObject<transforms::SE3>;
template struct ViewObject<transforms::SO3>;
template struct ViewObject<curves::DCurve<transforms::SE3>>;
template struct ViewObject<curves::TCurve<transforms::SE3>>;
template struct ViewObject<actor::state::Trajectory>;
template struct ViewObject<transforms::FramedVector<3>>;

}  // namespace resim::visualization
