
#include "resim_core/visualization/view.hh"

#include <glog/logging.h>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/client/view_client_libcurl.hh"
#include "resim_core/visualization/view_client.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization {

namespace {
using transforms::FSE3;
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
}  // namespace

View::View() {
  client_ = std::make_unique<LibcurlClient>("http://api.resim.ai:8080");
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
  {
    // In a separate scope since flush() needs this lock
    std::lock_guard<std::mutex> guard{primitives_mutex_};

    // We simply push the primitivies in here without packing
    // them. The whole ViewUpdate will be packed by the ViewClient.
    primitives_.emplace_back(ViewPrimitive{
        .id = UUID::new_uuid(),
        .payload = subject,
    });
  }
  // Currently, we pack each primitive into its own update and send eagerly to
  // the view server.
  flush();
  return *this;
}

template View &View::operator<< <SE3>(const SE3 &subject);

template View &View::operator<< <SO3>(const SO3 &subject);

template View &View::operator<< <FSE3>(const FSE3 &subject);

template View &View::operator<< <FSO3>(const FSO3 &subject);

template View &View::operator<< <curves::DCurve<SE3>>(
    const curves::DCurve<SE3> &subject);

template View &View::operator<< <curves::DCurve<FSE3>>(
    const curves::DCurve<FSE3> &subject);

template View &View::operator<< <curves::TCurve<FSE3>>(
    const curves::TCurve<FSE3> &subject);

template View &View::operator<< <actor::state::Trajectory>(
    const actor::state::Trajectory &subject);

void View::flush() {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  REASSERT(client_ != nullptr, "No client interface set!");
  Status send_status = client_->send_view_update(ViewUpdate{
      .primitives = std::move(primitives_),
  });
  CHECK_STATUS_OK(send_status);
  primitives_.clear();
}

void View::set_client(std::unique_ptr<ViewClient> &&client) {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  primitives_.clear();
  client_ = std::move(client);
}

}  // namespace resim::visualization
