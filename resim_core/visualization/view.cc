
#include "resim_core/visualization/view.hh"

#include <glog/logging.h>

#include "resim_core/utils/status.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization {

View::View() : client_{nullptr} {
  // TODO(https://app.asana.com/0/1203751014069901/1203791103932376/f) Set a
  // default client implementation once one is available
}

View &View::get_instance() {
  // Meyer's Singleton
  static View view;
  return view;
}

void View::flush() {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  CHECK(client_ != nullptr) << "No client interface set!";
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
