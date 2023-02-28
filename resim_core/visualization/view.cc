
#include "resim_core/visualization/view.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/client/view_client_libcurl.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization {

View::View() {
  client_ = std::make_unique<LibcurlClient>("http://api.resim.ai:8080");
}

View &View::get_instance() {
  // Meyer's Singleton
  static View view;
  return view;
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

void View::set_client(std::unique_ptr<ViewClient> &&client) {
  std::lock_guard<std::mutex> guard{primitives_mutex_};
  primitives_.clear();
  client_ = std::move(client);
}

}  // namespace resim::visualization
