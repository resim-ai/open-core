

#include "resim/visualization/log/visualize_world_glb.hh"

#include <foxglove/SceneUpdate.pb.h>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>

#include "resim/simulator/standard_frames.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"
#include "resim/visualization/foxglove/vector_to_foxglove.hh"

namespace resim::visualization::log {

void visualize_world_glb(
    const std::filesystem::path &world_glb_path,
    const time::Timestamp &time,
    const std::string &channel_name,
    InOut<LoggerInterface> logger) {
  std::ifstream glb_stream;
  REASSERT(std::filesystem::exists(world_glb_path));
  glb_stream.open(world_glb_path, std::ios::binary);

  ::foxglove::SceneUpdate update;
  auto &entity = *update.add_entities();
  time::proto::pack(time, entity.mutable_timestamp());
  entity.set_frame_id(std::string(simulator::SCENE_FRAME_NAME));
  entity.set_id("world");
  auto &model = *entity.add_models();
  foxglove::pack_into_foxglove(
      transforms::SE3::identity(),
      model.mutable_pose());
  foxglove::pack_into_foxglove(Eigen::Vector3d::Ones(), model.mutable_scale());
  model.set_media_type("model/gltb-binary");

  std::stringstream sstream;
  sstream << glb_stream.rdbuf();
  model.set_data(sstream.str());

  logger->add_proto_channel<::foxglove::SceneUpdate>(channel_name);
  logger->log_proto(channel_name, time, update);
}

}  // namespace resim::visualization::log
