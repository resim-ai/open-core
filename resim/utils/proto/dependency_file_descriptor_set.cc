// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/proto/dependency_file_descriptor_set.hh"

#include <queue>
#include <unordered_map>

#include "resim/assert/assert.hh"

namespace resim {

std::string dependency_file_descriptor_set(
    const google::protobuf::Descriptor &root) {
  // Perform a breadth-first-search, storing the dependencies we have
  // already visited by name in the dependencies map so we know we
  // don't need to search them if we encounter them again.
  std::unordered_map<std::string, const google::protobuf::FileDescriptor *>
      dependencies;
  dependencies.emplace(root.file()->name(), root.file());
  std::queue<const google::protobuf::FileDescriptor *> descriptor_queue{};
  descriptor_queue.push(root.file());

  while (not descriptor_queue.empty()) {
    const google::protobuf::FileDescriptor *const current =
        descriptor_queue.front();
    REASSERT(current != nullptr, "Invalid FileDescriptor!");
    for (int ii = 0; ii < current->dependency_count(); ++ii) {
      const google::protobuf::FileDescriptor *const dependency{
          current->dependency(ii)};
      if (not dependencies.contains(dependency->name())) {
        dependencies.emplace(dependency->name(), dependency);
        descriptor_queue.push(dependency);
      }
    }
    descriptor_queue.pop();
  }

  google::protobuf::FileDescriptorSet descriptor_set;
  for (const auto &[name, descriptor] : dependencies) {
    descriptor->CopyTo(descriptor_set.add_file());
  }

  return descriptor_set.SerializeAsString();
}
}  // namespace resim
