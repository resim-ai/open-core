#include "resim_core/geometry/oriented_box.hh"

#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::geometry {

using transforms::LieGroupType;

template <LieGroupType Group>
OrientedBox<Group>::OrientedBox(Group reference_from_box, ExtentsType extents)
    : reference_from_box_{std::move(reference_from_box)} {
  set_extents(std::move(extents));
}

template <LieGroupType Group>
const Group &OrientedBox<Group>::reference_from_box() const {
  return reference_from_box_;
}

template <LieGroupType Group>
const typename OrientedBox<Group>::ExtentsType &OrientedBox<Group>::extents()
    const {
  return extents_;
}

template <LieGroupType Group>
void OrientedBox<Group>::set_reference_from_box(Group reference_from_box) {
  reference_from_box_ = std::move(reference_from_box);
}

template <LieGroupType Group>
void OrientedBox<Group>::set_extents(ExtentsType extents) {
  REASSERT((extents.array() > 0.).all(), "Negative extent detected!");
  extents_ = std::move(extents);
}

template class OrientedBox<transforms::SE3>;

}  // namespace resim::geometry
