///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021-2022, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include "crocoddyl/multibody/residuals/coupled-joint-velocity.hpp"

namespace crocoddyl {

template <typename Scalar>
ResidualModelCoupledJointVelocityTpl<Scalar>::ResidualModelCoupledJointVelocityTpl(
    boost::shared_ptr<StateMultibody> state,
    const std::vector<pinocchio::FrameIndex> ids,
    const std::size_t nu)
    : Base(state, 2, nu, false, true, false),
      pin_model_(state->get_pinocchio()) {
  // TODO check dimension of ids == 2
  for (const auto &id : ids) {
    coupled_j_id_.push_back(id);
  }
  if ((static_cast<pinocchio::FrameIndex>(state->get_pinocchio()->nframes) <=
      ids[0]) || (static_cast<pinocchio::FrameIndex>(state->get_pinocchio()->nframes) <=
                  ids[1])) {
    throw_pretty(
        "Invalid argument: "
        << "one of the frame indices is wrong (it does not exist in the robot)");
  }
}

template <typename Scalar>
ResidualModelCoupledJointVelocityTpl<Scalar>::ResidualModelCoupledJointVelocityTpl(
    boost::shared_ptr<StateMultibody> state,
    const std::vector<pinocchio::FrameIndex> ids)
    : Base(state, 2, false, true, false),
      pin_model_(state->get_pinocchio()) {
  // TODO check dimension of ids == 2
  for (const auto &id : ids) {
    coupled_j_id_.push_back(id);
  }
  if ((static_cast<pinocchio::FrameIndex>(state->get_pinocchio()->nframes) <=
       ids[0]) || (static_cast<pinocchio::FrameIndex>(state->get_pinocchio()->nframes) <=
                   ids[1])) {
    throw_pretty(
        "Invalid argument: "
        << "the frame index is wrong (it does not exist in the robot)");
  }
}

template <typename Scalar>
ResidualModelCoupledJointVelocityTpl<Scalar>::~ResidualModelCoupledJointVelocityTpl() {}

template <typename Scalar>
void ResidualModelCoupledJointVelocityTpl<Scalar>::calc(
    const boost::shared_ptr<ResidualDataAbstract>& data,
    const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Compute the coupled joint velocity difference
  auto v1 = *d->pinocchio->v[coupled_j_id_[0]];
  auto v2 = *d->pinocchio->v[coupled_j_id_[1]];
  data->r = v1 - v2;
}

template <typename Scalar>
void ResidualModelCoupledJointVelocityTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract>& data,
    const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&) {
  // Get the partial derivatives of the local frame velocity
  Data* d = static_cast<Data*>(data.get());
  const std::size_t nv = state_->get_nv();
  data->Rx[0, coupled_j_id_[0] - 1] = 1;
  data->Rx[1, coupled_j_id_[1] - 1] = -1;
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> >
ResidualModelCoupledJointVelocityTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this,
                                      data);
}

template <typename Scalar>
void ResidualModelCoupledJointVelocityTpl<Scalar>::print(std::ostream& os) const {
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[",
                            "]");
  os << "ResidualModelCoupledJointVelocity {frame=" << pin_model_->frames[coupled_j_id_].name;
}

template <typename Scalar>
std::vector<pinocchio::FrameIndex> ResidualModelCoupledJointVelocityTpl<Scalar>::get_ids() const {
  return coupled_j_id_;
}

template <typename Scalar>
void ResidualModelCoupledJointVelocityTpl<Scalar>::set_ids(
    const std::vector<pinocchio::FrameIndex> ids) {
  // TODO check dimension of ids == 2
  for (const auto &id : ids) {
    coupled_j_id_.push_back(id);
  }
}

}  // namespace crocoddyl
