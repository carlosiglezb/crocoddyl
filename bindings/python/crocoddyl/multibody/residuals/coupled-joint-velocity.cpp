///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021-2023, University of Edinburgh, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/coupled-joint-velocity.hpp"

#include "python/crocoddyl/multibody/multibody.hpp"
#include "python/crocoddyl/utils/copyable.hpp"

namespace crocoddyl {
namespace python {

void exposeResidualCoupledJointVelocity() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelCoupledJointVelocity> >();

  bp::class_<ResidualModelCoupledJointVelocity, bp::bases<ResidualModelAbstract> >(
      "ResidualModelCoupledJointVelocity",
      "This residual function defines qdot(idx1) = qdot(idx2), with qdot being"
      "the joint velocity and idxi being the index of the joints being \n"
      "coupled (connected), respectively.",
      bp::init<boost::shared_ptr<StateMultibody>, std::size_t,
               pinocchio::Motion, pinocchio::ReferenceFrame, std::size_t>(
          bp::args("self", "state", "ids", "nu"),
          "Initialize the frame velocity residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param residual: residual model\n"
          ":param ids: reference frame ids\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
                    pinocchio::Motion, pinocchio::ReferenceFrame>(
          bp::args("self", "state", "ids"),
          "Initialize the frame velocity residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param residual: residual model\n"
          ":param ids: reference frame ids\n"))
      .def<void (ResidualModelCoupledJointVelocity::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelCoupledJointVelocity::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the frame velocity residual.\n\n"
          ":param data: residual data\n"
          ":param x: state point (dim. state.nx)\n"
          ":param u: control input (dim. nu)")
      .def<void (ResidualModelCoupledJointVelocity::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelCoupledJointVelocity::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelCoupledJointVelocity::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the frame velocity residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: state point (dim. state.nx)\n"
          ":param u: control input (dim. nu)")
      .def<void (ResidualModelCoupledJointVelocity::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModelCoupledJointVelocity::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the frame velocity residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for the frame velocity residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("coupled_j_id_", &ResidualModelCoupledJointVelocity::get_ids,
                    &ResidualModelCoupledJointVelocity::set_ids, "reference frame id")
      .def(CopyableVisitor<ResidualModelCoupledJointVelocity>());

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataCoupledJointVelocity> >();

  bp::class_<ResidualDataCoupledJointVelocity, bp::bases<ResidualDataAbstract> >(
      "ResidualDataCoupledJointVelocity", "Data for frame velocity residual.\n\n",
      bp::init<ResidualModelCoupledJointVelocity*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create frame velocity residual data.\n\n"
          ":param model: frame Velocity residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataCoupledJointVelocity::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data")
      .def(CopyableVisitor<ResidualDataCoupledJointVelocity>());
}

}  // namespace python
}  // namespace crocoddyl
