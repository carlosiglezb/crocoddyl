///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_RESIDUALS_COUPLED_JOINT_VELOCITY_HPP_
#define CROCODDYL_MULTIBODY_RESIDUALS_COUPLED_JOINT_VELOCITY_HPP_

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/motion.hpp>

#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

namespace crocoddyl {

/**
 * @brief Coupled joint velocity residual
 *
 * This residual function defines a tracking of frame velocity as
 * TODO update
 * \f$\mathbf{r}=\mathbf{v}-\mathbf{v}^*\f$, where
 * \f$\mathbf{v},\mathbf{v}^*\in~T_{\mathbf{p}}~\mathbb{SE(3)}\f$ are the
 * current and reference frame velocities, respectively. Note that the tangent
 * vector is described by the frame placement \f$\mathbf{p}\f$, and the
 * dimension of the residual vector is 6. Furthermore, the Jacobians of the
 * residual function are computed analytically.
 *
 * As described in `ResidualModelAbstractTpl`, the residual vector and its
 * Jacobians are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelCoupledJointVelocityTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataCoupledJointVelocityTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;

  /**
   * @brief Initialize the frame velocity residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] id          Reference frame id
   * @param[in] nu          Dimension of the control vector
   */
  ResidualModelCoupledJointVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                const std::vector<pinocchio::FrameIndex> ids,
                                const std::size_t nu);

  /**
   * @brief Initialize the frame velocity residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state       State of the multibody system
   * @param[in] id          Reference frame id
   * LOCAL_WORLD_ALIGNED)
   */
  ResidualModelCoupledJointVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                const std::vector<pinocchio::FrameIndex> ids);
  virtual ~ResidualModelCoupledJointVelocityTpl();

  /**
   * @brief Compute the frame velocity residual vector
   *
   * @param[in] data  Frame velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the Jacobians of the frame velocity residual
   *
   * @param[in] data  Frame velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Create the frame velocity residual data
   */
  virtual boost::shared_ptr<ResidualDataAbstract> createData(
      DataCollectorAbstract* const data);

  /**
   * @brief Return the reference frame id
   */
  std::vector<pinocchio::FrameIndex> get_ids() const;

  /**
   * @brief Modify reference frame id
   */
  void set_ids(const std::vector<pinocchio::FrameIndex> ids);

  /**
   * @brief Print relevant information of the frame-velocity residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nr_;
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;

 private:
  std::vector<pinocchio::FrameIndex> coupled_j_id_;        //!< Reference frame id
  boost::shared_ptr<typename StateMultibody::PinocchioModel>
      pin_model_;  //!< Pinocchio model
};

template <typename _Scalar>
struct ResidualDataCoupledJointVelocityTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  template <template <typename Scalar> class Model>
  ResidualDataCoupledJointVelocityTpl(Model<Scalar>* const model,
                               DataCollectorAbstract* const data)
      : Base(model, data) {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d =
        dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/residuals/coupled-joint-velocity.hxx"

#endif  // CROCODDYL_MULTIBODY_RESIDUALS_COUPLED_JOINT_VELOCITY_HPP_
