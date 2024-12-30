#include "generation/ConvexMPC.h"
#include <pinocchio/Orientation.h>
#include <rcpputils/asserts.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

ConvexMPC::ConvexMPC(Node::SharedPtr nodeHandle,
                     std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
                     std::shared_ptr<ReferenceBuffer> referenceBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      referenceBuffer_(referenceBuffer) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  base_name = config_["model"]["base_name"].as<std::string>();
  scalar_t freq_ = config_["generation"]["frequency"].as<scalar_t>();
  dt_ = 0.02;

  total_mass_ = pinocchioInterface_ptr_->total_mass();
  weight_.setZero(12, 12);
  weight_.diagonal() << 10, 10, 40, 0.2, 0.2, 0.1, 4, 4, 4, 0.1, 0.1, 0.1;

  solver_settings.mode = hpipm::HpipmMode::Speed;
  solver_settings.iter_max = 30;
  solver_settings.alpha_min = 1e-8;
  solver_settings.mu0 = 1e2;
  solver_settings.tol_stat = 1e-04;
  solver_settings.tol_eq = 1e-04;
  solver_settings.tol_ineq = 1e-04;
  solver_settings.tol_comp = 1e-04;
  solver_settings.reg_prim = 1e-12;
  solver_settings.pred_corr = 1;
  solver_settings.ric_alg = 0;
  solver_settings.split_step = 1;

  vel_cmd.setZero();
  yawd_ = 0.0;
  t0 = nodeHandle->now().seconds();
  RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"), "ConvexMPC: Construction done");
}

ConvexMPC::~ConvexMPC() {}

void ConvexMPC::setVelCmd(vector3_t vd, scalar_t yawd) {
  vel_cmd = vd;
  yawd_ = yawd;
}

void ConvexMPC::setHeightCmd(scalar_t h) { h_des = h; }

void ConvexMPC::generateTrajRef() {
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto base_pose_m = pinocchioInterface_ptr_->getFramePose(base_name);

  if (first_run) {
    first_run = false;
    pos_start = base_pose_m.translation();
    pos_start.z() = h_des;
    rpy_start = toEulerAngles(base_pose_m.rotation());
    rpy_start.head(2).setZero();
  } else {
    vector_t rpy_c = toEulerAngles(base_pose_m.rotation());
    if (computeEulerAngleErr(rpy_c, rpy_start).norm() < 0.3) {
      rpy_start += dt_ * vector3_t(0, 0, yawd_);
    }
    rpy_start.head(2).setZero();

    vector3_t vw = base_pose_m.rotation() * vel_cmd;
    /* if ((base_pose_m.translation() - pos_start).norm() < 0.2) {
      pos_start += dt_ * vw;
      pos_start.z() = h_des;
    } */
    pos_start.head(2) = base_pose_m.translation().head(2) + dt_ * vw.head(2);
    pos_start.z() = h_des;
  }

  std::vector<scalar_t> time;
  std::vector<vector_t> rpy_t, pos_t;
  scalar_t horizon_time = referenceBuffer_->getModeSchedule()->duration();
  size_t N = horizon_time / 0.05;
  for (size_t k = 0; k < N; k++) {
    time.push_back(t_now + 0.05 * k);
    vector3_t rpy_k = rpy_start;
    rpy_k.z() += 0.05 * k * yawd_;
    rpy_t.emplace_back(rpy_k);

    vector3_t vel_des = toRotationMatrix(rpy_k) * vel_cmd;
    vector3_t pos_k = pos_start + 0.05 * k * vel_des;
    pos_k.z() = h_des;
    pos_t.emplace_back(pos_k);
  }

  auto cubicspline_pos = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_pos->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      base_pose_m.rotation() * vel_cmd,
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_pos->fit(time, pos_t);
  referenceBuffer_->setIntegratedBasePosTraj(cubicspline_pos);

  auto cubicspline_rpy = std::make_shared<CubicSplineTrajectory>(3);
  cubicspline_rpy->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector3_t(0, 0, yawd_),
      CubicSplineInterpolation::BoundaryType::second_deriv, vector3_t::Zero());
  cubicspline_rpy->fit(time, rpy_t);
  referenceBuffer_->setIntegratedBaseRpyTraj(cubicspline_rpy);
}

void ConvexMPC::getDynamics(scalar_t time_cur, size_t k,
                            const std::shared_ptr<ModeSchedule> mode_schedule) {
  const scalar_t time_k = time_cur + k * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();
  auto foot_traj = referenceBuffer_->getFootPosTraj();

  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
  const size_t nf = foot_names.size();
  rcpputils::assert_true(nf == contact_flag.size());

  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  vector3_t rpy_des = rpy_traj->evaluate(time_k);

  matrix_t Rt = toRotationMatrix(rpy_des);
  matrix_t Ig_t = Rt * Ig_ * Rt.transpose();

  ocp_[k].A.setIdentity(12, 12);
  ocp_[k].A.block<3, 3>(0, 3).diagonal().fill(dt_);
  ocp_[k].A.block<3, 3>(6, 9) = dt_ * getJacobiFromOmegaToRPY(rpy_des);
  ocp_[k].B.setZero(12, nf * 3);
  vector3_t xc = phase * pos_traj->evaluate(time_k) +
                 (1.0 - phase) * base_pose.translation();
  for (size_t i = 0; i < nf; i++) {
    const auto &foot_name = foot_names[i];
    if (contact_flag[i]) {
      vector3_t pf_i = foot_traj[foot_name]->evaluate(time_k);
      ocp_[k].B.middleRows(3, 3).middleCols(3 * i, 3).diagonal().fill(
          dt_ / total_mass_);
      ocp_[k].B.bottomRows(3).middleCols(3 * i, 3) =
          Ig_t.inverse() * skew(dt_ * (pf_i - xc));
    }
  }

  ocp_[k].b.setZero(12);
  ocp_[k].b(5) += -dt_ * grav_;
}

void ConvexMPC::getInequalityConstraints(
    size_t k, size_t N, const std::shared_ptr<ModeSchedule> mode_schedule) {
  const size_t nf = foot_names.size();
  scalar_t phase = k * dt_ / mode_schedule->duration();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));

  if (k < N) {
    scalar_t mu = 1 / mu_;
    matrix_t Ci(5, 3);
    Ci << mu, 0, 1., -mu, 0, 1., 0, mu, 1., 0, -mu, 1., 0, 0, 1.;
    ocp_[k].C = matrix_t::Zero(5 * nf, 12);
    ocp_[k].D.setZero(5 * nf, 3 * nf);
    ocp_[k].lg.setZero(5 * nf);
    ocp_[k].ug.setZero(5 * nf);
    ocp_[k].lg_mask.setOnes(5 * nf);
    ocp_[k].ug_mask.setOnes(5 * nf);

    for (size_t i = 0; i < nf; i++) {
      ocp_[k].D.block<5, 3>(i * 5, i * 3) = Ci;
      ocp_[k].ug(5 * i + 4) = contact_flag[i] ? 400 : 0.0;
      ocp_[k].ug_mask.segment(5 * i, 4).setZero();
    }
  }
  /* std::cout << "\n############### " << k << " constraints ################\n"
           << cstr_k; */
}

void ConvexMPC::getCosts(scalar_t time_cur, size_t k, size_t N,
                         const std::shared_ptr<ModeSchedule> mode_schedule) {
  const size_t nf = foot_names.size();
  const scalar_t time_k = time_cur + (k + 1) * dt_;
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();
  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  vector3_t rpy_des =
      rpy_traj->evaluate(time_k) - rpy_traj->evaluate(time_cur) + rpy_des_start;
  vector3_t omega_des =
      getJacobiFromRPYToOmega(rpy_des) * rpy_traj->derivative(time_k, 1);
  vector3_t v_des = pos_traj->derivative(time_k, 1);

  vector_t x_des(12);
  x_des << pos_traj->evaluate(time_k), v_des, rpy_des, omega_des;

  // std::cout << "xdes " << k << ": " << x_des.transpose() << "\n";

  ocp_[k].Q = weight_;
  ocp_[k].S = matrix_t::Zero(3 * nf, 12);
  ocp_[k].q = -weight_ * x_des;
  ocp_[k].r.setZero(3 * nf);
  if (k < N) {
    ocp_[k].R = 1e-5 * matrix_t::Identity(3 * nf, 3 * nf);
    scalar_t phase = k * dt_ / mode_schedule->duration();
    auto contact_flag =
        biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
    int nc = 0;
    for (bool flag : contact_flag) {
      if (flag) {
        nc++;
      }
    }
    nc = max(1, nc);
    vector3_t force_des_i =
        total_mass_ / nc *
        (vector3_t(0, 0, grav_) + pos_traj->derivative(time_k, 2));
    vector_t force_des = vector_t::Zero(3 * nf);
    for (size_t k = 0; k < nf; k++) {
      if (contact_flag[k]) {
        force_des.segment(3 * k, 3) = force_des_i;
      }
    }
    ocp_[k].r = -ocp_[k].R * force_des;
  } else {
    // ocp_[k].Q = 1e2 * ocp_[k].Q;
    // ocp_[k].q = 1e2 * ocp_[k].q;
  }
}

void ConvexMPC::optimize() {
  // RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
  //             "ConvexMPC: generateTrajRef start");
  const scalar_t time_cur = nodeHandle_->now().seconds();
  auto mode_schedule = referenceBuffer_->getModeSchedule();
  auto pos_traj = referenceBuffer_->getIntegratedBasePosTraj();

  if (referenceBuffer_->getFootPosTraj().empty() || pos_traj == nullptr ||
      mode_schedule == nullptr) {
    return;
  }

  auto rpy_traj = referenceBuffer_->getIntegratedBaseRpyTraj();

  size_t N = mode_schedule->duration() / dt_;
  ocp_.resize(N + 1);

  matrix_t Ig_0 = pinocchioInterface_ptr_->getData().Ig.inertia().matrix();

  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  Ig_ = base_pose.rotation().transpose() * Ig_0 * base_pose.rotation();

  auto base_twist =
      pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);
  auto rpy_m = toEulerAngles(base_pose.rotation());
  rpy_des_start =
      rpy_m - computeEulerAngleErr(rpy_m, rpy_traj->evaluate(time_cur));

  for (size_t k = 0; k <= N; k++) {
    if (k < N) {
      getDynamics(time_cur, k, mode_schedule);
    }
    getInequalityConstraints(k, N, mode_schedule);
    getCosts(time_cur, k, N, mode_schedule);
  }

  if (solution_.size() == N + 1) {
    solver_settings.warm_start = 1;
  } else {
    solver_settings.warm_start = 0;
    solution_.resize(N + 1);
  }
  hpipm::OcpQpIpmSolver solver(ocp_, solver_settings);

  vector_t x0(12);
  x0 << base_pose.translation(), base_twist.linear(), rpy_m,
      base_twist.angular();

  const auto res = solver.solve(x0, ocp_, solution_);
  if (res == hpipm::HpipmStatus::Success ||
      res == hpipm::HpipmStatus::MaxIterReached) {
    // for (size_t i = 0; i < solution_.size(); i++) {
    //   std::cout << "x " << i << ": " << solution_[i].x.transpose() << "\n";
    // }
    // for (size_t i = 0; i < solution_.size(); i++) {
    //   std::cout << "u " << i << ": " << solution_[i].u.transpose() << "\n";
    // }
    fitTraj(time_cur, N);
  } else {
    std::cout << "ConvexMPC: " << res << "\n";
    exit(0);
  }
  // RCLCPP_INFO(rclcpp::get_logger("ConvexMPC"),
  //             "ConvexMPC: generateTrajRef done");
}

void ConvexMPC::fitTraj(scalar_t time_cur, size_t N) {
  std::vector<scalar_t> time_array;
  std::vector<vector_t> base_pos_array;
  std::vector<vector_t> base_vel_array;
  std::vector<vector_t> base_rpy_array;
  std::vector<vector_t> base_omega_array;
  std::vector<vector_t> force_array;

  matrix_t P(6, 12);
  P.setZero();
  P.topRows(3).middleCols(3, 3).setIdentity();
  P.bottomRows(3).middleCols(9, 3).setIdentity();
  matrix_t A = 1.0 / dt_ * (ocp_[0].A - matrix_t::Identity(12, 12));
  matrix_t B = 1.0 / dt_ * ocp_[0].B;
  vector_t drift = 1.0 / dt_ * ocp_[0].b;
  vector_t acc_des = P * (A * solution_[0].x + B * solution_[0].u + drift);

  for (size_t k = 0; k < N; k++) {
    time_array.emplace_back(time_cur + k * dt_);
    base_pos_array.emplace_back(solution_[k].x.head(3));
    base_vel_array.emplace_back(solution_[k].x.segment(3, 3));
    base_rpy_array.emplace_back(solution_[k].x.segment(6, 3));
    base_omega_array.emplace_back(solution_[k].x.tail(3));
    force_array.emplace_back(solution_[k].u);
  }
  auto base_pos_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_pos_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_[1].x.segment(3, 3),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      solution_.back().x.segment(3, 3));
  base_pos_traj_ptr_->fit(time_array, base_pos_array);
  referenceBuffer_->setOptimizedBasePosTraj(base_pos_traj_ptr_);

  auto base_vel_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_vel_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.segment(3, 3) - solution_[0].x.segment(3, 3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ *
          (solution_[N - 1].x.segment(3, 3) -
           solution_[N - 2].x.segment(3, 3)));
  base_vel_traj_ptr_->fit(time_array, base_vel_array);
  referenceBuffer_->setOptimizedBaseVelTraj(base_vel_traj_ptr_);

  auto base_rpy_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline_hermite);
  base_rpy_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.segment(6, 3) - solution_[0].x.segment(6, 3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ *
          (solution_[N - 1].x.segment(6, 3) -
           solution_[N - 2].x.segment(6, 3)));
  base_rpy_traj_ptr_->fit(time_array, base_rpy_array);
  referenceBuffer_->setOptimizedBaseRpyTraj(base_rpy_traj_ptr_);

  auto base_omega_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      3, CubicSplineInterpolation::SplineType::cspline);
  base_omega_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[1].x.tail(3) - solution_[0].x.tail(3)),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      1.0 / dt_ * (solution_[N - 1].x.tail(3) - solution_[N - 2].x.tail(3)));
  base_omega_traj_ptr_->fit(time_array, base_omega_array);
  referenceBuffer_->setOptimizedBaseOmegaTraj(base_omega_traj_ptr_);

  auto force_traj_ptr_ = std::make_shared<CubicSplineTrajectory>(
      foot_names.size() * 3, CubicSplineInterpolation::SplineType::cspline);
  force_traj_ptr_->set_boundary(
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector_t::Zero(foot_names.size() * 3),
      CubicSplineInterpolation::BoundaryType::first_deriv,
      vector_t::Zero(foot_names.size() * 3));
  force_traj_ptr_->fit(time_array, force_array);
  referenceBuffer_->setOptimizedForceTraj(force_traj_ptr_);
}
} // namespace clear
