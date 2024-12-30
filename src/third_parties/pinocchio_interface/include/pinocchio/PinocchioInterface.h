#pragma once
#include <core/types.h>
#include <mutex>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

using namespace std;

namespace pin = pinocchio;
namespace clear {
class PinocchioInterface {
private:
  pin::Model model_;
  pin::Data data_;
  vector_t qpos_, qvel_;
  vector<bool> contact_mask_;
  vector<string> contact_points_;

public:
  PinocchioInterface(const char *urdf_name);

  ~PinocchioInterface();

  void setContactPoints(vector<string> contact_points);

  const pin::Model &getModel();

  const pin::Data &getData();

  void updateRobotState(vector_t qpos, vector_t qvel);

  scalar_t total_mass();

  vector3_t getCoMPos();

  vector3_t getCoMVel();

  vector3_t getCoMAcc();

  const matrix3x_t &getJacobia_CoM();

  void getJacobia_local(string frame_name, matrix6x_t &J);

  void getJacobia_world(string frame_name, matrix6x_t &J);

  void getJacobia_localWorldAligned(string frame_name, matrix6x_t &J);

  void getContactPointJacobia_localWorldAligned(size_t idx, matrix6x_t &J);

  pin::FrameIndex getFrameID(string frame_name);

  Eigen::Ref<matrix_t> M();

  matrix_t Minv();

  Eigen::Ref<vector_t> nle();

  const pin::SE3 &getFramePose(string frame_name);

  pin::Motion getFrame6dVel_local(string frame_name);

  pin::Motion getFrame6dVel_localWorldAligned(string frame_name);

  pin::Motion getFrame6dAcc_local(string frame_name);

  pin::Motion getFrame6dAcc_world(string frame_name);

  pin::Motion getFrame6dAcc_localWorldAligned(string frame_name);

  size_t nq();

  size_t nv();

  size_t na();

  size_t nc();

  Eigen::Ref<vector_t> qpos();

  Eigen::Ref<vector_t> qvel();

  const matrix6x_t &getMomentumJacobia();

  vector6_t getMomentumTimeVariation();

  void setContactMask(const vector<bool> &mask);

  const vector<bool> &getContactMask();

  const vector<string> &getContactPoints();
};
} // namespace clear