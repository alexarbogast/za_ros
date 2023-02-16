#include <za_gazebo/model_kdl.h>

#include <eigen_conversions/eigen_kdl.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace za_gazebo {

int ModelKDL::segment(za::Frame frame) {
  // clang-format off
  switch (frame) {
    case za::Frame::kJoint1:      return 0;
    case za::Frame::kJoint2:      return 1;
    case za::Frame::kJoint3:      return 2;
    case za::Frame::kJoint4:      return 3;
    case za::Frame::kJoint5:      return 4;
    case za::Frame::kJoint6:      return 5;
    case za::Frame::kFlange:      return 6;
    case za::Frame::kEndEffector: return 7;
    default: return -1;
  }
  // clang-format on
}

bool ModelKDL::isCloseToSingularity(const KDL::Jacobian& jacobian) const {
  if (this->singularity_threshold_ < 0) {
    return false;
  }
  Eigen::Matrix<double, 6, 6> mat = jacobian.data * jacobian.data.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double critical = svd.singularValues().tail(1)(0);
  return critical < this->singularity_threshold_;
}

// Implementation copied from <kdl/isolveri.hpp> because
// KDL::ChainDynSolver inherits *privately* from SolverI ... -.-'
std::string ModelKDL::strError(const int error) {
  // clang-format off
  switch(error) {
  case KDL::SolverI::E_NOERROR:                 return "No error"; break;
  case KDL::SolverI::E_NO_CONVERGE:             return "Failed to converge"; break;
  case KDL::SolverI::E_UNDEFINED:               return "Undefined value"; break;
  case KDL::SolverI::E_DEGRADED:                return "Converged but degraded solution"; break;
#if ROS_VERSION_MINIMUM(1, 15, 0)
  // These were introduced in melodic
  case KDL::SolverI::E_NOT_UP_TO_DATE:          return "Internal data structures not up to date with Chain"; break;
  case KDL::SolverI::E_SIZE_MISMATCH:           return "The size of the input does not match the internal state"; break;
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: return "The maximum number of iterations is exceeded"; break;
  case KDL::SolverI::E_OUT_OF_RANGE:            return "The requested index is out of range"; break;
  case KDL::SolverI::E_NOT_IMPLEMENTED:         return "The requested function is not yet implemented"; break;
  case KDL::SolverI::E_SVD_FAILED:              return "SVD failed"; break;
#endif
  default: return "UNKNOWN ERROR";
  }
  // clang-format on
}
ModelKDL::ModelKDL(const urdf::Model& model,
                   const std::string& root,
                   const std::string& tip,
                   double singularity_threshold)
    : singularity_threshold_(singularity_threshold) {
  KDL::Tree tree;
  if (not kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("Cannot construct KDL tree from URDF");
  }

  if (not tree.getChain(root, tip, this->chain_)) {
    throw std::invalid_argument("Cannot find chain within URDF tree from root '" + root +
                                "' to tip '" + tip + "'. Do these links exist?");
  }

  ROS_INFO_STREAM("KDL Model initialized for chain from '" << root << "' -> '" << tip << "'");
}

void ModelKDL::augmentFrame(const std::string& name,
                            const std::array<double, 16>& transform,
                            KDL::Chain& chain) {
  Eigen::Affine3d t;
  KDL::Frame f;
  t.matrix() = Eigen::Matrix4d(transform.data());
  tf::transformEigenToKDL(t, f);
  chain.addSegment(KDL::Segment(name, KDL::Joint(KDL::Joint::None), f));
}

void ModelKDL::augmentFrame(const std::string& name,
                            const std::array<double, 3>& center_of_mass,
                            double mass,
                            const std::array<double, 9>& inertia,
                            KDL::Chain& chain) {
  KDL::Vector kc;
  KDL::RotationalInertia ki;
  std::copy(center_of_mass.begin(), center_of_mass.end(), std::begin(kc.data));
  std::copy(inertia.begin(), inertia.end(), std::begin(ki.data));

  chain.addSegment(KDL::Segment(name, KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector::Zero()),
                                KDL::RigidBodyInertia(mass, kc, ki)));
}

std::array<double, 16> ModelKDL::pose(
    za::Frame frame,
    const std::array<double, 6>& q,
    const std::array<double, 16>& F_T_EE)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Frame kp;

  // Agument the chain with the two new Frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);

  KDL::ChainFkSolverPos_recursive solver(chain);

  kq.data = Eigen::Matrix<double, 6, 1>(q.data());

  int error = solver.JntToCart(kq, kp, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL forward kinematics pose calculation failed with error: " +
                           strError(error));
  }
  Eigen::Affine3d p;
  tf::transformKDLToEigen(kp, p);

  std::array<double, 16> result;
  Eigen::MatrixXd::Map(&result[0], 4, 4) = p.matrix();

  return result;
}

std::array<double, 36> ModelKDL::bodyJacobian(
    za::Frame frame,
    const std::array<double, 6>& q,
    const std::array<double, 16>& F_T_EE)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Jacobian J(6);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 6, 1>(q.data());

  // Augment the chain with the two virtual frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);

  KDL::ChainJntToJacSolver solver(chain);

  int seg = segment(frame);
  int error = solver.JntToJac(kq, J, seg);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL zero jacobian calculation failed with error: " + strError(error));
  }

  // Shift the zero jacobian to the "body" frame by using the rotation of the FK of that frame
  KDL::Frame f = KDL::Frame::Identity();
  auto transform = pose(frame, q, F_T_EE);
  Eigen::Affine3d t;
  t.matrix() = Eigen::Matrix4d(transform.data());
  tf::transformEigenToKDL(t, f);

  J.changeBase(f.M.Inverse());

  // Singularity check
  if (isCloseToSingularity(J)) {
    ROS_WARN_STREAM_THROTTLE(1, "Body Jacobian close to singularity");
  }

  std::array<double, 36> result;
  Eigen::MatrixXd::Map(&result[0], 6, 6) = J.data;

  return result;
}

std::array<double, 36> ModelKDL::zeroJacobian(
    za::Frame frame,
    const std::array<double, 6>& q,
    const std::array<double, 16>& F_T_EE)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Jacobian J(6);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 6, 1>(q.data());

  // Augment the chain with the two virtual frames 'EE' and 'K'
  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("EE", F_T_EE, chain);

  KDL::ChainJntToJacSolver solver(chain);

  int error = solver.JntToJac(kq, J, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL zero jacobian calculation failed with error: " + strError(error));
  }

  // Singularity Check
  if (isCloseToSingularity(J)) {
    ROS_WARN_STREAM_THROTTLE(1, "Zero Jacobian close to singularity");
  }

  std::array<double, 36> result;
  Eigen::MatrixXd::Map(&result[0], 6, 6) = J.data;

  return result;
}

std::array<double, 36> ModelKDL::mass(
    const std::array<double, 6>& q,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::JntSpaceInertiaMatrix M(6);  // NOLINT(readability-identifier-naming)
  kq.data = Eigen::Matrix<double, 6, 1>(q.data());

  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("load", F_x_Ctotal, m_total, I_total, chain);
  KDL::ChainDynParam solver(chain, KDL::Vector(0, 0, -9.81));

  int error = solver.JntToMass(kq, M);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL mass calculation failed with error: " + strError(error));
  }

  std::array<double, 36> result;
  Eigen::MatrixXd::Map(&result[0], 6, 6) = M.data;

  return result;
}

std::array<double, 6> ModelKDL::coriolis(
    const std::array<double, 6>& q,
    const std::array<double, 6>& dq,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq, kdq, kc(6);
  kq.data = Eigen::Matrix<double, 6, 1>(q.data());
  kdq.data = Eigen::Matrix<double, 6, 1>(dq.data());

  KDL::Chain chain = this->chain_;  // copy
  augmentFrame("load", F_x_Ctotal, m_total, I_total, chain);
  KDL::ChainDynParam solver(chain, KDL::Vector(0, 0, -9.81));

  int error = solver.JntToCoriolis(kq, kdq, kc);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL coriolis calculation failed with error: " + strError(error));
  }

  std::array<double, 6> result;
  Eigen::VectorXd::Map(&result[0], kc.data.size()) = kc.data;

  return result;
}

std::array<double, 6> ModelKDL::gravity(
    const std::array<double, 6>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const {
  KDL::JntArray kq, kg(6);
  KDL::Vector grav(gravity_earth[0], gravity_earth[1], gravity_earth[2]);
  kq.data = Eigen::Matrix<double, 6, 1>(q.data());

  KDL::Chain chain = this->chain_;  // copy
  KDL::ChainDynParam solver(chain, grav);

  ///* ========= TEMPORARY ========== */
  //std::cout << "Gravity: " << gravity_earth[0] << gravity_earth[1] << gravity_earth[2] << std::endl;
//
  //for (auto& seg : chain.segments)
  //{
  //  std::cout << "Name: " << seg.getName() << std::endl;
  //  std::cout << "Joint: " << seg.getJoint().getName() << std::endl;
//
  //  auto cog = seg.getInertia().getCOG();
  //  std::cout <<  "COG:" << cog.data[0] << cog.data[1] << cog.data[2] << std::endl;
  //  
  //  std::cout << "Mass: " << seg.getInertia().getMass() << std::endl;
  //  auto rot = seg.getInertia().getRotationalInertia().data;
  //  std::cout << "I: " << rot[0] <<" "<< rot[1] <<" "<< rot[2] <<"\n"<< rot[3] <<" "<< rot[4] <<" "<< rot[5]<<"\n" << rot[6] <<" "<< rot[7] <<" "<< rot[8] <<std::endl;
  //}
  ///* ============================= */

  int error = solver.JntToGravity(kq, kg);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL gravity calculation failed with error: " + strError(error));
  }

  std::array<double, 6> result;
  Eigen::VectorXd::Map(&result[0], kg.data.size()) = kg.data;

  return result;
}

}  // namespace franka_gazebo
