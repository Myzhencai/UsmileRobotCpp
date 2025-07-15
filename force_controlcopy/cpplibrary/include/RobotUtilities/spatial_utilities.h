#pragma once
#ifndef _MATH_ULTILITIES_H_
#define _MATH_ULTILITIES_H_

#ifdef __cplusplus
#include <cmath>
#include <iostream>
#else
#include <math.h>
#include <stdio.h>
#endif

#include <memory>  // smart pointers
#include <random>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// YAML
#include <yaml-cpp/yaml.h>

namespace RUT {
/////////////////////////////////////////////////////////////////////////
//                   types and static variables
/////////////////////////////////////////////////////////////////////////

typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Quaterniond Quaterniond;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

typedef Eigen::Matrix<float, 1, 1> Vector1f;
typedef Eigen::Vector2f Vector2f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::MatrixXf MatrixXf;
typedef Eigen::VectorXf VectorXf;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Quaternionf Quaternionf;
typedef Eigen::Quaterniond Quaterniond;

/////////////////////////////////////////////////////////////////////////
//                          iostream
/////////////////////////////////////////////////////////////////////////
void stream_array_in(std::ostream& st, double* array, int length);
void stream_array_in(std::ostream& st, float* array, int length);
void stream_array_in(std::ostream& st, int* array, int length);
void stream_array_in(std::ostream& st, const std::vector<double>& vec,
                     int length);
void stream_array_in(std::ostream& st, const VectorXd& vec, int length);

/////////////////////////////////////////////////////////////////////////
//                          yaml
/////////////////////////////////////////////////////////////////////////

Eigen::MatrixXd deserialize_matrix(const YAML::Node& node);
template <typename T>
T deserialize_vector(const YAML::Node& node) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("node must be a sequence.");
  }
  std::vector<double> q = node.as<std::vector<double>>();
  return Eigen::Map<T, Eigen::Unaligned>(q.data(), q.size());
}

/////////////////////////////////////////////////////////////////////////
//                          scalar
/////////////////////////////////////////////////////////////////////////
void truncate(double* ele, const double _min, const double _max);
// Generate a random number in (0, 1).
double rand();
// Generate a random integer in [0, N)
int randi(int N);

// set seed using time
int srand();

// https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
struct normal_random_variable {
  normal_random_variable(MatrixXd const& covar)
      : normal_random_variable(VectorXd::Zero(covar.rows()), covar) {}

  normal_random_variable(VectorXd const& mean, MatrixXd const& covar)
      : mean(mean) {
    Eigen::SelfAdjointEigenSolver<MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  VectorXd mean;
  MatrixXd transform;

  VectorXd operator()() const {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;

    return mean + transform * VectorXd{mean.size()}.unaryExpr(
                                  [&](double x) { return dist(gen); });
  }
};

/////////////////////////////////////////////////////////////////////////
//                          vector&array
/////////////////////////////////////////////////////////////////////////

void buf_insert(const double ele, const int size, double* buf);
void copyArray(const float* src, float* dest, int dim);
void copyArray(const double* src, double* dest, int dim);
void setArray(float* array, float value, int dim);
void truncate(float* array, float min, float max, int dim);
double vec_max(const double* vec, const int size);
double vec_min(const double* vec, const int size);
double vec_mean(const double* vec, const int size);
double vec_slope(const double* x, const double* y, const int size);
/**
   * Find in the vector the element with maximum abs value.
   *
   * @param[in]  vec   The vector.
   * @param[in]  size  Size of the vector.
   * @param      id    Id of the maximum element
   *
   * @return     The maximum element
   */
double vec_max_abs(const double* vec, const int size, int* id);
// numerical differentiation with low pass filter
// input x, calculate dx/dt
// s/(as+1),
double diff_LPF(const double xdold, const double xnew, const double xold,
                const double T, const double a);
void truncate6f(Vector6f* v, float min, float max);
void truncate6d(Vector6d* v, double min, double max);
void truncate6d(Vector6d* v, const Vector6d& min, const Vector6d& max);
void stream_array_in6f(std::ostream& st, const Vector6f& array);
void stream_array_in6d(std::ostream& st, const Vector6d& array);
void double2float(const double* array_in, float* array_out, int n);
void float2double(const float* array_in, double* array_out, int n);
/**
   * Finds an element in a std vector.
   *
   * @param[in]  vec   The vector
   * @param[in]  ele   The element
   *
   * @return     index of the found element; -1 if not found.
   */
int findInVector(std::vector<int> vec, int ele);
int findInEigenVector(const Eigen::VectorXi& vec, int ele);

/**
   * Finds multiple elements in a vector. Return the common elements among the
   * two vectors.
   *
   * @param[in]  vec   The vector
   * @param[in]  eles  The elements to be checked
   *
   * @return     The common elements (not their indices)
   */
std::vector<int> findInVector(std::vector<int> vec, std::vector<int> eles);

// lines, plucker coordinates
Vector6d getPluckerLine(const Vector3d& p, const Vector3d& n);
double reciprocalProduct(const Vector6d& line1, const Vector6d& line2);
// note: this distance is signed.
double distBTPluckerLines(const Vector6d& line1, const Vector6d& line2);
// rad
double angleBTPluckerLines(const Vector6d& line1, const Vector6d& line2);
double distPoint2PluckerLine(const Vector3d& p, const Vector6d& line);

/////////////////////////////////////////////////////////////////////////
//                          Matrices
/////////////////////////////////////////////////////////////////////////
MatrixXd pseudoInverse(const MatrixXd& a,
                       double epsilon = std::numeric_limits<double>::epsilon());

// compute Reduced row echelon form of A, i.e. find a basis, not necessarily
// orthogonal, not necessarily unit length.
// In place computation.
// return the rank of A. After the computation, the first rank rows of A are
// the row space of the input A; the rest of A are zeros.
// This is basically the Gaussian elimination.
//
// TOL: acceptable magnitude of a pivot
int rref(MatrixXd* A, double TOL = 1e-9);
/**
   * Gram-Schmidt procedure. Compute an unitary basis of the rows of A.
   *
   * @param      A     The input matrix. Will be modified.
   * @param[in]  TOL   Tol
   *
   * @return     rank of A
   */
int rowSpace(MatrixXd* A, double TOL = 1e-9);
/**
   * Compute the nullspace of A. Note it will call rowspace() on A first, so
   * A will be modified.
   *
   * @param      A      { parameter_description }
   * @param      nullA  The null a
   * @param[in]  TOL    The tol
   *
   * @return     { description_of_the_return_value }
   */
int nullSpace(MatrixXd* A, MatrixXd* nullA, double TOL = 1e-9);
/////////////////////////////////////////////////////////////////////////
//                          Robotics
/////////////////////////////////////////////////////////////////////////
/*  Frames/spaces:
          W: world frame
          T: current tool frame
          So: set tool frame with offset
          Tf: transformed generalized space
      Quantities:
          SE3: 4x4 homogeneous coordinates
          se3: 6x1 twist coordinate of SE3
          spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for
     rotation td: 6x1 time derivative of twist. v: 6x1 velocity, either spatial or
     body

          wrench: 6x1 wrench. Makes work with body velocity
  */

Matrix3d wedge(const Vector3d& v);
Matrix4d wedge6(const Vector6d& t);
void wedge6(const Vector6d& t, Matrix4d& t_wedge);
/**
   * wedge(v) * p = A(p) * v
   *
   * @return     4x6 matrix A(p)
   */
MatrixXd wedgeRight6(const Vector4d& p);
Vector6d vee6(const Matrix4d& T);
RUT::Vector4d rpy2quat(double roll, double pitch, double yaw);
void rpy2quat(double roll, double pitch, double yaw, double* quat);

//
// Transformations
//

Matrix3d aa2mat(const double theta, const Vector3d& n);
// Axis-angle to quaternion
// input:
//   theta: scalar angle
//   n: 3 x 1, axis, not necessarily unit length
// output:
// q: 4x1
void aa2quat(const double theta, const Vector3d& n, Vector4d& q);
Vector4d aa2quat(const double theta, const Vector3d& n);
Matrix3d quat2SO3(const Vector4d& q);
Matrix3d quat2SO3(const Quaterniond& q);
Matrix3d quat2SO3(double qw, double qx, double qy, double qz);
// quaternion to axis-angle
// input:
//   q: 4x1
// output:
//   aa: 3x1, axis direction with norm being the angle
void quat2aa(const Vector4d& q, Vector3d& aa);
Matrix3d so32SO3(const Vector3d& v);
Vector3d SO32so3(const Matrix3d& R);
void SO32so3(const Matrix3d& R, double* so3);
void so32quat(const Vector3d& so3, double* q);
void so32quat(const Vector3d& so3, Vector4d& q);
void so32quat(const double* so3, Eigen::Ref<RUT::Vector4d> q);
void SO32quat(const Matrix3d& SO3, Eigen::Ref<RUT::Vector4d> q);
void SO32quat(const Matrix3d& SO3, double* q);
Matrix4d pose2SE3(const double* pose);
Matrix4d pose2SE3(const Vector7d& pose);
Matrix4d posemm2SE3(const double* pose);
Matrix4d se32SE3(const Vector6d& twist);
Matrix4d spt2SE3(const Vector6d& spt);
Matrix4d SE3Inv(const Matrix4d& SE3);
Vector6d SE32se3(const Matrix4d& SE3);
Vector6d SE32spt(const Matrix4d& SE3);
Matrix6d SE32Adj(const Matrix4d& SE3);
void SE32Pose(const Matrix4d& SE3, Vector7d& pose);
void SE32Pose(const Matrix4d& SE3, double* pose);
void SE32Posemm(const Matrix4d& SE3, double* pose);
Matrix3d rotX(double angle_rad);
Matrix3d rotY(double angle_rad);
Matrix3d rotZ(double angle_rad);
/**
   * @brief      Gets the rotation matrix from z vector. The x and y axes are
   *             choosen arbitrarily.
   *
   * @param[in]  z     { Unit vector measured in world frame }
   *
   * @return     Rotation matrix from world to the rotated frame.
   */
Matrix3d getRFromZ(const Vector3d& z);

//
// Quaternions
//
Quaternionf QuatMTimes(const Quaternionf& q1, const Quaternionf& q2);
Quaterniond QuatMTimes(const Quaterniond& q1, const Quaterniond& q2);
float angBTquat(const Quaternionf& q1, const Quaternionf& q2);
double angBTquat(const Quaterniond& q1, const Quaterniond& q2);
/**
 * This format assumes the last four elements of q1 and q2 are the quaternions.
 */
double angBTquat(const RUT::VectorXd& q1, const RUT::VectorXd& q2);

/**
   * Compute the quaternion @p qm that is @p angle away from quaternion @p qa
   * towards @p qb, along the direction of spherical interpolation. If the
   * distance between qa and qb is smaller than @p angle, return qb. Quaternions
   * are represented as [qw qx qy qz]. Throw error if qa and qb are exactly 180
   * degree away.
   * Modified from:
   *   http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
   *
   * @param[in]  qa     Starting quaternion.
   * @param[in]  qb     Final quaternion.
   * @param      qm     The interpolated quaternion.
   * @param[in]  angle  The angle.
   *
   * @return     0 if distance between qa and qb is smaller than @p angle.
   * Return 1 otherwise.
   */
int SlerpFixAngle(const Vector4d& qa, const Vector4d& qb, Vector4d& qm,
                  float angle);
//
// SE(3)
//
class CartesianPose {
 public:
  CartesianPose();
  ~CartesianPose();
  /**
     * Construct the pose from a 1x7 vector.
     *
     * @param[in]  pose  Pose vector. [x y z qw qx qy qz]
     */
  CartesianPose(std::vector<double> pose);
  CartesianPose(double* pose);
  /**
     * Constructs the pose from an Eigen Matrix. T must be either:
     *  a 4x4 homogeneous matrix, or
     *  a 7x1 vector.
     *
     * @param[in]  T     The 4x4 homogeneous matrix or 7x1 vector.
     */
  CartesianPose(const MatrixXd& T);
  CartesianPose(const Eigen::Isometry3d& iso);
  CartesianPose(const Quaterniond& q, const Vector3d& p);
  CartesianPose(const Matrix3d& R, const Vector3d& p);

  // other constructors
  CartesianPose(CartesianPose&& gp);
  CartesianPose& operator=(CartesianPose&& gp);       // move assignment
  CartesianPose(const CartesianPose& gp);             // copy
  CartesianPose& operator=(const CartesianPose& gp);  // copy assignment

  static CartesianPose Identity();

  // types
  using Ptr = std::shared_ptr<CartesianPose>;
  using ConstPtr = std::shared_ptr<const CartesianPose>;

  // setters/getters
  void setRotationMatrix(const Matrix3d& R);
  void setQuaternion(const Quaterniond& q);
  void setQuaternion(const std::vector<double>& q);
  void setXYZ(const Vector3d& p);
  void setXYZ(const std::vector<double>& p);
  void setX(double);
  void setY(double);
  void setZ(double);
  void scaleXYZ(double scale);

  Matrix3d getRotationMatrix() const;
  Quaterniond getQuaternion() const;
  Vector4d getQuaternionVec() const;
  Vector3d getXYZ() const;
  double getX() const;
  double getY() const;
  double getZ() const;
  Vector3d getXAxis() const;
  Vector3d getYAxis() const;
  Vector3d getZAxis() const;
  Matrix4d getTransformMatrix() const;
  Eigen::Isometry3d getIsometry3d() const;
  std::vector<double> getVector() const;
  void getArray(double* array) const;  // x y z qw qx qy qz

  // operators
  CartesianPose operator*(const CartesianPose& pose) const;
  Vector3d operator*(const Vector3d& p) const;
  CartesianPose inv() const;
  /**
     * Compute the pose that moves from this pose towards the goal @p pose in
     * the shortest path (straight line for translation, SLERP direction for
     * rotation). The distance moved is bounded by @p max_trans and @p
     * max_rotation.
     *
     * @param[in]  pose          The goal pose
     * @param[in]  max_trans     The maximum translation (unit is the same with
     *                           XYZ)
     * @param[in]  max_rotation  The maximum rotation (rad)
     *
     * @return     The incremental cartesian pose.
     */
  CartesianPose increTowards(const CartesianPose& pose, double max_trans,
                             double max_rotation);

  // Transformations
  Vector3d transformVec(const Vector3d& v) const;
  Vector3d transformPoint(const Vector3d& p) const;
  MatrixXd transformPoints(const MatrixXd& ps) const;
  Quaterniond transformQuat(const Quaterniond& q) const;

  // metric

  /**
     * Distance between this pose and another pose
     *
     * @param[in]  pose   Another pose
     * @param[in]  ratio  scaling of rotation to length
     *
     * @return     angle*ratio + length
     */
  double distBTPose(const CartesianPose& pose, double ratio = 1.0) const;

  // MISC
  void print() const;
  void printPose() const;
  std::string poseString() const;
  // public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  double qw_;
  double qx_;
  double qy_;
  double qz_;
  Vector3d* p_;
  Matrix3d* R_;
};

//
// Vector operations
//
/**
   * find the (signed) angle from vector x to vector b.
   *
   * @param[in]  x            initial vector.
   * @param[in]  b            final vector.
   * @param[in]  z            if specified, it is the rotation axis. It is used
   *                          to specify positive direction of rotation.
   * @param[in]  nonnegative  when z is present, nonnegative will decide the
   *                          range of return angle between [-pi, pi] or [0, 2pi]
   *
   * @return     The angle. If @p z is not given, range is [0, pi]. If @p z is
   *             given, range is [-pi, pi] (@p nonnegative = false) or
   *             [0, 2pi] (@p nonnegative = true).
   */
double angBTVec(Vector3d x, Vector3d b, Vector3d z = Vector3d::Zero(),
                bool nonnegative = false);

//
// Others
//

// Return the 6x6 jacobian matrix mapping from spt time derivative
//  to body velocity.
// Jac * spt time derivative = body velocity
Matrix6d JacobianSpt2BodyV(const Matrix3d& R);

/**
   * Compute the contact frame from two contact points.
   *    X: p2 - p1
   *    Y: world Z.cross(X)
   *    Z: X.cross(Y)
   *    p: center of p1 and p2
   *
   * @param[in]  p1    The p1
   * @param[in]  p2    The p2
   *
   * @return     The frame from two point.
   */
CartesianPose getFrameFromTwoPoint(const Vector3d& p1, const Vector3d& p2);
/////////////////////////////////////////////////////////////////////////
//                      Motion Planning
/////////////////////////////////////////////////////////////////////////

/**
   * Linear interpolation between @p pose0 and @p pose_set.
   *
   * @param[in]  pose0      The initial pose. [x y z qw qx qy qz]
   * @param[in]  pose_set   The goal pose
   * @param[in]  Nsteps     The number of time steps
   * @param      pose_traj  The interpolated pose traj. 7 x Nsteps matrix
   */
void MotionPlanningLinear(const double* pose0, const double* pose_set,
                          const int Nsteps, MatrixXd* pose_traj);

/**
   * 1D trapezodial interpolation from x0 = 0 to x_f, limited by
   * maximum acceleration @p a_max, maximum velocity @p v_max. The return
   * trajectory is sampled in @p Nsteps time steps. The total duration of the
   * trajectory is 2*t1 + t2.
   *
   * If @p Nsteps=0 (default), the function only computes time @p t1, @p t2,
   * do not generate the trajectory.
   *
   * @param[in]  x_f     The final position
   * @param[in]  a_max   Maximum acceleration
   * @param[in]  v_max   Maximum velocity
   * @param      t1      Time duration of the acceleration phase
   * @param      t2      Time duration of the constant speed phase
   * @param[in]  Nsteps  The number of sampling points
   * @param      x_traj  The interpolated trajectory, 1 x Nsteps array
   */
void TrapezodialTrajectory(double x_f, double a_max, double v_max, double* t1,
                           double* t2, int Nsteps = 0, double* x_traj = 0);
/**
   * Pose to pose trapezodial interpolation. Poses are represented as double
   * arrays of length 7 (x, y, z, qw, qx, qy, qz). Unit of translation is mm.
   * Number of frames is determined by the speed/acceleration limit and the
   * control rate @p rate.
   * @param[in]  pose0        The current pose
   * @param[in]  pose_set     The goal pose
   * @param[in]  a_max_trans  Maximum translational acceleration (mm/s^2)
   * @param[in]  v_max_trans  Maximum translational velocity (mm/s)
   * @param[in]  a_max_rot    Maximum rotational acceleration (rad/s^2)
   * @param[in]  v_max_rot    Maximum rotational velocity (rad/s)
   * @param[in]  rate         Control rate (Hz).
   * @param      pose_traj    Output trajectory, a 7 x Nsteps Eigen Matrix.
   */
void MotionPlanningTrapezodial(const double* pose0, const double* pose_set,
                               double a_max_trans, double v_max_trans,
                               double a_max_rot, double v_max_rot, double rate,
                               MatrixXd* pose_traj);

/////////////////////////////////////////////////////////////////////////
//                      Probability
/////////////////////////////////////////////////////////////////////////
double Gaussian(double x, double var);
}  // namespace RUT

#endif  // _MATH_ULTILITIES_H_
