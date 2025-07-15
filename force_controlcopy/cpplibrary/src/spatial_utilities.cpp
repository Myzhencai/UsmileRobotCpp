#include "RobotUtilities/spatial_utilities.h"

#include <algorithm>
#include <ctime>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace RUT {
/////////////////////////////////////////////////////////////////////////
//                   types and variables
/////////////////////////////////////////////////////////////////////////
const static double kEpsilon = 1e-7;
const static float PIf = 3.1416f;
const static double PI = 3.1415926;

/////////////////////////////////////////////////////////////////////////
//                          iostream
/////////////////////////////////////////////////////////////////////////
void stream_array_in(std::ostream& st, double* array, int length) {
  for (int i = 0; i < length; i++) {
    st << array[i] << "\t";
  }
}

void stream_array_in(std::ostream& st, float* array, int length) {
  for (int i = 0; i < length; i++) {
    st << array[i] << "\t";
  }
}

void stream_array_in(std::ostream& st, int* array, int length) {
  for (int i = 0; i < length; i++) {
    st << array[i] << "\t";
  }
}

void stream_array_in(std::ostream& st, const std::vector<double>& vec,
                     int length) {
  for (int i = 0; i < length; ++i) {
    st << vec[i] << "\t";
  }
}

void stream_array_in(std::ostream& st, const Eigen::VectorXd& vec, int length) {
  for (int i = 0; i < length; ++i) {
    st << vec[i] << "\t";
  }
}
/////////////////////////////////////////////////////////////////////////
//                          Yaml
/////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd deserialize_matrix(const YAML::Node& node) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("node must be a sequence.");
  }
  if (!node[0].IsSequence()) {
    throw std::invalid_argument("node must be a sequence of sequences.");
  }
  int nr = node.size();
  int nc = node[0].size();
  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nr, nc);
  for (int r = 0; r < nr; ++r) {
    for (int c = 0; c < nc; ++c) {
      mat(r, c) = node[r][c].as<double>();
    }
  }
  return mat;
}

/////////////////////////////////////////////////////////////////////////
//                          scalar
/////////////////////////////////////////////////////////////////////////

void truncate(double* ele, const double _min, const double _max) {
  if ((*ele) > _max)
    (*ele) = _max;
  else if ((*ele) < _min)
    (*ele) = _min;
}

double rand() {
  int std_rand = std::rand();
  return double(std_rand) / double(RAND_MAX);
}
int randi(int N) {
  int std_rand = std::rand();
  return std_rand % N;
}
int srand() {
  int seed = std::time(NULL);
  std::srand(seed);
  return seed;
}

/////////////////////////////////////////////////////////////////////////
//                          vector&array
/////////////////////////////////////////////////////////////////////////

void buf_insert(const double ele, const int size, double* buf) {
  for (int i = 1; i < size; ++i) {
    buf[size - i] = buf[size - 1 - i];
  }
  buf[0] = ele;
}

void copyArray(const float* src, float* dest, int dim) {
  for (int i = 0; i < dim; i++) {
    dest[i] = src[i];
  }
}

void copyArray(const double* src, double* dest, int dim) {
  for (int i = 0; i < dim; i++) {
    dest[i] = src[i];
  }
}

void setArray(float* array, float value, int dim) {
  for (int i = 0; i < dim; i++) {
    array[i] = value;
  }
}

void truncate(float* array, float min, float max, int dim) {
  for (int i = 0; i < dim; i++) {
    array[i] = (array[i] > max) ? max : array[i];
    array[i] = (array[i] < min) ? min : array[i];
  }
}

double vec_max(const double* vec, const int size) {
  double m = vec[0];
  double t1;
  for (int i = 0; i < size; ++i) {
    t1 = vec[i];
    if (t1 > m)
      m = t1;
  }
  return m;
}

double vec_min(const double* vec, const int size) {
  double m = vec[0];
  double t1;
  for (int i = 0; i < size; ++i) {
    t1 = vec[i];
    if (t1 < m)
      m = t1;
  }
  return m;
}

double vec_max_abs(const double* vec, const int size, int* id) {
  double m = vec[0];
  double t1;
  for (int i = 0; i < size; ++i) {
    t1 = fabs(vec[i]);
    if (t1 > m) {
      *id = i;
      m = t1;
    }
  }
  return m;
}

double vec_mean(const double* vec, const int size) {
  double sum = 0;
  for (int i = 0; i < size; ++i) {
    sum += vec[i];
  }
  return sum / double(size);
}

double vec_slope(const double* x, const double* y, const int size) {
  double avgX = vec_mean(x, size);
  double avgY = vec_mean(y, size);

  double numerator = 0.0;
  double denominator = 0.0;

  double xd = 0;
  for (int i = 0; i < size; ++i) {
    xd = x[i] - avgX;
    numerator += (xd) * (y[i] - avgY);
    denominator += xd * xd;
  }

  return numerator / denominator;
}

// numerical differentiation with low pass filter
// input x, calculate dx/dt
// s/(as+1),
double diff_LPF(const double xdold, const double xnew, const double xold,
                const double T, const double a) {
  double As = exp(-T / a);
  return As * xdold + (1 - As) * ((xnew - xold) / T);
}

void truncate6f(Vector6f* v, float min, float max) {
  for (int i = 0; i < 6; i++) {
    (*v)[i] = ((*v)[i] > max) ? max : (*v)[i];
    (*v)[i] = ((*v)[i] < min) ? min : (*v)[i];
  }
}

void truncate6d(Vector6d* v, double min, double max) {
  for (int i = 0; i < 6; i++) {
    (*v)[i] = ((*v)[i] > max) ? max : (*v)[i];
    (*v)[i] = ((*v)[i] < min) ? min : (*v)[i];
  }
}

void truncate6d(Vector6d* v, const Vector6d& min, const Vector6d& max) {
  for (int i = 0; i < 6; i++) {
    (*v)[i] = ((*v)[i] > max[i]) ? max[i] : (*v)[i];
    (*v)[i] = ((*v)[i] < min[i]) ? min[i] : (*v)[i];
  }
}

void stream_array_in6f(std::ostream& st, const Vector6f& array) {
  for (int i = 0; i < 6; i++) {
    st << array(i);
    st << "\t";
  }
}
void stream_array_in6d(std::ostream& st, const Vector6d& array) {
  for (int i = 0; i < 6; i++) {
    st << array(i);
    st << "\t";
  }
}

/////////////////////////////////////////////////////////////////////////
//                          Matrices
/////////////////////////////////////////////////////////////////////////
MatrixXd pseudoInverse(const MatrixXd& a, double epsilon) {
  if (a.norm() < epsilon) {
    return MatrixXd::Zero(a.cols(), a.rows());
  } else {
    Eigen::JacobiSVD<MatrixXd> svd(a,
                                   Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *
                       svd.singularValues().array().abs()(0);
    return svd.matrixV() *
           (svd.singularValues().array().abs() > tolerance)
               .select(svd.singularValues().array().inverse(), 0)
               .matrix()
               .asDiagonal() *
           svd.matrixU().adjoint();
  }
}

// This is a rewriting of matlab's implementation
int rref(MatrixXd* A, double TOL) {
  // Loop over the entire matrix.
  int m = A->rows();
  int n = A->cols();
  int i = 0;
  int j = 0;
  int rank = 0;
  VectorXd column;
  while (i < m && j < n) {
    // Find value and index of largest element in the remainder of column j.
    // [p, k] = max(abs(A(i:m,j)));
    int k;  // index
    column = A->block(i, j, m - i, 1).cwiseAbs();
    double p = column.maxCoeff(&k);
    k = k + i;
    if (p <= TOL) {
      // The column is negligible, zero it out.
      // A(i:m,j) = 0;
      A->block(i, j, m - i, 1) = MatrixXd::Zero(m - i, 1);
      j++;
    } else {
      rank++;
      // Swap i-th and k-th rows.
      // A([i k],j:n) = A([k i],j:n);
      MatrixXd temp_row = A->block(i, j, 1, n - j);
      A->block(i, j, 1, n - j) = A->block(k, j, 1, n - j);
      A->block(k, j, 1, n - j) = temp_row;

      // Divide the pivot row by the pivot element.
      // A(i,j:n) = A(i,j:n)./A(i,j);
      A->block(i, j, 1, n - j) /= (*A)(i, j);
      // Subtract multiples of the pivot row from all the other rows.
      for (k = 0; k < m; ++k) {
        if (k == i)
          continue;
        // A(k,j:n) = A(k,j:n) - A(k,j).*A(i,j:n);
        A->block(k, j, 1, n - j) =
            A->block(k, j, 1, n - j) - (*A)(k, j) * A->block(i, j, 1, n - j);
      }
      i++;
      j++;
    }
  }
  return rank;
}

int rowSpace(MatrixXd* A, double TOL) {
  // Loop over the entire matrix.
  int m = std::min(A->rows(), A->cols());
  int rows = A->rows();
  int dim = A->cols();
  MatrixXd I = MatrixXd::Identity(dim, dim);
  MatrixXd proj;
  VectorXd norms;
  MatrixXd pivot_row, ith_row;
  for (int i = 0; i < m; ++i) {
    // Find the row with largest norm
    norms = A->bottomRows(rows - i).rowwise().norm();
    int k;
    double p = norms.maxCoeff(&k);
    if (p <= TOL) {
      // time to stop
      return i;
    }
    k = k + i;
    // use the kth row to eliminate all other rows
    pivot_row = A->middleRows(k, 1).rowwise().normalized();
    proj = I - pivot_row.transpose() * pivot_row;
    A->bottomRows(rows - i) = A->bottomRows(rows - i) * proj.transpose();
    // move the kth row to ith row
    if (k != i) {
      ith_row = A->middleRows(i, 1);
      A->middleRows(i, 1) = pivot_row;
      A->middleRows(k, 1) = ith_row;
    } else {
      A->middleRows(i, 1) = pivot_row;
    }
  }
  return m;
}

int nullSpace(MatrixXd* A, MatrixXd* nullA, double TOL) {
  int rank = rowSpace(A, TOL);
  int rows = A->rows();
  int cols = A->cols();
  assert(rank <= cols);
  assert(rank <= rows);
  if (rank == cols) {
    *nullA = MatrixXd(0, cols);
    return rank;
  }
  /**
     * augment A with an identity matrix
     */
  MatrixXd A_aug(rank + cols, cols);
  MatrixXd I = MatrixXd::Identity(cols, cols);
  A_aug << A->topRows(rank), I;
  /**
     * do Gram-Schmidt again
     */
  // Loop over the entire matrix.
  int m = cols;
  int rows_aug = A_aug.rows();
  MatrixXd proj;
  VectorXd norms;
  MatrixXd pivot_row, ith_row;
  for (int i = 0; i < m; ++i) {
    // Find the row with largest norm
    int k;
    if (i >= rank) {
      norms = A_aug.bottomRows(rows_aug - i).rowwise().norm();
      double p = norms.maxCoeff(&k);
      if (p <= TOL) {
        // time to stop
        m = i;
        break;
      }
      k = k + i;
    } else {
      k = i;
    }
    // use the kth row to eliminate all other rows
    pivot_row = A_aug.middleRows(k, 1).rowwise().normalized();
    proj = I - pivot_row.transpose() * pivot_row;
    A_aug.bottomRows(rows_aug - i) =
        A_aug.bottomRows(rows_aug - i) * proj.transpose();
    // move the kth row to ith row
    if (k != i) {
      ith_row = A_aug.middleRows(i, 1);
      A_aug.middleRows(i, 1) = pivot_row;
      A_aug.middleRows(k, 1) = ith_row;
    } else {
      A_aug.middleRows(i, 1) = pivot_row;
    }
  }
  /**
     * Read the rows after rank
     */
  m = m - rank;
  *nullA = A_aug.middleRows(rank, m);
  return rank;
}

/////////////////////////////////////////////////////////////////////////
//                          Robotics
/////////////////////////////////////////////////////////////////////////
// clang-format off
/*  Frames/spaces:
        W: world frame
        T: current tool frame
        So: set tool frame with offset
        Tf: transformed generalized space
    Quantities:
        SE3: 4x4 homogeneous coordinates
        se3: 6x1 twist coordinate of SE3
        spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
        td: 6x1 time derivative of twist.
        v: 6x1 velocity, either spatial or body

        wrench: 6x1 wrench. Makes work with body velocity
*/
// clang-format on
Matrix3d wedge(const Vector3d& v) {
  Matrix3d v_wedge;
  v_wedge << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return v_wedge;
}

Matrix4d wedge6(const Vector6d& t) {
  Matrix4d t_wedge;
  t_wedge << 0, -t(5), t(4), t(0), t(5), 0, -t(3), t(1), -t(4), t(3), 0, t(2),
      0, 0, 0, 0;
  return t_wedge;
}

void wedge6(const Vector6d& t, Matrix4d& t_wedge) {
  t_wedge << 0, -t(5), t(4), t(0), t(5), 0, -t(3), t(1), -t(4), t(3), 0, t(2),
      0, 0, 0, 0;
}

MatrixXd wedgeRight6(const Vector4d& p) {
  MatrixXd wl(4, 6);
  double p1 = p(0);
  double p2 = p(1);
  double p3 = p(2);
  double p4 = p(3);
  wl << p4, 0, 0, 0, p3, -p2, 0, p4, 0, -p3, 0, p1, 0, 0, p4, p2, -p1, 0, 0, 0,
      0, 0, 0, 0;
  return wl;
}

Vector6d vee6(const Matrix4d& T) {
  assert((T.block<3, 3>(0, 0) + T.block<3, 3>(0, 0).transpose()).norm() < 1e-7);
  assert(T.bottomRows(1).norm() < 1e-7);
  Vector6d t_vee;
  double v1 = T(0, 3);
  double v2 = T(1, 3);
  double v3 = T(2, 3);
  double w1 = T(2, 1);
  double w2 = T(0, 2);
  double w3 = T(1, 0);
  t_vee << v1, v2, v3, w1, w2, w3;
  return t_vee;
}

RUT::Vector4d rpy2quat(double roll, double pitch, double yaw) {
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  RUT::Vector4d quat;
  quat[0] = cr * cp * cy + sr * sp * sy;
  quat[1] = sr * cp * cy - cr * sp * sy;
  quat[2] = cr * sp * cy + sr * cp * sy;
  quat[3] = cr * cp * sy - sr * sp * cy;
  return quat;
}

void rpy2quat(double roll, double pitch, double yaw, double* quat) {
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  quat[0] = cr * cp * cy + sr * sp * sy;
  quat[1] = sr * cp * cy - cr * sp * sy;
  quat[2] = cr * sp * cy + sr * cp * sy;
  quat[3] = cr * cp * sy - sr * sp * cy;
}

// Axis-angle to matrix
// input:
//   theta: scalar
//   n: 3 x 1
// output:
// m: 3x3
Matrix3d aa2mat(const double theta, const Vector3d& n) {
  Vector3d n_unit = n.normalized();
  Matrix3d N = wedge(n_unit);
  // m = eye(3) + sin(theta)*N + (1-cos(theta))*N*N;
  Matrix3d m = Matrix3d::Identity() + std::sin(theta) * N +
               (1 - std::cos(theta)) * N * N;
  return m;
}

void aa2quat(const double theta, const Vector3d& n, Vector4d& q) {
  q[0] = std::cos(theta / 2);
  q.tail(3) = std::sin(theta / 2) * n.normalized();
}

Vector4d aa2quat(const double theta, const Vector3d& n) {
  Vector4d q;
  q[0] = std::cos(theta / 2);
  q.tail(3) = std::sin(theta / 2) * n.normalized();
  return q;
}

Matrix3d quat2SO3(const Vector4d& q) {
  Quaterniond quat(q(0), q(1), q(2), q(3));
  return quat.normalized().toRotationMatrix();
}

Matrix3d quat2SO3(const Quaterniond& q) {
  return q.normalized().toRotationMatrix();
}

Matrix3d quat2SO3(double qw, double qx, double qy, double qz) {
  Quaterniond q(qw, qx, qy, qz);
  return q.normalized().toRotationMatrix();
}

void quat2aa(const Vector4d& q, Vector3d& aa) {
  double theta = 2 * acos(q(0));
  if (theta < kEpsilon) {
    aa = Vector3d::Zero();
  } else {
    aa = theta * q.tail(3).normalized();
  }
}

Matrix3d so32SO3(const Vector3d& v) {
  double theta = v.norm();
  if (theta > kEpsilon) {
    Vector3d vn = v / theta;
    Matrix3d v_wedge = wedge(vn);
    Matrix3d SO3;
    SO3 = Matrix3d::Identity() + v_wedge * sin(theta) +
          v_wedge * v_wedge * (1.0 - cos(theta));
    return SO3;
  } else {
    return Matrix3d::Identity();
  }
}

Vector3d SO32so3(const Matrix3d& R) {
  Vector3d so3;
  double temp_arg_to_cos = (R.trace() - 1.0) / 2.0;
  truncate(&temp_arg_to_cos, -1.0, 1.0);
  double theta = acos(temp_arg_to_cos);
  if (fabs(theta) < kEpsilon) {
    so3(0) = 1.0;
    so3(1) = 0.0;
    so3(2) = 0.0;
  } else {
    so3(0) = R(2, 1) - R(1, 2);
    so3(1) = R(0, 2) - R(2, 0);
    so3(2) = R(1, 0) - R(0, 1);
    so3 /= 2.0 * sin(theta);
  }
  so3 *= theta;
  return so3;
}

void SO32so3(const Matrix3d& R, double* so3) {
  double temp_arg_to_cos = (R.trace() - 1.0) / 2.0;
  truncate(&temp_arg_to_cos, -1.0, 1.0);
  double theta = acos(temp_arg_to_cos);
  if (fabs(theta) < kEpsilon) {
    so3[0] = 1.0;
    so3[1] = 0.0;
    so3[2] = 0.0;
  } else {
    so3[0] = (R(2, 1) - R(1, 2)) / (2.0 * sin(theta));
    so3[1] = (R(0, 2) - R(2, 0)) / (2.0 * sin(theta));
    so3[2] = (R(1, 0) - R(0, 1)) / (2.0 * sin(theta));
  }
  so3[0] *= theta;
  so3[1] *= theta;
  so3[2] *= theta;
}

void so32quat(const Vector3d& so3, double* q) {
  double theta = so3.norm();
  if (theta < kEpsilon) {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  } else {
    // q = [cos(theta/2); sin(theta/2)*so3/theta];
    double sin_theta = sin(theta / 2.0) / theta;
    q[0] = cos(theta / 2.0);
    q[1] = so3(0) * sin_theta;
    q[2] = so3(1) * sin_theta;
    q[3] = so3(2) * sin_theta;
  }
}

void so32quat(const Vector3d& so3, Vector4d& q) {
  double theta = so3.norm();
  if (theta < kEpsilon) {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  } else {
    // q = [cos(theta/2); sin(theta/2)*so3/theta];
    double sin_theta = sin(theta / 2.0) / theta;
    q[0] = cos(theta / 2.0);
    q[1] = so3(0) * sin_theta;
    q[2] = so3(1) * sin_theta;
    q[3] = so3(2) * sin_theta;
  }
}

void so32quat(const double* so3, Eigen::Ref<RUT::Vector4d> q) {
  double theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);
  if (theta < kEpsilon) {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  } else {
    // q = [cos(theta/2); sin(theta/2)*so3/theta];
    double sin_theta = sin(theta / 2.0) / theta;
    q[0] = cos(theta / 2.0);
    q[1] = so3[0] * sin_theta;
    q[2] = so3[1] * sin_theta;
    q[3] = so3[2] * sin_theta;
  }
}

void SO32quat(const Matrix3d& SO3, Eigen::Ref<RUT::Vector4d> q) {
  // double so3[3];
  // SO32so3(SO3, so3);
  // so32quat(so3, q);

  // Eigen implementation assumes input matrix is strictly orthogonal and is not
  // numerically stable sometimes.
  Quaterniond q_eigen(SO3);
  q_eigen.normalize();
  // convert q_eigen to a vector q
  // Note q = q_eigen.coeffs() uses (w, x, y, z) ordering, which is not the convention we use
  q[0] = q_eigen.w();
  q[1] = q_eigen.x();
  q[2] = q_eigen.y();
  q[3] = q_eigen.z();
}

void SO32quat(const Matrix3d& SO3, double* q) {
  // Eigen implementation assumes input matrix is strictly orthogonal and is not
  // numerically stable sometimes.
  Quaterniond q_eigen(SO3);
  q_eigen.normalize();
  q[0] = q_eigen.w();
  q[1] = q_eigen.x();
  q[2] = q_eigen.y();
  q[3] = q_eigen.z();
  // double r11 = SO3(0,0);
  // double r12 = SO3(0,1);
  // double r13 = SO3(0,2);
  // double r21 = SO3(1,0);
  // double r22 = SO3(1,1);
  // double r23 = SO3(1,2);
  // double r31 = SO3(2,0);
  // double r32 = SO3(2,1);
  // double r33 = SO3(2,2);
  // q1_sq = 0.25*(1.0+r11+r22+r33);
  // q2_sq = 0.25*(1.0+r11-r22-r33);
  // q3_sq = 0.25*(1.0-r11+r22-r33);
  // q4_sq = 0.25*(1.0-r11-r22+r33);
}

Matrix4d pose2SE3(const double* pose) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3(0, 3) = pose[0];
  SE3(1, 3) = pose[1];
  SE3(2, 3) = pose[2];
  SE3.block<3, 3>(0, 0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4d pose2SE3(const Vector7d& pose) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3(0, 3) = pose[0];
  SE3(1, 3) = pose[1];
  SE3(2, 3) = pose[2];
  SE3.block<3, 3>(0, 0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4d posemm2SE3(const double* pose) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3(0, 3) = pose[0] / 1000.0;
  SE3(1, 3) = pose[1] / 1000.0;
  SE3(2, 3) = pose[2] / 1000.0;
  SE3.block<3, 3>(0, 0) = quat2SO3(pose[3], pose[4], pose[5], pose[6]);
  return SE3;
}

Matrix4d se32SE3(const Vector6d& twist) {
  Matrix4d SE3 = Matrix4d::Identity();
  double theta = twist.tail(3).norm();
  if (theta < kEpsilon) {
    // no rotation
    SE3(0, 3) = twist(0);
    SE3(1, 3) = twist(1);
    SE3(2, 3) = twist(2);
  } else {
    Vector3d v = twist.head(3);
    Vector3d w = twist.tail(3);
    Matrix3d R = so32SO3(w);
    v /= theta;
    w /= theta;
    SE3.block<3, 3>(0, 0) = R;
    SE3.block<3, 1>(0, 3) = (Matrix3d::Identity() - R) * (w.cross(v)) +
                            w * w.transpose() * v * theta;
  }
  return SE3;
}

Matrix4d spt2SE3(const Vector6d& spt) {
  Matrix4d SE3 = Matrix4d::Identity();
  SE3.block<3, 3>(0, 0) = so32SO3(spt.tail(3));
  SE3.block<3, 1>(0, 3) = spt.head(3);
  return SE3;
}

Matrix4d SE3Inv(const Matrix4d& SE3) {
  Matrix4d SE3_inv = Matrix4d::Identity();
  SE3_inv.block<3, 1>(0, 3) =
      -SE3.block<3, 3>(0, 0).transpose() * SE3.block<3, 1>(0, 3);
  SE3_inv.block<3, 3>(0, 0) = SE3.block<3, 3>(0, 0).transpose();
  return SE3_inv;
}

Vector6d SE32se3(const Matrix4d& SE3) {
  Vector3d p = SE3.block<3, 1>(0, 3);
  Vector3d omega = SO32so3(SE3.block<3, 3>(0, 0));
  double theta = omega.norm();
  if (theta < kEpsilon) {
    Vector6d se3;
    se3 << p(0), p(1), p(2), 0, 0, 0;
    return se3;
  } else {
    omega /= theta;
    Matrix3d M =
        (Matrix3d::Identity() - wedge(omega * theta).exp()) * wedge(omega) +
        omega * omega.transpose() * theta;
    Vector6d se3;
    se3.head(3) = M.fullPivLu().solve(p);
    se3.tail(3) = omega;
    se3 *= theta;
    return se3;
  }
}

Vector6d SE32spt(const Matrix4d& SE3) {
  Vector6d spt;
  spt.head(3) = SE3.block<3, 1>(0, 3);
  spt.tail(3) = SO32so3(SE3.block<3, 3>(0, 0));
  return spt;
}

Matrix6d SE32Adj(const Matrix4d& SE3) {
  Matrix6d Adj = Matrix6d::Zero();
  Adj.topLeftCorner(3, 3) = SE3.topLeftCorner(3, 3);
  Adj.bottomRightCorner(3, 3) = SE3.topLeftCorner(3, 3);
  Adj.topRightCorner(3, 3) =
      wedge(SE3.block<3, 1>(0, 3)) * SE3.topLeftCorner(3, 3);
  return Adj;
}

void SE32Pose(const Matrix4d& SE3, Vector7d& pose) {
  pose.head<3>() = SE3.block<3, 1>(0, 3);
  SO32quat(SE3.block<3, 3>(0, 0), pose.block<4, 1>(3, 0));
}

void SE32Pose(const Matrix4d& SE3, double* pose) {
  pose[0] = SE3(0, 3);
  pose[1] = SE3(1, 3);
  pose[2] = SE3(2, 3);
  SO32quat(SE3.block<3, 3>(0, 0), pose + 3);
}

void SE32Posemm(const Matrix4d& SE3, double* pose) {
  pose[0] = SE3(0, 3) * 1000.0;
  pose[1] = SE3(1, 3) * 1000.0;
  pose[2] = SE3(2, 3) * 1000.0;
  SO32quat(SE3.block<3, 3>(0, 0), pose + 3);
}

Matrix6d JacobianSpt2BodyV(const Matrix3d& R) {
  Matrix6d Jac;
  Jac = Matrix6d::Identity();
  Jac(3, 3) = R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2) + R(2, 2) * R(2, 2);
  Jac(3, 5) = -R(0, 0) * R(0, 2) - R(1, 0) * R(1, 2) - R(2, 0) * R(2, 2);
  Jac(4, 3) = -R(0, 0) * R(0, 1) - R(1, 0) * R(1, 1) - R(2, 0) * R(2, 1);
  Jac(4, 4) = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0) + R(2, 0) * R(2, 0);
  Jac(5, 4) = -R(0, 1) * R(0, 2) - R(1, 1) * R(1, 2) - R(2, 1) * R(2, 2);
  Jac(5, 5) = R(0, 1) * R(0, 1) + R(1, 1) * R(1, 1) + R(2, 1) * R(2, 1);

  return Jac;
}

CartesianPose getFrameFromTwoPoint(const Vector3d& p1, const Vector3d& p2) {
  //  X: p2-p1
  //  Y: world Z.cross(X)
  //  Z: X.cross(Y)
  Vector3d X = (p2 - p1).normalized();
  Vector3d Y = (Vector3d::UnitZ().cross(X)).normalized();
  Vector3d Z = X.cross(Y);
  Matrix3d R_WC;
  R_WC << X, Y, Z;
  Vector3d p_WC = 0.5 * (p2 + p1);
  RUT::CartesianPose pose_WC(R_WC, p_WC);
  return pose_WC;
}

Matrix3d rotX(double angle_rad) {
  Vector3d x;
  x << 1, 0, 0;
  return aa2mat(angle_rad, x);
}
Matrix3d rotY(double angle_rad) {
  Vector3d y;
  y << 0, 1, 0;
  return aa2mat(angle_rad, y);
}
Matrix3d rotZ(double angle_rad) {
  Vector3d z;
  z << 0, 0, 1;
  return aa2mat(angle_rad, z);
}

Matrix3d getRFromZ(const Vector3d& z) {
  Vector3d t0, t1;
  t0 << 1, 0, 0;
  t1 << 0, 1, 0;
  Vector3d x, y;
  if (fabs(z.dot(t0)) > 0.8) {
    x = z.cross(t1);
  } else {
    x = z.cross(t0);
  }
  x.normalize();
  y = z.cross(x);
  Matrix3d R;
  R << x, y, z;
  return R;
}

Eigen::Quaternionf QuatMTimes(const Eigen::Quaternionf& q1,
                              const Eigen::Quaternionf& q2) {
  float s1 = q1.w();
  Eigen::Vector3f v1(q1.x(), q1.y(), q1.z());

  float s2 = q2.w();
  Eigen::Vector3f v2(q2.x(), q2.y(), q2.z());

  float cr_v1 = v1(1) * v2(2) - v1(2) * v2(1);
  float cr_v2 = v1(2) * v2(0) - v1(0) * v2(2);
  float cr_v3 = v1(0) * v2(1) - v1(1) * v2(0);

  Eigen::Quaternionf qp;
  qp.w() = s1 * s2 - v2.dot(v1);
  qp.x() = v2(0) * s1 + s2 * v1(0) + cr_v1;
  qp.y() = v2(1) * s1 + s2 * v1(1) + cr_v2;
  qp.z() = v2(2) * s1 + s2 * v1(2) + cr_v3;

  return qp;
}

Eigen::Quaterniond QuatMTimes(const Eigen::Quaterniond& q1,
                              const Eigen::Quaterniond& q2) {
  double s1 = q1.w();
  Vector3d v1(q1.x(), q1.y(), q1.z());

  double s2 = q2.w();
  Vector3d v2(q2.x(), q2.y(), q2.z());

  double cr_v1 = v1(1) * v2(2) - v1(2) * v2(1);
  double cr_v2 = v1(2) * v2(0) - v1(0) * v2(2);
  double cr_v3 = v1(0) * v2(1) - v1(1) * v2(0);

  Eigen::Quaterniond qp;
  qp.w() = s1 * s2 - v2.dot(v1);
  qp.x() = v2(0) * s1 + s2 * v1(0) + cr_v1;
  qp.y() = v2(1) * s1 + s2 * v1(1) + cr_v2;
  qp.z() = v2(2) * s1 + s2 * v1(2) + cr_v3;

  return qp;
}

float angBTquat(const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2) {
  Eigen::Quaternionf q = QuatMTimes(q1.normalized().inverse(), q2.normalized());

  float ang = 2.0f * acos(q.w());  // acos: [0, pi]

  if (ang > PIf) {
    ang = 2.0f * PIf - ang;
  }
  return fabs(ang);
}

double angBTquat(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
  double dot = q1.normalized().dot(q2.normalized());
  double cos_value = 2.0 * dot * dot - 1.0;
  double ang;
  if (cos_value > 0.999999)
    ang = 0;
  else if (cos_value < -0.999999)
    ang = PI;
  else
    ang = acos(cos_value);
  if (ang > PI) {
    ang = 2.0 * PI - ang;
  }
  return fabs(ang);
}

double angBTquat(const RUT::VectorXd& q1, const RUT::VectorXd& q2) {
  double dot = q1.tail<4>().normalized().dot(q2.tail<4>().normalized());
  double cos_value = 2.0 * dot * dot - 1.0;
  double ang;
  if (cos_value > 0.999999)
    ang = 0;
  else if (cos_value < -0.999999)
    ang = PI;
  else
    ang = acos(cos_value);
  if (ang > PI) {
    ang = 2.0 * PI - ang;
  }
  return fabs(ang);
}

double angBTVec(Vector3d x, Vector3d b, Vector3d z, bool nonnegative) {
  x.normalize();
  b.normalize();
  if (z.norm() < 1e-5) {
    return acos(x.dot(b));
  } else {
    z.normalize();
    double ang = atan2(x.cross(b).dot(z), x.dot(b));
    if (nonnegative)
      ang = (ang < 0) ? 2 * PI + ang : ang;
    return ang;
  }
}

int SlerpFixAngle(const Vector4d& qa, const Vector4d& qb, Vector4d& qm,
                  float angle) {
  assert(abs(qa.norm() - 1) < 0.01);
  assert(abs(qb.norm() - 1) < 0.01);

  // Calculate angle between them.
  double cosHalfTheta =
      qa(0) * qb(0) + qa(1) * qb(1) + qa(2) * qb(2) + qa(3) * qb(3);
  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (abs(cosHalfTheta) >= 1.0) {
    qm(0) = qa(0);
    qm(1) = qa(1);
    qm(2) = qa(2);
    qm(3) = qa(3);
    return 0;
  }
  // Calculate temporary values. acos return [0, PI]
  double halfTheta = acos(cosHalfTheta);

  // qa-qb is smaller than angle. Return qb
  if (2 * halfTheta < angle) {
    qm(0) = qb(0);
    qm(1) = qb(1);
    qm(2) = qb(2);
    qm(3) = qb(3);
    return 0;
  }

  double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // throw error
  if (fabs(sinHalfTheta) < 0.001) {
    // qm(0) = qa(0); qm(1) = qa(1); qm(2) = qa(2);qm(3) = qa(3);
    std::cout << "[RobotUtilities::SlerpFixAngle] [Commanded orientation is"
                 " 180 deg away from current orientation! ] Motion Stopped."
              << std::endl;
    exit(1);
  }

  angle /= 2;
  double ratioA = sin(halfTheta - angle) / sinHalfTheta;
  double ratioB = sin(angle) / sinHalfTheta;

  // calculate Quaternion.
  qm(0) = (qa(0) * ratioA + qb(0) * ratioB);
  qm(1) = (qa(1) * ratioA + qb(1) * ratioB);
  qm(2) = (qa(2) * ratioA + qb(2) * ratioB);
  qm(3) = (qa(3) * ratioA + qb(3) * ratioB);
  return 1;
}

CartesianPose::CartesianPose() {
  // q_ = new Eigen::Quaterniond(1, 0, 0, 0);
  qw_ = 1;
  qx_ = 0;
  qy_ = 0;
  qz_ = 0;
  p_ = new Vector3d(0, 0, 0);
  R_ = new Matrix3d();
  *R_ = Matrix3d::Identity();
}

CartesianPose::~CartesianPose() {
  // delete q_;
  // std::cout << "[CartesianPose] deleted q_" << std::endl;
  delete p_;
  delete R_;
}

CartesianPose::CartesianPose(std::vector<double> pose) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  (*p_)[0] = pose[0];
  (*p_)[1] = pose[1];
  (*p_)[2] = pose[2];
  double norm = sqrt(pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] +
                     pose[6] * pose[6]);
  assert(abs(norm - 1.0) < 0.1);
  qw_ = pose[3] / norm;
  qx_ = pose[4] / norm;
  qy_ = pose[5] / norm;
  qz_ = pose[6] / norm;
  Eigen::Quaterniond q = Eigen::Quaterniond(qw_, qx_, qy_, qz_);
  *R_ = q.toRotationMatrix();
}

CartesianPose::CartesianPose(double* pose) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  (*p_)[0] = pose[0];
  (*p_)[1] = pose[1];
  (*p_)[2] = pose[2];
  double norm = sqrt(pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] +
                     pose[6] * pose[6]);
  assert(abs(norm - 1.0) < 0.1);
  qw_ = pose[3] / norm;
  qx_ = pose[4] / norm;
  qy_ = pose[5] / norm;
  qz_ = pose[6] / norm;
  Eigen::Quaterniond q = Eigen::Quaterniond(qw_, qx_, qy_, qz_);
  *R_ = q.toRotationMatrix();
}

CartesianPose::CartesianPose(const Eigen::MatrixXd& T) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  if ((T.rows() == 4) && (T.cols() == 4)) {
    *p_ = T.block<3, 1>(0, 3);
    *R_ = T.block<3, 3>(0, 0);
    Eigen::Quaterniond q = Eigen::Quaterniond(*R_);
    if (fabs(q.norm() - 1) > 0.1) {
      std::cout << "[cpplibrary] Error: input matrix is not orthonormal."
                << std::endl;
      std::cout << "R:\n" << *R_ << std::endl;
      std::cout << "R'*R:\n" << R_->transpose() * (*R_) << std::endl;
    }
    qw_ = q.w();
    qx_ = q.x();
    qy_ = q.y();
    qz_ = q.z();
  } else if ((T.rows() == 7) && (T.cols() == 1)) {
    (*p_)[0] = T(0);
    (*p_)[1] = T(1);
    (*p_)[2] = T(2);
    double norm = sqrt(T(3) * T(3) + T(4) * T(4) + T(5) * T(5) + T(6) * T(6));
    assert(abs(norm - 1.0) < 0.1);
    qw_ = T(3) / norm;
    qx_ = T(4) / norm;
    qy_ = T(5) / norm;
    qz_ = T(6) / norm;
    Eigen::Quaterniond q = Eigen::Quaterniond(qw_, qx_, qy_, qz_);
    *R_ = q.toRotationMatrix();
  } else {
    std::cerr << "[utilities.CartesianPose] wrong input matrix size. "
              << T.rows() << " x " << T.cols() << std::endl;
  }
}

CartesianPose::CartesianPose(const Eigen::Isometry3d& iso) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  *p_ = iso.translation();
  *R_ = iso.rotation();
  Eigen::Quaterniond q = Eigen::Quaterniond(*R_);
  qw_ = q.w();
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
}

CartesianPose::CartesianPose(const Eigen::Quaterniond& q, const Vector3d& p) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  *p_ = p;
  qw_ = q.w();
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
  *R_ = q.toRotationMatrix();
}

CartesianPose::CartesianPose(const Matrix3d& R, const Vector3d& p) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  *p_ = p;
  *R_ = R;
  Eigen::Quaterniond q = Eigen::Quaterniond(*R_);
  qw_ = q.w();
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
}

CartesianPose::CartesianPose(CartesianPose&& pose) : p_(nullptr), R_(nullptr) {
  p_ = pose.p_;
  R_ = pose.R_;

  qw_ = pose.qw_;
  qx_ = pose.qx_;
  qy_ = pose.qy_;
  qz_ = pose.qz_;

  pose.p_ = nullptr;
  pose.R_ = nullptr;
}

CartesianPose& CartesianPose::operator=(CartesianPose&& pose) {
  if (this != &pose) {
    delete p_;
    delete R_;
    p_ = pose.p_;
    R_ = pose.R_;
    qw_ = pose.qw_;
    qx_ = pose.qx_;
    qy_ = pose.qy_;
    qz_ = pose.qz_;
    pose.p_ = nullptr;
    pose.R_ = nullptr;
  }
  return *this;
}  // move assignment

CartesianPose::CartesianPose(const CartesianPose& pose) {
  // q_ = new Eigen::Quaterniond();
  p_ = new Vector3d();
  R_ = new Matrix3d();
  *p_ = *pose.p_;
  *R_ = *pose.R_;
  qw_ = pose.qw_;
  qx_ = pose.qx_;
  qy_ = pose.qy_;
  qz_ = pose.qz_;
}

// copy assignment
CartesianPose& CartesianPose::operator=(const CartesianPose& pose) {
  *p_ = *pose.p_;
  *R_ = *pose.R_;
  qw_ = pose.qw_;
  qx_ = pose.qx_;
  qy_ = pose.qy_;
  qz_ = pose.qz_;
  return *this;
}

CartesianPose CartesianPose::Identity() {
  return CartesianPose(Eigen::Matrix4d::Identity());
}

void CartesianPose::setRotationMatrix(const Matrix3d& R) {
  *R_ = R;
  Eigen::Quaterniond q = Eigen::Quaterniond(*R_);
  qw_ = q.w();
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
}

void CartesianPose::setQuaternion(const Eigen::Quaterniond& q) {
  qw_ = q.w();
  qx_ = q.x();
  qy_ = q.y();
  qz_ = q.z();
  *R_ = q.toRotationMatrix();
}

void CartesianPose::setQuaternion(const std::vector<double>& q_vec) {
  qw_ = q_vec[0];
  qx_ = q_vec[1];
  qy_ = q_vec[2];
  qz_ = q_vec[3];
  Eigen::Quaterniond q = Eigen::Quaterniond(qw_, qx_, qy_, qz_);
  *R_ = q.toRotationMatrix();
}

void CartesianPose::setXYZ(const Vector3d& p) {
  *p_ = p;
}

void CartesianPose::setXYZ(const std::vector<double>& p) {
  (*p_)[0] = p[0];
  (*p_)[1] = p[1];
  (*p_)[2] = p[2];
}

void CartesianPose::setX(double x) {
  (*p_)[0] = x;
}
void CartesianPose::setY(double y) {
  (*p_)[1] = y;
}
void CartesianPose::setZ(double z) {
  (*p_)[2] = z;
}

void CartesianPose::scaleXYZ(double scale) {
  (*p_)[0] *= scale;
  (*p_)[1] *= scale;
  (*p_)[2] *= scale;
}

Matrix3d CartesianPose::getRotationMatrix() const {
  return *R_;
}

Eigen::Quaterniond CartesianPose::getQuaternion() const {
  return Eigen::Quaterniond(qw_, qx_, qy_, qz_);
}

Vector4d CartesianPose::getQuaternionVec() const {
  Vector4d vec;
  vec << qw_, qx_, qy_, qz_;
  return vec;
}

Vector3d CartesianPose::getXYZ() const {
  return *p_;
}

double CartesianPose::getX() const {
  return (*p_)(0);
}
double CartesianPose::getY() const {
  return (*p_)(1);
}
double CartesianPose::getZ() const {
  return (*p_)(2);
}

Vector3d CartesianPose::getXAxis() const {
  return R_->block<3, 1>(0, 0);
}

Vector3d CartesianPose::getYAxis() const {
  return R_->block<3, 1>(0, 1);
}

Vector3d CartesianPose::getZAxis() const {
  return R_->block<3, 1>(0, 2);
}

Eigen::Matrix4d CartesianPose::getTransformMatrix() const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = *R_;
  T.block<3, 1>(0, 3) = *p_;
  return T;
}

Eigen::Isometry3d CartesianPose::getIsometry3d() const {
  Eigen::Isometry3d transform =
      Eigen::Translation<double, 3>((*p_)[0], (*p_)[1], (*p_)[2]) *
      Eigen::Quaterniond(qw_, qx_, qy_, qz_);
  return transform;
}

std::vector<double> CartesianPose::getVector() const {
  std::vector<double> vec = {(*p_)[0], (*p_)[1], (*p_)[2], qw_, qx_, qy_, qz_};
  return vec;
}

void CartesianPose::getArray(double* array) const {
  array[0] = (*p_)[0];
  array[1] = (*p_)[1];
  array[2] = (*p_)[2];
  array[3] = qw_;
  array[4] = qx_;
  array[5] = qy_;
  array[6] = qz_;
}

CartesianPose CartesianPose::operator*(const CartesianPose& pose) const {
  Eigen::Matrix4d T = getTransformMatrix() * pose.getTransformMatrix();
  return CartesianPose(T);
}

Vector3d CartesianPose::operator*(const Vector3d& p) const {
  return (*R_) * p + (*p_);
}

CartesianPose CartesianPose::inv() const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R_->transpose();
  T.block<3, 1>(0, 3) = -R_->transpose() * (*p_);
  CartesianPose pose(T);
  return pose;
}

CartesianPose CartesianPose::increTowards(const CartesianPose& pose,
                                          double max_trans,
                                          double max_rotation) {
  // rotation
  Vector4d q_vec;
  SlerpFixAngle(this->getQuaternionVec(), pose.getQuaternionVec(), q_vec,
                max_rotation);
  // translation
  Vector3d p0 = this->getXYZ();
  Vector3d p1 = pose.getXYZ();
  Vector3d p;
  if ((p0 - p1).norm() < max_trans) {
    p = p1;
  } else {
    p = p0 + (p1 - p0).normalized() * max_trans;
  }
  std::vector<double> pose_vec = {p(0),     p(1),     p(2),    q_vec(0),
                                  q_vec(1), q_vec(2), q_vec(3)};
  return CartesianPose(pose_vec);
}

Vector3d CartesianPose::transformVec(const Vector3d& v) const {
  return (*R_) * v;
}

Vector3d CartesianPose::transformPoint(const Vector3d& p) const {
  return (*R_) * p + (*p_);
}

Eigen::MatrixXd CartesianPose::transformPoints(
    const Eigen::MatrixXd& ps) const {
  assert(ps.rows() == 3);
  int num_points = ps.cols();

  Eigen::MatrixXd ps_transformed = (*R_) * ps;
  for (int i = 0; i < num_points; ++i) {
    ps_transformed(0, i) += (*p_)[0];
    ps_transformed(1, i) += (*p_)[1];
    ps_transformed(2, i) += (*p_)[2];
  }
  return ps_transformed;
}

Eigen::Quaterniond CartesianPose::transformQuat(
    const Eigen::Quaterniond& q) const {
  Matrix3d R = q.toRotationMatrix();
  return Eigen::Quaterniond((*R_) * R);
}

double CartesianPose::distBTPose(const CartesianPose& pose,
                                 double ratio) const {
  Eigen::Quaterniond q1(qw_, qx_, qy_, qz_);
  Eigen::Quaterniond q2(pose.qw_, pose.qx_, pose.qy_, pose.qz_);
  double angle = angBTquat(q1, q2);
  double dist = (*p_ - *pose.p_).norm();
  return angle * ratio + dist;
}

void CartesianPose::print() const {
  std::cout << "p:\n";
  std::cout << *p_ << std::endl;
  std::cout << "R:\n";
  std::cout << *R_ << std::endl;
  std::cout << "q:\n";
  std::cout << qw_ << " " << qx_ << " " << qy_ << " " << qz_ << std::endl;
}

void CartesianPose::printPose() const {
  std::cout << p_->transpose() << ", ";
  std::cout << qw_ << " " << qx_ << " " << qy_ << " " << qz_ << std::endl;
}

std::string CartesianPose::poseString() const {
  std::string line = std::to_string((*p_)[0]) + ", " +
                     std::to_string((*p_)[1]) + ", " +
                     std::to_string((*p_)[2]) + " | " + std::to_string(qw_) +
                     ", " + std::to_string(qx_) + ", " + std::to_string(qy_) +
                     ", " + std::to_string(qz_);
  return line;
}

void double2float(const double* array_in, float* array_out, int n) {
  for (int i = 0; i < n; ++i)
    array_out[i] = array_in[i];
}
void float2double(const float* array_in, double* array_out, int n) {
  for (int i = 0; i < n; ++i)
    array_out[i] = array_in[i];
}

int findInVector(std::vector<int> vec, int ele) {
  std::vector<int>::iterator it = std::find(vec.begin(), vec.end(), ele);
  if (it != vec.end()) {
    return std::distance(vec.begin(), it);
  } else {
    return -1;
  }
}

std::vector<int> findInVector(std::vector<int> vec, std::vector<int> eles) {
  // Sort the vector
  std::sort(vec.begin(), vec.end());
  std::sort(eles.begin(), eles.end());

  // Initialise a vector to store the common values
  // and an iterator to traverse this vector
  std::vector<int> result(vec.size() + eles.size());
  std::vector<int>::iterator it;

  it = set_intersection(vec.begin(), vec.end(), eles.begin(), eles.end(),
                        result.begin());
  int num = std::distance(result.begin(), it);
  result.resize(num);
  // std::cout << "result size: " << result.size() << std::endl;
  // std::cout << "\nCommon elements:\n";
  // for (std::vector<int>::iterator st = result.begin(); st != it; ++st)
  //     std::cout << *st << ", ";
  // std::cout << '\n';
  // getchar();
  return result;
}

int findInEigenVector(const Eigen::VectorXi& vec, int ele) {
  std::vector<int> v_std;
  v_std.resize(vec.size());
  Eigen::VectorXi::Map(&v_std[0], vec.size()) = vec;
  return findInVector(v_std, ele);
}

Vector6d getPluckerLine(const Vector3d& p, const Vector3d& n) {
  Vector6d line;
  line << n.normalized(), p.cross(n.normalized());
  return line;
}
double reciprocalProduct(const Vector6d& line1, const Vector6d& line2) {
  return line1.head<3>().dot(line2.tail<3>()) +
         line2.head<3>().dot(line1.tail<3>());
}
double distBTPluckerLines(const Vector6d& line1, const Vector6d& line2) {
  return reciprocalProduct(line1, line2) /
         line1.head<3>().cross(line2.head<3>()).norm();
}
double angleBTPluckerLines(const Vector6d& line1, const Vector6d& line2) {
  Vector3d n1 = line1.head<3>();
  Vector3d n2 = line2.head<3>();
  double dot = std::fabs(n1.dot(n2));
  if (dot < 1e-10)
    return PI / 2;
  double cross = n1.cross(n2).norm();
  double tan_alpha = cross / dot;
  // std::cout << "[angleBTPluckerLines] n1: " << n1.transpose() << std::endl;
  // std::cout << "[angleBTPluckerLines] n2: " << n2.transpose() << std::endl;
  // std::cout << "[angleBTPluckerLines] dot: " << dot << std::endl;
  // std::cout << "[angleBTPluckerLines] cross: " << cross << std::endl;
  // std::cout << "[angleBTPluckerLines] sin_alpha: " << sin_alpha << std::endl;
  return std::atan(tan_alpha);
}

double distPoint2PluckerLine(const Vector3d& p, const Vector6d& line) {
  Vector3d q = line.head<3>();
  Vector3d q0 = line.tail<3>();
  return (q0 - p.cross(q)).norm() / q.norm();
}

/////////////////////////////////////////////////////////////////////////
//                      Motion Planning
/////////////////////////////////////////////////////////////////////////
void MotionPlanningLinear(const double* pose0, const double* pose_set,
                          const int Nsteps, MatrixXd* pose_traj) {
  Eigen::Quaternionf q0(pose0[3], pose0[4], pose0[5], pose0[6]);
  Eigen::Quaternionf qset(pose_set[3], pose_set[4], pose_set[5], pose_set[6]);
  Eigen::Quaternionf q;

  pose_traj->resize(7, Nsteps);
  for (int i = 0; i < Nsteps; ++i) {
    q = q0.slerp(double(i + 1) / double(Nsteps), qset);
    (*pose_traj)(0, i) =
        (pose0[0] * double(Nsteps - i - 1) + pose_set[0] * double(i + 1)) /
        double(Nsteps);
    (*pose_traj)(1, i) =
        (pose0[1] * double(Nsteps - i - 1) + pose_set[1] * double(i + 1)) /
        double(Nsteps);
    (*pose_traj)(2, i) =
        (pose0[2] * double(Nsteps - i - 1) + pose_set[2] * double(i + 1)) /
        double(Nsteps);
    (*pose_traj)(3, i) = q.w();
    (*pose_traj)(4, i) = q.x();
    (*pose_traj)(5, i) = q.y();
    (*pose_traj)(6, i) = q.z();
  }
}

void TrapezodialTrajectory(double x_f, double a_max, double v_max, double* t1,
                           double* t2, int Nsteps, double* x_traj) {
  assert(x_f > -1e-7);
  assert(a_max > 0);
  assert(v_max > 0);
  double delta_x1 = v_max * v_max / 2.0 / a_max;
  if (2.0 * delta_x1 > x_f) {
    double v_max_actual = std::sqrt(x_f * a_max);
    *t1 = v_max_actual / a_max;
    *t2 = 0;
  } else {
    *t1 = v_max / a_max;
    double delta_x2 = x_f - 2 * delta_x1;
    *t2 = delta_x2 / v_max;
  }
  if (Nsteps == 0)
    return;

  double t_all = 2 * (*t1) + (*t2);
  for (int i = 0; i < Nsteps; ++i) {
    double ratio = double(i) / double(Nsteps - 1);  // [0, 1]
    double t = ratio * t_all;
    if (t < *t1) {
      x_traj[i] = 0.5 * a_max * t * t;
    } else if (t < *t1 + *t2) {
      x_traj[i] = 0.5 * (*t1) * v_max + v_max * (t - (*t1));
    } else {
      x_traj[i] = x_f - 0.5 * a_max * (t_all - t) * (t_all - t);
    }
  }  // end for
}

void MotionPlanningTrapezodial(const double* pose0, const double* pose_set,
                               double a_max_trans, double v_max_trans,
                               double a_max_rot, double v_max_rot, double rate,
                               MatrixXd* pose_traj) {
  Vector3d p0, pf;
  p0 << pose0[0], pose0[1], pose0[2];
  pf << pose_set[0], pose_set[1], pose_set[2];

  Eigen::Quaterniond q0(pose0[3], pose0[4], pose0[5], pose0[6]);
  Eigen::Quaterniond qf(pose_set[3], pose_set[4], pose_set[5], pose_set[6]);
  double dist_trans = (p0 - pf).norm();
  double dist_rot = angBTquat(q0, qf);

  double t1_trans = 0;
  double t2_trans = 0;
  double t1_rot = 0;
  double t2_rot = 0;
  TrapezodialTrajectory(dist_trans, a_max_trans, v_max_trans, &t1_trans,
                        &t2_trans);
  TrapezodialTrajectory(dist_rot, a_max_rot, v_max_rot, &t1_rot, &t2_rot);
  int Nsteps = (int)std::round(
      std::max(2.0 * t1_trans + t2_trans, 2.0 * t1_rot + t2_rot) * rate);

  if (Nsteps > 2) {
    // need to do interpolation
    // get ratio
    double* r_trans = new double[Nsteps];
    double* r_rot = new double[Nsteps];
    TrapezodialTrajectory(dist_trans, a_max_trans, v_max_trans, &t1_trans,
                          &t2_trans, Nsteps, r_trans);
    TrapezodialTrajectory(dist_rot, a_max_rot, v_max_rot, &t1_rot, &t2_rot,
                          Nsteps, r_rot);

    // regularize r_trans and r_rot to [0, 1]
    // note here Nsteps > 2
    if (dist_trans > 1e-5)
      for (int i = 0; i < Nsteps; ++i)
        r_trans[i] /= dist_trans;
    else
      for (int i = 0; i < Nsteps; ++i)
        r_trans[i] = i / double(Nsteps - 1);

    if (dist_rot > 1e-5)
      for (int i = 0; i < Nsteps; ++i)
        r_rot[i] /= dist_rot;
    else
      for (int i = 0; i < Nsteps; ++i)
        r_rot[i] = i / double(Nsteps - 1);

    // compute the actual pose from r
    Eigen::Quaterniond q;
    pose_traj->resize(7, Nsteps);
    for (int i = 0; i < Nsteps; ++i) {
      q = q0.slerp(r_rot[i], qf);
      (*pose_traj)(0, i) =
          pose0[0] * (1.0 - r_trans[i]) + pose_set[0] * r_trans[i];
      (*pose_traj)(1, i) =
          pose0[1] * (1.0 - r_trans[i]) + pose_set[1] * r_trans[i];
      (*pose_traj)(2, i) =
          pose0[2] * (1.0 - r_trans[i]) + pose_set[2] * r_trans[i];
      (*pose_traj)(3, i) = q.w();
      (*pose_traj)(4, i) = q.x();
      (*pose_traj)(5, i) = q.y();
      (*pose_traj)(6, i) = q.z();
    }
    delete[] r_trans;
    delete[] r_rot;
  } else if (Nsteps == 2) {
    // no need for interpolation
    pose_traj->resize(7, 2);
    (*pose_traj)(0, 0) = pose0[0];
    (*pose_traj)(1, 0) = pose0[1];
    (*pose_traj)(2, 0) = pose0[2];
    (*pose_traj)(3, 0) = pose0[3];
    (*pose_traj)(4, 0) = pose0[4];
    (*pose_traj)(5, 0) = pose0[5];
    (*pose_traj)(6, 0) = pose0[6];
    (*pose_traj)(0, 1) = pose_set[0];
    (*pose_traj)(1, 1) = pose_set[1];
    (*pose_traj)(2, 1) = pose_set[2];
    (*pose_traj)(3, 1) = pose_set[3];
    (*pose_traj)(4, 1) = pose_set[4];
    (*pose_traj)(5, 1) = pose_set[5];
    (*pose_traj)(6, 1) = pose_set[6];
  } else {
    // no need for interpolation
    pose_traj->resize(7, 1);
    (*pose_traj)(0, 0) = pose_set[0];
    (*pose_traj)(1, 0) = pose_set[1];
    (*pose_traj)(2, 0) = pose_set[2];
    (*pose_traj)(3, 0) = pose_set[3];
    (*pose_traj)(4, 0) = pose_set[4];
    (*pose_traj)(5, 0) = pose_set[5];
    (*pose_traj)(6, 0) = pose_set[6];
  }
}

double Gaussian(double x, double var) {
  double k = 1.0 / var / std::sqrt(2.0 * PI);
  double exp = std::exp(-x * x / 2.0 / var / var);
  return k * exp;
}

}  // namespace RUT
