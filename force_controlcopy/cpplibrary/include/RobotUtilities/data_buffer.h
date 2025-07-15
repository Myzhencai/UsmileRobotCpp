#pragma once
#include <Eigen/Dense>
#include <deque>
#include <iostream>

namespace RUT {

template <class T>
class DataBufferBase {
 public:
  DataBufferBase() {};

  /**
   * @brief Initializes the data buffer.
   * 
   * @param buffer_size The max length of the buffer, set to -1 to allow arbitrary length.
   * @param nrows The number of rows for a data point.
   * @param ncols The number of columns for a data point.
   */
  void initialize(int buffer_size, int nrows = 1, int ncols = 1,
                  std::string buffer_name = "") {
    _buffer.clear();
    _buffer_size = buffer_size;
    this->_nrows = nrows;
    this->_ncols = ncols;
    this->_buffer_name = buffer_name;
  }

  T get(int i) { return _buffer[i]; }
  T operator[](int i) { return _buffer[i]; }
  /**
   * @brief Removes and returns the oldest data from the buffer.
   * 
   * @return The oldest data from the buffer.
   */
  const T pop() {
    T result = _buffer.front();
    _buffer.pop_front();
    return result;
  }

  void remove_last_k(int k) {
    for (int i = 0; i < k; i++) {
      _buffer.pop_back();
    }
  }

  void clear() { _buffer.clear(); }

  void put(const T& data) {
    if ((_buffer_size > 0) && (_buffer.size() >= _buffer_size)) {
      _buffer.pop_front();
    }
    _buffer.push_back(data);
  }

  /**
   * @brief Checks if the buffer is full. For non-fixed length buffer, always return false.
   * 
   */
  bool is_full() { return _buffer.size() == _buffer_size; }
  bool is_empty() { return _buffer.size() == 0; }
  int size() { return _buffer.size(); }

 protected:
  std::deque<T> _buffer;     ///< The buffer to store the data.
  int _buffer_size;          ///< The size of the buffer.
  int _nrows;                ///< The number of rows in the buffer.
  int _ncols;                ///< The number of columns in the buffer.
  std::string _buffer_name;  ///< The name of the buffer.
};

// Primary template, which works for Eigen::MatrixXd, Eigen::MatrixXf, etc.
template <class T>
class DataBuffer : public DataBufferBase<T> {
 public:
  const T get_last_k(int k) {
    if (this->_buffer.size() < k) {
      throw std::runtime_error(
          "Buffer size: " + std::to_string(this->_buffer.size()) +
          " is less than query size: " + std::to_string(k));
    }
    T result(this->_nrows, this->_ncols * k);
    int start_id = this->_buffer_size - k;
    for (int i = 0; i < k; i++) {
      result.block(0, i * this->_ncols, this->_nrows, this->_ncols) =
          this->_buffer[start_id + i];
    }
    return result;
  }
};

// Partial specialization for double
template <>
class DataBuffer<double> : public DataBufferBase<double> {
 public:
  /// @brief Get the last k data points and concatenate them into a single matrix
  /// @param k
  /// @return Data concatenated in the column direction: [data_1, data_2, ..., data_k]
  const Eigen::VectorXd get_last_k(int k) {
    if (this->_buffer.size() < k) {
      throw std::runtime_error(
          "Buffer size: " + std::to_string(this->_buffer.size()) +
          " is less than query size: " + std::to_string(k));
    }
    Eigen::VectorXd result(k);
    int start_id = this->_buffer_size - k;
    for (int i = 0; i < k; i++) {
      result(i) = this->_buffer[start_id + i];
    }
    return result;
  }
};

template <>
class DataBuffer<Eigen::VectorXd> : public DataBufferBase<Eigen::VectorXd> {
 public:
  /// @brief Get the last k data points and concatenate them into a single matrix
  /// @param k
  /// @return Data concatenated in the column direction: [data_1, data_2, ..., data_k]
  const Eigen::MatrixXd get_last_k(int k) {
    if (this->_buffer.size() < k) {
      throw std::runtime_error(
          "Buffer size: " + std::to_string(this->_buffer.size()) +
          " is less than query size: " + std::to_string(k));
    }
    Eigen::MatrixXd result(this->_nrows, k);
    int start_id = this->_buffer_size - k;
    for (int i = 0; i < k; i++) {
      result.block(0, i * this->_ncols, this->_nrows, this->_ncols) =
          this->_buffer[start_id + i];
    }
    return result;
  }
};

}  // namespace RUT