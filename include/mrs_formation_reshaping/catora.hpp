/* This is an implementation of the Hungarian algorithm and assignment part of CAT-ORA in C++
 * The code is based on http://csclab.murraystate.edu/~bob.pilgrim/445/munkres.html and its adaptation
 * written by Fernando B. Giannasi available from https://github.com/phoemur/hungarian_algorithm.
 * Compared to the previous version, this implementation of Hungarian algorithm includes several changes
 * providing significant decrease of computational complexity.  */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <string>
#include <type_traits>
#include <vector>
#include <set>
#include <Eigen/Dense>
#include <iomanip>
#include <chrono>

#define SCALE_FACTOR 100
#define SF_INVERTED_SQRT 0.1

namespace Catora
{

struct Edge
{
  int row;
  int col;
};

struct Node
{
  std::vector<Edge>        initial_solution;
  Edge                     restricted_edge;
  std::vector<long double> row_duals;
  std::vector<long double> col_duals;
  int                      depth;
};

template <typename T>
struct RestrictedNode
{
  Edge restricted_edge;
  int  depth;
};

/* Utility function to print Matrix */
template <template <typename, typename...> class Container, typename T, typename... Args>
// disable for string, which is std::basic_string<char>, a container itself
typename std::enable_if<!std::is_convertible<Container<T, Args...>, std::string>::value && !std::is_constructible<Container<T, Args...>, std::string>::value,
                        std::ostream&>::type
operator<<(std::ostream& os, const Container<T, Args...>& con) {
  os << " ";
  for (auto& elem : con)
    os << elem << " ";

  os << "\n";
  return os;
}

/* CAT-ORA ALGORITHM //{ */

// | ---------------- CAT-ORA ALGORITHM SECTION --------------- |

/* Handle negative elements if present. If allowed = true, add abs(minval) to
 * every element to create one zero. Else throw an exception */
template <typename T>
void handle_negatives(std::vector<std::vector<T>>& matrix, bool allowed = true) {
  T minval = std::numeric_limits<T>::max();

  for (auto& elem : matrix) {
    for (auto& num : elem) {
      minval = std::min(minval, num);
    }
  }

  if (minval < 0) {
    if (!allowed) {  // throw
      throw std::runtime_error("Only non-negative values allowed");
    } else {  // add abs(minval) to every element to create one zero
      minval = abs(minval);

      for (auto& elem : matrix) {
        for (auto& num : elem) {
          num += minval;
        }
      }
    }
  }
}

/* Ensure that the matrix is square by the addition of dummy rows/columns if necessary */
template <typename T>
void pad_matrix(std::vector<std::vector<T>>& matrix) {
  std::size_t i_size = matrix.size();
  std::size_t j_size = matrix[0].size();

  if (i_size > j_size) {
    for (auto& vec : matrix) {
      vec.resize(i_size, std::numeric_limits<T>::max());
    }
  } else if (i_size < j_size) {
    while (matrix.size() < j_size) {
      matrix.push_back(std::vector<T>(j_size, std::numeric_limits<T>::max()));
    }
  }
}

/* Check the feasibility of the matrix element with respect to dual variables */
template <typename T>
bool is_feasible(int row, int col, T val, const std::vector<long double>& row_duals, const std::vector<long double>& col_duals) {
  return fabs(row_duals[row] + col_duals[col] - val) < 0.1;
}

/* Find the lower bound on the bottleneck value */
template <typename T>
void find_bottleneck_lower_bound(T& threshold_lb, const std::vector<std::vector<T>>& matrix) {

  // get the greatest value among minimum of particular rows
  threshold_lb = 0;
  for (auto& row : matrix) {
    threshold_lb = std::max(threshold_lb, *min_element(row.begin(), row.end()));
  }

  // get the greatest value among minimum of particular columns
  for (unsigned c = 0; c < matrix.size(); c++) {
    T local_min = std::numeric_limits<T>::max();
    for (auto& row : matrix) {
      local_min = std::min(local_min, row[c]);
    }
    threshold_lb = std::max(threshold_lb, local_min);
  }
}

/* Get a set of thresholds higher than the lower bound */
template <typename T>
void get_thresholds_set(std::vector<Edge>& threshold_edges, T& threshold_lb, const std::vector<std::vector<T>>& matrix) {

  int sz = matrix.size();
  for (int row = 0; row < sz; row++) {
    for (int col = 0; col < sz; col++) {

      if (matrix[row][col] > threshold_lb) {
        threshold_edges.push_back({row, col});
      }
    }
  }

  sort(threshold_edges.begin(), threshold_edges.end(), [&](Edge i, Edge j) { return matrix[i.row][i.col] > matrix[j.row][j.col]; });
}

/* replace elements equal or greater than threshold with large constant */
template <typename T>
void bound_matrix(T& threshold, std::vector<std::vector<int>>& M_bounded, const std::vector<std::vector<T>>& matrix) {

  int sz = matrix.size();  // square matrix is granted

  for (int r = 0; r < sz; ++r) {
    for (int c = 0; c < sz; ++c) {
      if (matrix[r][c] > threshold) {
        M_bounded[r][c] = 1;
      }
    }
  }
}

/* update duals for a single updated edge */
template <typename T>
void update_duals(const Edge& updated_edge, const std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M, std::vector<long double>& RowDuals,
                  std::vector<long double>& ColDuals, const std::vector<std::vector<int>>& M_bounded) {

  int sz = matrix.size();  // square matrix is granted

  if (matrix[updated_edge.row][updated_edge.col] < RowDuals[updated_edge.row] + ColDuals[updated_edge.col]) {

    long double minval = std::numeric_limits<long double>::max();
    for (int j = 0; j < sz; ++j) {

      if (M_bounded[updated_edge.row][j] == 1) {
        continue;
      }

      if (minval > matrix[updated_edge.row][j] - ColDuals[j]) {
        minval = matrix[updated_edge.row][j] - ColDuals[j];
      }
    }

    RowDuals[updated_edge.row] = minval;

    if (M[updated_edge.row][updated_edge.col] != 1) {

      for (int j = 0; j < sz; ++j) {

        if (M[updated_edge.row][j] == 1) {
          M[updated_edge.row][j] = 0;
        }
      }
    }

  } else {  // remove the edge from the assignment
    M[updated_edge.row][updated_edge.col] = 0;
  }
}

/* replace elements equal or greater than threshold with large constant */
template <typename T>
void bound_matrix_and_update_duals(T& threshold, const std::vector<Edge>& updated_edges, std::vector<std::vector<int>>& M_bounded,
                                   const std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M, std::vector<long double>& RowDuals,
                                   std::vector<long double>& ColDuals) {

  for (auto& edge : updated_edges) {

    if (M_bounded[edge.row][edge.col] == 1 && matrix[edge.row][edge.col] <= threshold) {
      M_bounded[edge.row][edge.col] = 0;
      update_duals(edge, matrix, M, RowDuals, ColDuals, M_bounded);
    }
  }
}


/* return the index of element equal to one in the vector */
int get_index_with_one(std::vector<int> v) {

  int  index = -1;
  auto it    = std::find(v.begin(), v.end(), 1);
  if (it != v.end()) {
    index = std::distance(v.begin(), it);
  }

  return index;
}

/* checking feasibility of solution, unbounded matrix has to be used for check being valid */
template <typename T>
bool is_solution_feasible(std::vector<Edge>& colliding_edges, const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M,
                          const std::vector<std::vector<T>>& start_dists, const std::vector<std::vector<T>>& goal_dists,
                          const std::vector<Eigen::Vector3d>& start_positions, const std::vector<Eigen::Vector3d>& goal_positions, double min_dist,
                          bool use_approximation = false) {

  bool   is_feasible = true;
  bool   done        = false;
  double Mx_sq       = 0.81;
  for (int row_from = 0; row_from < M.size(); row_from++) {

    if (done) {
      break;
    }

    int col_from = get_index_with_one(M[row_from]);

    for (int row_to = row_from + 1; row_to < M.size(); row_to++) {
      int col_to = get_index_with_one(M[row_to]);

      if (use_approximation) {
        T max_l_unused = std::max(matrix[row_from][col_to], matrix[row_to][col_from]);
        T max_l_used   = std::max(matrix[row_from][col_from], matrix[row_to][col_to]);
        T val_used     = matrix[row_from][col_from] + matrix[row_to][col_to];
        T val_unused   = matrix[row_from][col_to] + matrix[row_to][col_from];

        if (max_l_used < Mx_sq * max_l_unused || val_used < val_unused) {
          continue;
        }
      }

      long int A           = start_dists[col_from][col_to];
      long int C           = goal_dists[row_from][row_to];
      double   B           = SCALE_FACTOR * (start_positions[col_from] - start_positions[col_to]).dot(goal_positions[row_from] - goal_positions[row_to]);
      double   denominator = A - 2 * B + C;  // if denominator is zero, the distance between pairs of starts and goals are equal and the vectors are
                                             // parallel, thus the minimum distance equals start or goal distances

      double alpha = (A - B) / denominator;
      if (alpha <= 0 || alpha >= 1) {
        continue;
      }

      double min_dist_exact =
          denominator != 0
              ? SF_INVERTED_SQRT * sqrt((A * C - B * B) / denominator)
              : SF_INVERTED_SQRT *
                    sqrt(std::min(A, C));  // if min(A, C) is zero, it means that some of goals or starts are overlapping, thus the task is not feasible


      if (min_dist_exact >= min_dist) {
        continue;
      }

      is_feasible = false;
      done        = true;

      if (matrix[row_from][col_from] > matrix[row_to][col_to]) {
        colliding_edges.push_back({row_to, col_to});
        colliding_edges.push_back({row_from, col_from});
      } else {
        colliding_edges.push_back({row_from, col_from});
        colliding_edges.push_back({row_to, col_to});
      }
      break;
    }
  }

  return is_feasible;
}

/* For each row of the matrix, find the smallest element and subtract it from every
 * element in its row.
 * For each col of the matrix, find the smallest element and subtract it from every
 * element in its col. Go to Step 2. */
template <typename T>
void step1(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, std::vector<long double>& RowDuals,
           std::vector<long double>& ColDuals, int& step) {

  // process cols - set the initial beta dual variables
  int sz = matrix.size();  // square matrix is granted
  for (int c = 0; c < sz; ++c) {
    T minval = std::numeric_limits<T>::max();
    for (int r = 0; r < sz; ++r) {
      minval = std::min(minval, matrix[r][c]);
    }

    ColDuals[c] = minval;
  }

  for (int r = 0; r < sz; ++r) {
    T    minval     = std::numeric_limits<T>::max();
    bool minval_set = false;
    for (int c = 0; c < sz; ++c) {
      if (M_bounded[r][c] == 0) {
        if (minval > matrix[r][c] - ColDuals[c]) {
          minval     = matrix[r][c] - ColDuals[c];
          minval_set = true;
        }
      }
    }

    RowDuals[r] = minval_set ? minval : 0.0;
  }

  step = 2;
}

/* helper to clear the temporary vectors */
inline void clear_covers(std::vector<int>& cover) {
  std::fill(cover.begin(), cover.end(), 0);
}

/* Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or
 * column, star Z. Repeat for each element in the matrix. Go to Step 3.  In this step,
 * we introduce the mask matrix M, which in the same dimensions as the cost matrix and
 * is used to star and prime zeros of the cost matrix.  If M(i,j)=1 then C(i,j) is a
 * starred zero,  If M(i,j)=2 then C(i,j) is a primed zero.  We also define two vectors
 * RowCover and ColCover that are used to "cover" the rows and columns of the cost matrix.
 * In the nested loop (over indices i and j) we check to see if C(i,j) is a zero value
 * and if its column or row is not already covered.  If not then we star this zero
 * (i.e. set M(i,j)=1) and cover its row and column (i.e. set R_cov(i)=1 and C_cov(j)=1).
 * Before we go on to Step 3, we uncover all rows and columns so that we can use the
 * cover vectors to help us count the number of starred zeros. */
template <typename T>
void step2(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, std::vector<std::vector<int>>& M,
           std::vector<int>& RowCover, std::vector<int>& ColCover, std::vector<long double>& RowDuals, std::vector<long double>& ColDuals, int& step) {

  int sz = matrix.size();

  for (int r = 0; r < sz; ++r) {

    for (int c = 0; c < sz; ++c) {

      if (M_bounded[r][c] == 0 && is_feasible(r, c, matrix[r][c], RowDuals, ColDuals)) {

        if (RowCover[r] == 0 && ColCover[c] == 0) {
          M[r][c]     = 1;
          RowCover[r] = 1;
          ColCover[c] = 1;
        }
      }
    }
  }

  clear_covers(RowCover);  // reset vectors for posterior using

  step = 3;
}

/* Cover each column containing a starred zero.  If K columns are covered, the starred
 * zeros describe a complete set of unique assignments.  In this case, Go to DONE,
 * otherwise, Go to Step 4. Once we have searched the entire cost matrix, we count the
 * number of independent zeros found.  If we have found (and starred) K independent zeros
 * then we are done.  If not we procede to Step 4.*/
void step3(std::vector<std::vector<int>>& M, std::vector<int>& ColCover, int& step, int& cardinality) {

  int sz      = M.size();
  cardinality = 0;

  for (int r = 0; r < sz; ++r)
    for (int c = 0; c < sz; ++c)
      if (M[r][c] == 1) {
        ColCover[c] = 1;
        cardinality++;
      } else if (M[r][c] == 2) {
        M[r][c] = 0;
      }

  if (cardinality >= sz) {
    step = 7;  // solution found
  } else {
    step = 4;
  }
}

// Following functions to support step 4
template <typename T>
void find_a_zero(int& row, int& col, const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded,
                 const std::vector<int>& RowCover, const std::vector<int>& ColCover, const std::vector<long double>& RowDuals,
                 const std::vector<long double>& ColDuals) {

  int sz = matrix.size();
  row    = -1;
  col    = -1;

  for (int c = 0; c < sz; c++) {

    if (ColCover[c] == 1) {
      continue;
    }

    for (int r = 0; r < sz; r++) {
      if (RowCover[r] == 0 && M_bounded[r][c] == 0 && is_feasible(r, c, matrix[r][c], RowDuals, ColDuals)) {
        row = r;
        col = c;
        return;
      }
    }
  }
}

bool star_in_row(int row, const std::vector<std::vector<int>>& M) {

  bool tmp = false;
  for (unsigned c = 0; c < M.size(); c++) {
    if (M[row][c] == 1) {
      tmp = true;
      break;
    }
  }

  return tmp;
}


bool find_star_in_row(int row, int& col, const std::vector<std::vector<int>>& M) {

  int sz = M.size();
  for (int c = 0; c < sz; c++) {
    if (M[row][c] == 1) {
      col = c;
      return true;
    }
  }

  return false;
}


/* Find a noncovered zero and prime it.  If there is no starred zero in the row containing
 * this primed zero, Go to Step 5.  Otherwise, cover this row and uncover the column
 * containing the starred zero. Continue in this manner until there are no uncovered zeros
 * left. Save the smallest uncovered value and Go to Step 6. */
template <typename T>
void step4(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, std::vector<std::vector<int>>& M,
           std::vector<int>& RowCover, std::vector<int>& ColCover, std::vector<long double>& RowDuals, std::vector<long double>& ColDuals, int& path_row_0,
           int& path_col_0, int& step) {

  int  row  = -1;
  int  col  = -1;
  bool done = false;

  while (!done) {

    find_a_zero(row, col, matrix, M_bounded, RowCover, ColCover, RowDuals, ColDuals);

    if (row == -1) {
      done = true;
      step = 6;
    } else {
      M[row][col] = 2;
      if (find_star_in_row(row, col, M)) {
        RowCover[row] = 1;
        ColCover[col] = 0;
      } else {
        done       = true;
        step       = 5;
        path_row_0 = row;
        path_col_0 = col;
      }
    }
  }
}

// Following functions to support step 5
void find_star_in_col(int c, int& r, const std::vector<std::vector<int>>& M) {

  r = -1;
  for (int i = 0; i < M.size(); i++) {
    if (M[i][c] == 1) {
      r = i;
      break;
    }
  }
}

void find_prime_in_row(int r, int& c, const std::vector<std::vector<int>>& M) {

  for (int j = 0; j < M.size(); j++) {
    if (M[r][j] == 2) {
      c = j;
      break;
    }
  }
}

/* void augment_path(std::vector<std::vector<int>>& path, int path_count, std::vector<std::vector<int>>& M, std::vector<int>& ColCover) { */
void augment_path(std::vector<std::vector<int>>& path, int path_count, std::vector<std::vector<int>>& M) {

  for (int p = 0; p < path_count; p++) {
    if (M[path[p][0]][path[p][1]] == 1) {
      M[path[p][0]][path[p][1]] = 0;
    } else {
      M[path[p][0]][path[p][1]] = 1;
    }
  }
}

void erase_primes(std::vector<std::vector<int>>& M) {

  for (auto& row : M) {
    for (auto& val : row) {

      if (val == 2) {
        val = 0;
      }
    }
  }
}


/* Construct a series of alternating primed and starred zeros as follows.
 * Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote the
 * starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the
 * row of Z1 (there will always be one).  Continue until the series terminates at a
 * primed zero that has no starred zero in its column.  Unstar each starred zero of
 * the series, star each primed zero of the series, erase all primes and uncover every
 * line in the matrix.  Return to Step 3.  You may notice that Step 5 seems vaguely
 * familiar.  It is a verbal description of the augmenting path algorithm (for solving
 * the maximal matching problem). */
void step5(std::vector<std::vector<int>>& path, int path_row_0, int path_col_0, std::vector<std::vector<int>>& M, std::vector<int>& RowCover,
           std::vector<int>& ColCover, int& step) {

  int r          = -1;
  int c          = -1;
  int path_count = 1;

  path[path_count - 1][0] = path_row_0;
  path[path_count - 1][1] = path_col_0;

  bool done = false;

  while (!done) {

    find_star_in_col(path[path_count - 1][1], r, M);
    if (r > -1) {

      path_count += 1;
      path[path_count - 1][0] = r;
      path[path_count - 1][1] = path[path_count - 2][1];
    } else {
      done = true;
    }

    if (!done) {
      find_prime_in_row(path[path_count - 1][0], c, M);
      path_count += 1;
      path[path_count - 1][0] = path[path_count - 2][0];
      path[path_count - 1][1] = c;
    }
  }

  augment_path(path, path_count, M);
  clear_covers(RowCover);
  clear_covers(ColCover);

  step = 3;
}

template <typename T>
int n_feasible_elements(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, const std::vector<long double>& RowDuals,
                        const std::vector<long double>& ColDuals) {

  int result = 0;
  for (unsigned r = 0; r < matrix.size(); r++) {
    for (unsigned c = 0; c < matrix.size(); c++) {

      if (M_bounded[r][c] == 0 && fabs(matrix[r][c] - RowDuals[r] - ColDuals[c]) < 0.1) {
        result++;
      }
    }
  }

  return result;
}

// methods to support step 6
template <typename T>
bool find_smallest(long double& minval, const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded,
                   const std::vector<int>& RowCover, const std::vector<int>& ColCover, const std::vector<long double>& RowDuals,
                   const std::vector<long double>& ColDuals) {

  bool   minval_found = false;
  size_t sz           = matrix.size();
  for (unsigned r = 0; r < sz; r++) {

    if (RowCover[r] == 0) {

      for (unsigned c = 0; c < sz; c++) {

        if (ColCover[c] == 0 && M_bounded[r][c] == 0) {
          long double val = static_cast<long double>(matrix[r][c]) - RowDuals[r] - ColDuals[c];
          if (minval > val) {
            minval       = val;
            minval_found = true;
          }
        }
      }
    }
  }

  return minval_found;
}

/* Add the value found in Step 4 to every element of each covered row, and subtract it
 * from every element of each uncovered column. Return to Step 4 without altering any
 * stars, primes, or covered lines. Notice that this step uses the smallest uncovered
 * value in the cost matrix to modify the matrix. Even though this step refers to the
 * value being found in Step 4 it is more convenient to wait until you reach Step 6
 * before searching for this value. It may seem that since the values in the cost
 * matrix are being altered, we would lose sight of the original problem.
 * However, we are only changing certain values that have already been tested and
 * found not to be elements of the minimal assignment. Also we are only changing the
 * values by an amount equal to the smallest value in the cost matrix, so we will not
 * jump over the optimal (i.e. minimal assignment) with this change. */
template <typename T>
bool step6(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, const std::vector<int>& RowCover,
           const std::vector<int>& ColCover, std::vector<long double>& RowDuals, std::vector<long double>& ColDuals, int& step) {

  long double minval             = std::numeric_limits<long double>::max();
  bool        smallest_val_found = find_smallest(minval, matrix, M_bounded, RowCover, ColCover, RowDuals, ColDuals);

  if (!smallest_val_found) {  // minval is exceeded if all remaining entries in the matrix are bounded -> go to step 7 to increase the threshold
    step = 7;
    return true;
  }

  long double minval_f = minval / 2.0;  // match value of theta from dynamic hungarian algorithm, convert to double precision

  int sz = matrix.size();
  for (int r = 0; r < sz; r++) {
    if (RowCover[r] == 1) {
      RowDuals[r] -= minval_f;
    } else {
      RowDuals[r] += minval_f;
    }
  }

  for (int c = 0; c < sz; c++) {
    if (ColCover[c] == 0) {
      ColDuals[c] += minval_f;
    } else {
      ColDuals[c] -= minval_f;
    }
  }

  step = 4;
  return false;
}

/* Calculates the optimal cost from mask matrix */
template <template <typename, typename...> class Container, typename T, typename... Args>
std::tuple<T, std::vector<std::vector<int>>, double> output_solution(const Container<Container<T, Args...>>& original, const std::vector<std::vector<int>>& M,
                                                                     const double solution_time) {
  T res = 0;

  for (unsigned j = 0; j < original.begin()->size(); ++j) {
    for (unsigned i = 0; i < original.size(); ++i) {

      if (M[i][j]) {
        auto it1 = original.begin();
        std::advance(it1, i);
        auto it2 = it1->begin();
        std::advance(it2, j);
        res += *it2;
        continue;
      }
    }
  }

  return std::make_tuple(res, M, solution_time);
}


/* internal hungarian function used for finding solution inside branching part */
template <typename T>
bool internal_hungarian(const std::vector<std::vector<T>>& matrix, const std::vector<std::vector<int>>& M_bounded, std::vector<std::vector<int>>& M,
                        std::vector<long double>& RowDuals, std::vector<long double>& ColDuals) {

  int sz = matrix.size();

  std::vector<int> RowCover(sz, 0);
  std::vector<int> ColCover(sz, 0);

  // initialize ColCoverMatching
  int path_row_0, path_col_0;  // temporary to hold the smallest uncovered value

  // Array for the augmenting path algorithm
  std::vector<std::vector<int>> path(2 * sz, std::vector<int>(2, 0));

  /* Now Work The Steps */
  bool done               = false;
  bool threshold_exceeded = false;
  int  step               = 3;
  int  cardinality        = 0;

  while (!done) {
    auto                          start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff;
    switch (step) {
      case 3:
        step3(M, ColCover, step, cardinality);
        break;
      case 4:
        step4(matrix, M_bounded, M, RowCover, ColCover, RowDuals, ColDuals, path_row_0, path_col_0, step);
        break;
      case 5:
        step5(path, path_row_0, path_col_0, M, RowCover, ColCover, step);
        break;
      case 6:
        threshold_exceeded = step6(matrix, M_bounded, RowCover, ColCover, RowDuals, ColDuals, step);
        break;
      default:
        done = true;
        break;
    }
  }

  if (threshold_exceeded) {
    return false;
  }

  return true;
}

/* explores the solutions by expanding childs with different set of restricted nodes */
void expand_childs(std::vector<Node>& open_list, Node& parent, const std::vector<Edge> colliding_edges, const std::vector<Edge> solution_edges) {

  std::vector<long double> r_duals(parent.row_duals);
  std::vector<long double> c_duals(parent.col_duals);
  std::vector<Edge>        initial_solution(solution_edges);

  for (auto& edge : colliding_edges) {
    Node child;
    child.restricted_edge  = edge;
    child.initial_solution = initial_solution;
    child.row_duals        = r_duals;
    child.col_duals        = c_duals;
    child.depth            = parent.depth + 1;
    open_list.push_back(child);
  }
}

/* extract solution from matrix */
std::vector<Edge> extract_solution(const std::vector<std::vector<int>>& M) {
  std::vector<Edge> solution;
  int               sz = M.size();
  for (int r = 0; r < sz; r++) {
    for (int c = 0; c < sz; c++) {

      if (M[r][c] == 1) {
        solution.push_back({r, c});
        break;
      }
    }
  }

  return solution;
}

// updates the original matrix by restricted edges
template <typename T>
void update_restricted_edges_and_duals(std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M_bounded, std::vector<std::vector<int>>& M,
                                       std::vector<long double>& RowDuals, std::vector<long double>& ColDuals, std::list<RestrictedNode<T>>& restricted_nodes,
                                       const Node& node) {

  if (!restricted_nodes.empty()) {

    RestrictedNode<T> current = restricted_nodes.back();
    while (!restricted_nodes.empty() && current.depth >= node.depth) {
      M_bounded[current.restricted_edge.row][current.restricted_edge.col] = 0;
      restricted_nodes.pop_back();
      current = restricted_nodes.back();
    }
  }

  RestrictedNode<T> rn                                      = {node.restricted_edge, node.depth};
  M_bounded[rn.restricted_edge.row][rn.restricted_edge.col] = 1;
  restricted_nodes.push_back(rn);

  update_duals(rn.restricted_edge, matrix, M, RowDuals, ColDuals, M_bounded);
}


template <typename T>
void restore_original_matrix(std::vector<std::vector<int>>& M_bounded, std::list<RestrictedNode<T>>& restricted_nodes) {

  if (restricted_nodes.empty()) {
    return;
  }

  RestrictedNode<T> current = restricted_nodes.back();
  while (current.depth > 0) {
    M_bounded[current.restricted_edge.row][current.restricted_edge.col] = 0;
    restricted_nodes.pop_back();

    if (!restricted_nodes.empty()) {
      current = restricted_nodes.back();
    } else {
      break;
    }
  }
}

/* explores the solutions by DFS with branching */
template <typename T>
bool branch_solution(std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M_bounded, std::vector<std::vector<int>>& M,
                     const std::vector<std::vector<T>>& start_dists, const std::vector<std::vector<T>>& goal_dists, const std::vector<long double>& RowDuals,
                     const std::vector<long double>& ColDuals, const std::vector<Edge>& colliding_edges, const std::vector<Eigen::Vector3d>& start_positions,
                     const std::vector<Eigen::Vector3d>& goal_positions, double min_dist) {

  int               sz = matrix.size();
  std::vector<Node> open_list;
  Edge              restr_edge;
  std::vector<Edge> solution_edges = extract_solution(M);

  Node root = {solution_edges, restr_edge, RowDuals, ColDuals, 0};
  expand_childs(open_list, root, colliding_edges, solution_edges);

  std::list<RestrictedNode<T>> restricted_nodes;

  while (!open_list.empty()) {
    Node current = open_list.back();
    open_list.pop_back();

    // initialize local masked matrix
    std::vector<std::vector<int>> Ml(sz, std::vector<int>(sz, 0));

    for (auto& edge : current.initial_solution) {
      Ml[edge.row][edge.col] = 1;
    }

    // update matrix with restricted edges
    update_restricted_edges_and_duals(matrix, M_bounded, Ml, current.row_duals, current.col_duals, restricted_nodes, current);

    // run hungarian
    if (!internal_hungarian(matrix, M_bounded, Ml, current.row_duals, current.col_duals)) {
      // solution not found with current constraints on unfeasible edges
      continue;
    }

    // get colliding edges
    std::vector<Edge> coll_edges;
    if (!is_solution_feasible(coll_edges, matrix, Ml, start_dists, goal_dists, start_positions, goal_positions,
                              min_dist)) {  // collision found -> start branching

      expand_childs(open_list, current, coll_edges, extract_solution(Ml));

    } else {
      M = Ml;  // propagate found solution to main loop
      restore_original_matrix(M_bounded, restricted_nodes);
      return true;
    }
  }

  restore_original_matrix(M_bounded, restricted_nodes);

  // solution not found
  return false;
}

/* Main function of the dynamic hungarian algorithm */
template <template <typename, typename...> class Container, typename T, typename... Args>
typename std::enable_if<std::is_integral<T>::value, std::tuple<T, std::vector<std::vector<int>>, double>>::type  // Work only on integral types
catora_assignment(const Container<Container<T, Args...>>& original, const Container<Container<T, Args...>>& start_dists,
                  const Container<Container<T, Args...>>& goal_dists, const std::vector<Eigen::Vector3d>& start_positions,
                  const std::vector<Eigen::Vector3d>& goal_positions, double min_dist, bool allow_negatives = true) {

  // NOTE: start_dists and end_dists are supposed to be squared, otherwise, the collision check won't work
  /* std::cout << "Starting CAT-ORA assignment algorithm:" << std::endl; */
  auto start_time = std::chrono::high_resolution_clock::now();

  // Work on a vector copy to preserve original matrix
  // Didn't passed by value cause needed to access both
  std::vector<std::vector<T>> matrix(original.size(), std::vector<T>(original.begin()->size()));

  T greatest = 0;
  for (auto& row : original) {
    greatest = std::max(greatest, *std::max_element(begin(row), end(row)));
  }

  auto it = original.begin();
  for (auto& vec : matrix) {
    std::copy(it->begin(), it->end(), vec.begin());
    it = std::next(it);
  }

  // handle negative values -> pass true if allowed or false otherwise
  // if it is an unsigned type just skip this step
  if (!std::is_unsigned<T>::value) {
    handle_negatives(matrix, allow_negatives);
  }

  // make square matrix
  pad_matrix(matrix);
  std::size_t sz = matrix.size();


  /* The masked matrix M.  If M(i,j)=1 then C(i,j) is a starred zero,
   * If M(i,j)=2 then C(i,j) is a primed zero. */
  std::vector<std::vector<int>> M(sz, std::vector<int>(sz, 0));

  // The masked matrix used as indication of bounded elements in original matrix
  std::vector<std::vector<int>> M_bounded(M);

  /* We also define two vectors RowCover and ColCover that are used to "cover"
   *the rows and columns of the cost matrix C*/
  std::vector<int> RowCover(sz, 0);
  std::vector<int> ColCover(sz, 0);
  std::vector<int> ColCoverMatching(sz, 0);

  /* We also define two vectors RowDuals and ColDuals that are used to decide
   * on feasibility of particular elements of matrix */
  std::vector<long double> RowDuals(sz, 0.0);
  std::vector<long double> ColDuals(sz, 0.0);

  int path_row_0, path_col_0;  // temporary to hold the smallest uncovered value

  // Array for the augmenting path algorithm
  std::vector<std::vector<int>> path(2 * sz, std::vector<int>(2, 0));

  // get initial lower bound and vector of thresholds
  T threshold;
  find_bottleneck_lower_bound(threshold, matrix);
  std::vector<Edge> threshold_edges;
  get_thresholds_set(threshold_edges, threshold, matrix);  // returns vector of potential thresholds in descending order

  bound_matrix(threshold, M_bounded, matrix);  // sets large constant to entries greater than threshold

  bool done             = false;
  bool update_threshold = false;
  int  step             = 1;
  int  cardinality      = 0;

  while (!done) {
    std::chrono::duration<double> diff;
    switch (step) {
      case 1:
        step1(matrix, M_bounded, RowDuals, ColDuals, step);
        break;
      case 2:
        step2(matrix, M_bounded, M, RowCover, ColCoverMatching, RowDuals, ColDuals, step);
        break;
      case 3:
        step3(M, ColCover, step, cardinality);
        break;
      case 4:
        step4(matrix, M_bounded, M, RowCover, ColCover, RowDuals, ColDuals, path_row_0, path_col_0, step);
        break;
      case 5:
        step5(path, path_row_0, path_col_0, M, RowCover, ColCover, step);
        break;
      case 6:
        update_threshold = step6(matrix, M_bounded, RowCover, ColCover, RowDuals, ColDuals, step);
        break;
      case 7: {
        auto start = std::chrono::high_resolution_clock::now();
        // if there is no feasible assignment at all then increase the threshold, if the collision is found start the branching and try to found another
        // solution with the same threshold, if it does not exist then increase the threshold NOTE: the solution have to be always feasible. When all entries of
        // the matrix are below the threshold, the solution reduces to LSAP problem with a complete matrix which guarantees the solution with no collisions

        if (update_threshold) {
          // increase the bound
          if (!threshold_edges.empty()) {

            std::vector<Edge> edges_over_threshold;
            if (threshold_edges.size() > (sz - cardinality)) {

              edges_over_threshold.insert(edges_over_threshold.end(), threshold_edges.begin() + threshold_edges.size() - (sz - cardinality - 1),
                                          threshold_edges.end());
              threshold_edges.erase(threshold_edges.begin() + threshold_edges.size() - (sz - cardinality - 1), threshold_edges.end());
              threshold = matrix[threshold_edges.back().row][threshold_edges.back().col];

              while (!threshold_edges.empty() && matrix[threshold_edges.back().row][threshold_edges.back().col] == threshold) {
                edges_over_threshold.push_back(threshold_edges.back());
                threshold_edges.pop_back();
              }

            } else {

              threshold = matrix[threshold_edges.front().row][threshold_edges.front().col];
              edges_over_threshold.insert(edges_over_threshold.end(), threshold_edges.begin(), threshold_edges.end());
              threshold_edges.clear();
            }
            //
            // update matrix with new threshold
            clear_covers(RowCover);
            clear_covers(ColCover);

            bound_matrix_and_update_duals(threshold, edges_over_threshold, M_bounded, matrix, M, RowDuals, ColDuals);

            step = 3;

            update_threshold = false;
            break;
          } else {
            std::cout << "ERROR: No feasible solution found on original matrix. This should never happen." << std::endl;
          }

        } else {

          std::vector<Edge> colliding_edges;
          bool              result = true;
          if (!is_solution_feasible(colliding_edges, matrix, M, start_dists, goal_dists, start_positions, goal_positions,
                                    min_dist)) {  // collision found -> start branching
            // branch the solution
            result = branch_solution(matrix, M_bounded, M, start_dists, goal_dists, RowDuals, ColDuals, colliding_edges, start_positions, goal_positions,
                                     min_dist);  // bounded matrix has to be used
          }

          if (!result) {
            std::vector<Edge> edges_over_threshold;
            if (!threshold_edges.empty()) {

              threshold = matrix[threshold_edges.back().row][threshold_edges.back().col];
              while (!threshold_edges.empty() && matrix[threshold_edges.back().row][threshold_edges.back().col] == threshold) {
                edges_over_threshold.push_back(threshold_edges.back());
                threshold_edges.pop_back();
              }

              clear_covers(RowCover);
              clear_covers(ColCover);
              bound_matrix_and_update_duals(threshold, edges_over_threshold, M_bounded, matrix, M, RowDuals, ColDuals);

              step = 3;

              break;
            } else {
              std::cout << "ERROR: No feasible solution found on original matrix. This should never happen." << std::endl;
            }
          }
        }

        for (auto& vec : M) {
          vec.resize(original.begin()->size());
        }

        M.resize(original.size());
        done = true;
      } break;
      default:
        done = true;
        break;
    }
  }

  auto                          end_time         = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_end         = end_time - start_time;
  double                        computation_time = diff_end.count() * 1000.0;  // time in ms

  /* std::cout << "CAT-ORA assignment: computation time = " << computation_time << "ms\n"; */

  return output_solution(original, M, computation_time);
}

//}

/* ORIGINAL HUNGARIAN ALGORITHM //{ */

// | --------------- ORIGINAL HUNGARIAN SECTION --------------- |

/* step1lsap() //{ */

/* For each row of the matrix, find the smallest element and subtract it from every
 * element in its row.
 * For each col of the matrix, find the smallest element and subtract it from every
 * element in its col. Go to Step 2. */
template <typename T>
void step1lsap(std::vector<std::vector<T>>& matrix, int& step) {

  // process rows
  for (auto& row : matrix) {
    auto smallest = *std::min_element(begin(row), end(row));
    if (smallest > 0) {
      for (auto& n : row) {
        n -= smallest;
      }
    }
  }

  // process cols
  int sz = matrix.size();  // square matrix is granted
  for (int j = 0; j < sz; ++j) {
    T minval = std::numeric_limits<T>::max();
    for (int i = 0; i < sz; ++i) {
      minval = std::min(minval, matrix[i][j]);
    }

    if (minval > 0) {
      for (int i = 0; i < sz; ++i) {
        matrix[i][j] -= minval;
      }
    }
  }

  step = 2;
}

//}

/* step2lsap() //{ */

/* Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or
 * column, star Z. Repeat for each element in the matrix. Go to Step 3.  In this step,
 * we introduce the mask matrix M, which in the same dimensions as the cost matrix and
 * is used to star and prime zeros of the cost matrix.  If M(i,j)=1 then C(i,j) is a
 * starred zero,  If M(i,j)=2 then C(i,j) is a primed zero.  We also define two vectors
 * RowCover and ColCover that are used to "cover" the rows and columns of the cost matrix.
 * In the nested loop (over indices i and j) we check to see if C(i,j) is a zero value
 * and if its column or row is not already covered.  If not then we star this zero
 * (i.e. set M(i,j)=1) and cover its row and column (i.e. set R_cov(i)=1 and C_cov(j)=1).
 * Before we go on to Step 3, we uncover all rows and columns so that we can use the
 * cover vectors to help us count the number of starred zeros. */
template <typename T>
void step2lsap(const std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M, std::vector<int>& RowCover, std::vector<int>& ColCover, int& step) {

  int sz = matrix.size();

  for (int r = 0; r < sz; ++r) {
    for (int c = 0; c < sz; ++c) {
      if (matrix[r][c] == 0) {
        if (RowCover[r] == 0 && ColCover[c] == 0) {
          M[r][c]     = 1;
          RowCover[r] = 1;
          ColCover[c] = 1;
        }
      }
    }
  }

  clear_covers(RowCover);  // reset vectors for posterior using
  clear_covers(ColCover);

  step = 3;
}


//}

/* step3lsap() //{ */

/* Cover each column containing a starred zero.  If K columns are covered, the starred
 * zeros describe a complete set of unique assignments.  In this case, Go to DONE,
 * otherwise, Go to Step 4. Once we have searched the entire cost matrix, we count the
 * number of independent zeros found.  If we have found (and starred) K independent zeros
 * then we are done.  If not we procede to Step 4.*/
void step3lsap(std::vector<std::vector<int>>& M, std::vector<int>& ColCover, int& step) {

  int sz       = M.size();
  int colcount = 0;

  for (int r = 0; r < sz; ++r) {
    for (int c = 0; c < sz; ++c) {
      if (M[r][c] == 1) {
        ColCover[c] = 1;
      } else if (M[r][c] == 2) {
        M[r][c] = 0;
      }
    }
  }

  for (auto& n : ColCover) {
    if (n == 1) {
      colcount++;
    }
  }

  if (colcount >= sz) {
    step = 7;  // solution found
  } else {
    step = 4;
  }
}

//}

/* find_a_zero_lsap() //{ */

// Following functions to support step 4
template <typename T>
void find_a_zero_lsap(int& row, int& col, const std::vector<std::vector<T>>& matrix, const std::vector<int>& RowCover, const std::vector<int>& ColCover) {

  int sz = matrix.size();

  row = -1;
  col = -1;

  for (int c = 0; c < sz; c++) {
    if (ColCover[c] == 1) {
      continue;
    }

    for (int r = 0; r < sz; r++) {
      if (RowCover[r] == 0 && matrix[r][c] == 0) {
        row = r;
        col = c;
        return;
      }
    }
  }
}

//}

/* step4lsap() //{ */

/* Find a noncovered zero and prime it.  If there is no starred zero in the row containing
 * this primed zero, Go to Step 5.  Otherwise, cover this row and uncover the column
 * containing the starred zero. Continue in this manner until there are no uncovered zeros
 * left. Save the smallest uncovered value and Go to Step 6. */
template <typename T>
void step4lsap(const std::vector<std::vector<T>>& matrix, std::vector<std::vector<int>>& M, std::vector<int>& RowCover, std::vector<int>& ColCover,
               int& path_row_0, int& path_col_0, int& step) {

  int  row  = -1;
  int  col  = -1;
  bool done = false;

  while (!done) {
    find_a_zero_lsap(row, col, matrix, RowCover, ColCover);

    if (row == -1) {
      done = true;
      step = 6;
    } else {
      M[row][col] = 2;
      if (find_star_in_row(row, col, M)) {
        RowCover[row] = 1;
        ColCover[col] = 0;
      } else {
        done       = true;
        step       = 5;
        path_row_0 = row;
        path_col_0 = col;
      }
    }
  }
}

//}

/* step5lsap //{ */

/* Construct a series of alternating primed and starred zeros as follows.
 * Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote the
 * starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the
 * row of Z1 (there will always be one).  Continue until the series terminates at a
 * primed zero that has no starred zero in its column.  Unstar each starred zero of
 * the series, star each primed zero of the series, erase all primes and uncover every
 * line in the matrix.  Return to Step 3.  You may notice that Step 5 seems vaguely
 * familiar.  It is a verbal description of the augmenting path algorithm (for solving
 * the maximal matching problem). */
void step5lsap(std::vector<std::vector<int>>& path, int path_row_0, int path_col_0, std::vector<std::vector<int>>& M, std::vector<int>& RowCover,
               std::vector<int>& ColCover, int& step) {

  int r          = -1;
  int c          = -1;
  int path_count = 1;

  path[path_count - 1][0] = path_row_0;
  path[path_count - 1][1] = path_col_0;

  bool done = false;

  while (!done) {
    find_star_in_col(path[path_count - 1][1], r, M);
    if (r > -1) {
      path_count += 1;
      path[path_count - 1][0] = r;
      path[path_count - 1][1] = path[path_count - 2][1];
    } else {
      done = true;
    }

    if (!done) {
      find_prime_in_row(path[path_count - 1][0], c, M);
      path_count += 1;
      path[path_count - 1][0] = path[path_count - 2][0];
      path[path_count - 1][1] = c;
    }
  }

  augment_path(path, path_count, M);
  clear_covers(RowCover);
  clear_covers(ColCover);

  step = 3;
}
//}

/* find_smallest_lsap //{ */

// methods to support step 6
template <typename T>
void find_smallest_lsap(T& minval, const std::vector<std::vector<T>>& matrix, const std::vector<int>& RowCover, const std::vector<int>& ColCover) {

  for (unsigned r = 0; r < matrix.size(); r++) {
    if (RowCover[r] == 0) {
      for (unsigned c = 0; c < matrix.size(); c++) {
        if (ColCover[c] == 0) {
          if (minval > matrix[r][c]) {
            minval = matrix[r][c];
          }
        }
      }
    }
  }
}

//}

/* step6lsap() //{ */

/* Add the value found in Step 4 to every element of each covered row, and subtract it
 * from every element of each uncovered column.  Return to Step 4 without altering any
 * stars, primes, or covered lines. Notice that this step uses the smallest uncovered
 * value in the cost matrix to modify the matrix.  Even though this step refers to the
 * value being found in Step 4 it is more convenient to wait until you reach Step 6
 * before searching for this value.  It may seem that since the values in the cost
 * matrix are being altered, we would lose sight of the lsapinal problem.
 * However, we are only changing certain values that have already been tested and
 * found not to be elements of the minimal assignment.  Also we are only changing the
 * values by an amount equal to the smallest value in the cost matrix, so we will not
 * jump over the optimal (i.e. minimal assignment) with this change. */
template <typename T>
void step6lsap(std::vector<std::vector<T>>& matrix, const std::vector<int>& RowCover, const std::vector<int>& ColCover, int& step) {

  T minval = std::numeric_limits<T>::max();
  find_smallest_lsap(minval, matrix, RowCover, ColCover);

  int sz = matrix.size();
  for (int r = 0; r < sz; r++) {
    for (int c = 0; c < sz; c++) {
      if (RowCover[r] == 1) {
        matrix[r][c] += minval;
      }
      if (ColCover[c] == 0) {
        matrix[r][c] -= minval;
      }
    }
  }

  step = 4;
}
//}

/* Main function of the original Hungarian algorithm //{ */
template <template <typename, typename...> class Container, typename T, typename... Args>
typename std::enable_if<std::is_integral<T>::value, std::tuple<T, std::vector<std::vector<int>>, double>>::type hungarian(
    const Container<Container<T, Args...>>& original, bool allow_negatives = true) {

  /* std::cout << "Starting hungarian algorithm:" << std::endl; */
  auto start_time = std::chrono::high_resolution_clock::now();

  // Work on a vector copy to preserve original matrix
  // Didn't passed by value cause needed to access both
  std::vector<std::vector<T>> matrix(original.size(), std::vector<T>(original.begin()->size()));

  auto it = original.begin();
  for (auto& vec : matrix) {
    std::copy(it->begin(), it->end(), vec.begin());
    it = std::next(it);
  }

  // handle negative values -> pass true if allowed or false otherwise
  // if it is an unsigned type just skip this step
  if (!std::is_unsigned<T>::value) {
    handle_negatives(matrix, allow_negatives);
  }

  // make square matrix
  pad_matrix(matrix);
  std::size_t sz = matrix.size();

  /* The masked matrix M.  If M(i,j)=1 then C(i,j) is a starred zero,
   * If M(i,j)=2 then C(i,j) is a primed zero. */
  std::vector<std::vector<int>> M(sz, std::vector<int>(sz, 0));

  /* We also define two vectors RowCover and ColCover that are used to "cover"
   * the rows and columns of the cost matrix C*/
  std::vector<int> RowCover(sz, 0);
  std::vector<int> ColCover(sz, 0);

  int path_row_0, path_col_0;  // temporary to hold the smallest uncovered value

  // Array for the augmenting path algorithm
  std::vector<std::vector<int>> path(2 * sz, std::vector<int>(2, 0));

  bool done = false;
  int  step = 1;

  while (!done) {
    switch (step) {
      case 1:
        step1lsap(matrix, step);
        break;
      case 2:
        step2lsap(matrix, M, RowCover, ColCover, step);
        break;
      case 3:
        step3lsap(M, ColCover, step);
        break;
      case 4:
        step4lsap(matrix, M, RowCover, ColCover, path_row_0, path_col_0, step);
        break;
      case 5:
        step5lsap(path, path_row_0, path_col_0, M, RowCover, ColCover, step);
        break;
      case 6:
        step6lsap(matrix, RowCover, ColCover, step);
        break;
      case 7:
        for (auto& vec : M) {
          vec.resize(original.begin()->size());
        }
        M.resize(original.size());
        done = true;
        break;
      default:
        done = true;
        break;
    }
  }

  auto                          end_time           = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_comp_time     = end_time - start_time;
  auto                          computational_time = diff_comp_time.count() * 1000.0;  // time in ms

  /* std::cout << "Original hungarian: computational time = " << computational_time << " ms\n"; */

  return output_solution(original, M, computational_time);
}

//}

//}

}  // namespace Catora
