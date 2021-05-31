//
// Created by usl on 12/10/20.
//

#ifndef LINCALIB_EIGEN_UTILS_H
#define LINCALIB_EIGEN_UTILS_H

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <math.h>

namespace Eigen {

    template <typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    template <typename T>
    using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

    template <typename K, typename V>
    using aligned_map = std::map<K, V, std::less<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;

    template <typename K, typename V>
    using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;

/** sorts vectors from large to small
   * vec: vector to be sorted
   * sorted_vec: sorted results
   * ind: the position of each element in the sort result in the original vector
   * https://www.programmersought.com/article/343692646/
 */
    inline void sort_vec(const Eigen::Vector3d& vec,
                         Eigen::Vector3d& sorted_vec,
                         Eigen::Vector3i& ind) {
        ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size()-1);//[0 1 2]
        auto rule=[vec](int i, int j)->bool{
            return vec(i)>vec(j);
        };  // regular expression, as a predicate of sort

        std::sort(ind.data(), ind.data()+ind.size(), rule);

        // The data member function returns a pointer to the first element of VectorXd,
        // similar to begin()
        for (int i=0;i<vec.size();i++) {
            sorted_vec(i) = vec(ind(i));
        }
    }
}  // namespace Eigen

#endif //LINCALIB_EIGEN_UTILS_H
