/*===========================================================================*\
 *                                                                           *
 *                              OpenFlipper                                  *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openflipper.org                            *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenFlipper.                                         *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
\*===========================================================================*/

#include <queue>

namespace ACG {
namespace Algorithm {

namespace _DBSCAN_PRIVATE {

template<typename VALUE_TYPE>
class constant_1 {
    public:
        inline double operator()(VALUE_TYPE it) const {
            return 1.0;
        }
};

template<typename INPUT_ITERATOR, typename WEIGHT_FUNC>
inline double neighborhoodWeight(INPUT_ITERATOR first, const INPUT_ITERATOR last, WEIGHT_FUNC &weight_func) {
    double result = 0;
    for (; first != last; ++first)
        result += weight_func(**first);
    return result;
}

/*
 * Private functions.
 */
template<typename INPUT_ITERATOR, typename DISTANCE_FUNC, typename OUTPUT_ITERATOR>
inline
void region_query(INPUT_ITERATOR first, const INPUT_ITERATOR last, const INPUT_ITERATOR center,
                  DISTANCE_FUNC &distance_func, OUTPUT_ITERATOR result, const double epsilon) {

    for (; first != last; ++first) {
        if (center == first) continue;
        if (distance_func(*center, *first) <= epsilon) {
            *result++ = first;
        }
    }
}


template<typename INPUT_ITERATOR, typename DISTANCE_FUNC, typename WEIGHT_FUNC>
inline
void expand_cluster(INPUT_ITERATOR first, const INPUT_ITERATOR last, const INPUT_ITERATOR center,
                    DISTANCE_FUNC &distance_func, const double epsilon, const double n_min,
                    std::vector<int> &id_cache, const int current_cluster_id, WEIGHT_FUNC &weight_func) {


    std::queue<INPUT_ITERATOR> bfq;
    bfq.push(center);

    id_cache[std::distance(first, center)] = current_cluster_id;

    std::vector<INPUT_ITERATOR> neighborhood; neighborhood.reserve(std::distance(first, last));

    while (!bfq.empty()) {

        INPUT_ITERATOR current_element = bfq.front();
        bfq.pop();

        /*
         * Precondition: id_cache[current_idx] > 0
         */

        neighborhood.clear();
        region_query(first, last, current_element, distance_func, std::back_inserter(neighborhood), epsilon);

        /*
         * If the current element is not inside a dense area,
         * we don't use it as a seed to expand the cluster.
         */
        if (neighborhoodWeight(neighborhood.begin(), neighborhood.end(), weight_func) < n_min)
            continue;

        /*
         * Push yet unvisited elements onto the queue.
         */
        for (typename std::vector<INPUT_ITERATOR>::iterator it = neighborhood.begin(), it_end = neighborhood.end();
                it != it_end; ++it) {
            const size_t neighbor_idx = std::distance(first, *it);
            /*
             * Is the element classified as non-noise, yet?
             */
            if (id_cache[neighbor_idx] <= 0) {
                /*
                 * Classify it and use it as a seed.
                 */
                id_cache[neighbor_idx] = current_cluster_id;
                bfq.push(*it);
            }
        }
    }
}

} /* namespace _DBSCAN_PRIVATE */

} /* namespace Algorithm */
} /* namespace ACG */
