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

#ifndef DBSCAN_HH_
#define DBSCAN_HH_

#include <vector>
#include <iterator>
#include <algorithm>

/*
 * Private functions.
 */
#include "DBSCANT_impl.hh"

namespace ACG {
namespace Algorithm {

/**
 * Implements the DBSCAN algorithm introduced in
 *
 * Ester, Martin, Hans-Peter Kriegel, Jörg S, and Xiaowei Xu. “A Density-based Algorithm for Discovering Clusters in Large Spatial Databases with Noise.” 226–231. AAAI Press, 1996.
 *
 * Classifies the sequence [first, last) into clusters. Outputs `int`s into the result sequence, one
 * for each element in the input sequence. `0` means noise, values greater than 0 specify a cluster index.
 *
 * Returned cluster indices are guaranteed to be a continuous range starting at 1.
 *
 *
 * Result has to support the operation `*result++ = <int>`.
 *
 * Example:
 *
 *     std::vector<Point> points;
 *     std::vector<int> clusters;
 *     ...
 *     ACG::Algorithm::DBSCAN(points.begin(), points.end(), Point::DistanceFunc(),
 *                            std::back_inserter(clusters), 4.0001, 3)
 *
 * @param Input iterator to the initial position of the data set.
 * @param Input iterator to the final position of the data set.
 * @param distance_func Binary function taking two elements as arguments. Returns the distance between these elements.
 * @param result Output iterator to the initial position of the result range. The range includes as many elements as [first, last).
 * @param epsilon The density-reachable neighborhood radius.
 * @param n_min The density-reachable count threshold.
 * @param weight_func Unary function taking an element as its argument. Returns the weight with which it is to be considered for DBSCAN.
 *   The off the shelf DBSCAN uses a constant 1, here.
 * @return The number of clusters found.
 */
template<typename INPUT_ITERATOR, typename DISTANCE_FUNC, typename OUTPUT_ITERATOR, typename WEIGHT_FUNC>
int DBSCAN(const INPUT_ITERATOR first, const INPUT_ITERATOR last, DISTANCE_FUNC distance_func,
           OUTPUT_ITERATOR result, const double epsilon, const double n_min, WEIGHT_FUNC weight_func) {

    const size_t input_size = std::distance(first, last);

    std::vector<int> id_cache(input_size, -1);

    int idx = 0;
    int current_cluster_id = 0;

    for (INPUT_ITERATOR it = first; it != last; ++it, ++idx) {

        // Visit every element only once.
        if (id_cache[idx] >= 0) continue;

        // Gather neighborhood.
        std::vector<INPUT_ITERATOR> neighborhood; neighborhood.reserve(input_size);
        _DBSCAN_PRIVATE::region_query(first, last, it, distance_func, std::back_inserter(neighborhood), epsilon);

        if (_DBSCAN_PRIVATE::neighborhoodWeight(neighborhood.begin(), neighborhood.end(), weight_func) < n_min) {
            // It's noise.
            id_cache[idx] = 0;
        } else {
            // It's the seed of a cluster.
            _DBSCAN_PRIVATE::expand_cluster(first, last, it, distance_func, epsilon, n_min, id_cache, ++current_cluster_id, weight_func);
        }
    }

    std::copy(id_cache.begin(), id_cache.end(), result);

    return current_cluster_id;
}

/**
 * Version of DBSCAN with weight_func being a constant 1.
 */
template<typename INPUT_ITERATOR, typename DISTANCE_FUNC, typename OUTPUT_ITERATOR>
int DBSCAN(const INPUT_ITERATOR first, const INPUT_ITERATOR last, DISTANCE_FUNC distance_func,
           OUTPUT_ITERATOR result, const double epsilon, const double n_min) {
    return DBSCAN(first, last, distance_func, result, epsilon, n_min, typename _DBSCAN_PRIVATE::constant_1<typename INPUT_ITERATOR::value_type>());
}

} /* namespace Algorithm */
} /* namespace ACG */
#endif /* DBSCAN_HH_ */
