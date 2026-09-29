/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _MULTIAGENT_COLLISON_CHECKER_CPP_
#define _MULTIAGENT_COLLISON_CHECKER_CPP_

#include <algorithm>
#include <string>
#include <ros/ros.h>
#include <multiagent_collision_check/multiagent_collision_checker.h>

bool multiagent::isInCollision(const Eigen::Vector4d& start, const Eigen::Vector4d& end,
                               const double safety_radius,
                               const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths) {
    for (typename std::vector<std::vector<Eigen::Vector3d>*>::const_iterator it = agent_paths.begin();
         it != agent_paths.end(); it++) {
        for (int it_segment = 1; it_segment < (*it)->size(); it_segment++) {
            if (safety_radius > closestDistanceBetweenLines(Eigen::Vector3d(start.x(), start.y(), start.z()),
                                                            Eigen::Vector3d(end.x(), end.y(), end.z()),
                                                            (**it)[it_segment - 1], (**it)[it_segment])) {
                return true;
            }
        }
    }
    return false;
}

bool multiagent::isInCollision(const Eigen::Vector4d& state, const double safety_radius,
                               const std::vector<std::vector<Eigen::Vector3d>*>& agent_paths) {
    for (typename std::vector<std::vector<Eigen::Vector3d>*>::const_iterator it = agent_paths.begin();
         it != agent_paths.end(); it++) {
        for (int it_segment = 1; it_segment < (*it)->size(); it_segment++) {
            if (safety_radius > closestDistanceBetweenLines(Eigen::Vector3d(state.x(), state.y(), state.z()),
                                                            Eigen::Vector3d(state.x(), state.y(), state.z()),
                                                            (**it)[it_segment - 1], (**it)[it_segment])) {
                return true;
            }
        }
    }
    return false;
}

// Keeps a fraction inside the segment, 0 at the start and 1 at the end
double multiagent::clampFraction(const double fraction) {
    return std::min(std::max(fraction, 0.0), 1.0);
}

// Closest point on a segment to a point, also for a zero length segment
Eigen::Vector3d multiagent::closestPointOnSegment(const Eigen::Vector3d& point,
                                                  const Eigen::Vector3d& segment_start,
                                                  const Eigen::Vector3d& segment_end) {
    const Eigen::Vector3d direction = segment_end - segment_start;
    const double length_squared = direction.squaredNorm();
    if (length_squared == 0) {
        return segment_start;
    }
    const double fraction = clampFraction((point - segment_start).dot(direction) / length_squared);
    return segment_start + direction * fraction;
}

double multiagent::closestDistanceBetweenLines(const Eigen::Vector3d& start1,
                                               const Eigen::Vector3d& end1,
                                               const Eigen::Vector3d& start2,
                                               const Eigen::Vector3d& end2) {
    const Eigen::Vector3d direction1 = end1 - start1;
    const Eigen::Vector3d direction2 = end2 - start2;
    const double length1_squared = direction1.squaredNorm();
    const double length2_squared = direction2.squaredNorm();

    // Zero Length Segments
    if (length1_squared == 0) {
        return (closestPointOnSegment(start1, start2, end2) - start1).norm();
    }
    if (length2_squared == 0) {
        return (closestPointOnSegment(start2, start1, end1) - start2).norm();
    }

    const Eigen::Vector3d start_offset = start1 - start2;
    const double directions_dot = direction1.dot(direction2);
    const double offset_along1 = direction1.dot(start_offset);
    const double offset_along2 = direction2.dot(start_offset);

    // Closest Point of the Infinite Lines on Segment 1, the start for parallel lines
    const double denominator = length1_squared * length2_squared - directions_dot * directions_dot;
    double fraction1 = 0.0;
    if (denominator > 1e-12 * length1_squared * length2_squared) {
        fraction1 = (directions_dot * offset_along2 - length2_squared * offset_along1) / denominator;
    }
    fraction1 = clampFraction(fraction1);

    // Best Point on Segment 2 for It, then Back on Segment 1
    const double fraction2 = clampFraction((directions_dot * fraction1 + offset_along2) / length2_squared);
    fraction1 = clampFraction((directions_dot * fraction2 - offset_along1) / length1_squared);

    const Eigen::Vector3d closest1 = start1 + direction1 * fraction1;
    const Eigen::Vector3d closest2 = start2 + direction2 * fraction2;
    return (closest1 - closest2).norm();
}

#endif  // _MULTIAGENT_COLLISON_CHECKER_CPP_
