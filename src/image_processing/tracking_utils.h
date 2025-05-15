#pragma once

#include <cmath>
#include <opencv2/core/types.hpp>

const double earth_radius = 6378137.0; // in meters (WGS84)

/// @brief Apply 2d rotation matrix to the given [x y] vector 
///
///     [cos(theta)     -sin(theta)] [x
///     [sin(theta)      cos(theta)]  y]
///
/// @param x X coordinate in 2D space
/// @param y Y coordinate in 2D space
/// @param theta_degrees 
void rotate_point(double& x, double& y, float theta_degrees) {
    // Convert the angle from degrees to radians
    float theta_radians = theta_degrees * M_PI / 180.0;

    // Apply the rotation matrix
    float new_x = x * std::cos(theta_radians) - y * std::sin(theta_radians);
    float new_y = x * std::sin(theta_radians) + y * std::cos(theta_radians);

    // Update the original point with the new rotated coordinates
    x = new_x;
    y = new_y;
}

/// @brief Perform various operations to determine if 2 given rectangles are 'similar'
///         
///     Check difference between x and y coordinates, as well as difference between
///         height and width 
///
/// @param r1 
/// @param r2 
/// @param tol 
/// @return 
bool are_rectangles_similar(const cv::Rect2d& r1, const cv::Rect2d& r2, double tol = 30) {
    return (std::abs(r1.x - r2.x) < tol &&
            std::abs(r1.y - r2.y) < tol &&
            std::abs(r1.width - r2.width) < tol &&
            std::abs(r1.height - r2.height) < tol);
}