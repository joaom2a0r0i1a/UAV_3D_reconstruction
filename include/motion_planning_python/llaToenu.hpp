#include <iostream>
#include <cmath>
#include <array>

const double a = 6378137.0;
const double b = 6356752.314245;
const double e_sq = 1 - (b * b) / (a * a);

std::array<double, 3> llaToEcef(double lat, double lon, double alt) {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    double z = (N * (1 - e_sq) + alt) * sin(lat_rad);

    return {x, y, z};
}

std::array<double, 3> ecefToEnu(double lat_ref, double lon_ref, double x, double y, double z, double x_ref, double y_ref, double z_ref) {
    double lat_ref_rad = lat_ref * M_PI / 180.0;
    double lon_ref_rad = lon_ref * M_PI / 180.0;

    double dx = x - x_ref;
    double dy = y - y_ref;
    double dz = z - z_ref;

    double t[3][3] = {
        {-sin(lon_ref_rad), cos(lon_ref_rad), 0},
        {-sin(lat_ref_rad) * cos(lon_ref_rad), -sin(lat_ref_rad) * sin(lon_ref_rad), cos(lat_ref_rad)},
        {cos(lat_ref_rad) * cos(lon_ref_rad), cos(lat_ref_rad) * sin(lon_ref_rad), sin(lat_ref_rad)}
    };

    double e = t[0][0] * dx + t[0][1] * dy + t[0][2] * dz;
    double n = t[1][0] * dx + t[1][1] * dy + t[1][2] * dz;
    double u = t[2][0] * dx + t[2][1] * dy + t[2][2] * dz;

    return {e, n, u};
}

std::array<double, 3> llaToEnu(double lat_ref, double lon_ref, double alt_ref, double lat, double lon, double alt) {
    std::array<double, 3> ecef_ref = llaToEcef(lat_ref, lon_ref, alt_ref);
    std::array<double, 3> ecef = llaToEcef(lat, lon, alt);

    return ecefToEnu(lat_ref, lon_ref, ecef[0], ecef[1], ecef[2], ecef_ref[0], ecef_ref[1], ecef_ref[2]);
}
