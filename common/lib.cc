#include <iostream>
#include <cmath>
#include <ogr_geometry.h>
#include <ogr_spatialref.h>
#include "../common.h"


// =========================================================================
// FUNCTION 1: Compute Distance Between Two Lat/Lon Points
// =========================================================================
double Global::computeDistance(double lonA, double latA, double lonB, double latB, OGRCoordinateTransformation* poCT) {
    OGRPoint pointA(lonA, latA);
    OGRPoint pointB(lonB, latB);

    // Transform points to the metric projection (e.g., EPSG:7755)
    if (pointA.transform(poCT) != OGRERR_NONE || pointB.transform(poCT) != OGRERR_NONE) {
        std::cerr << "Distance transformation failed." << std::endl;
        return -1.0;
    }

    // Returns distance in meters
    return pointA.Distance(&pointB);
}

// =========================================================================
// FUNCTION 2: Compute Bearing Between Two Lat/Lon Points
// =========================================================================
double Global::computeBearing(double lonA, double latA, double lonB, double latB, OGRCoordinateTransformation* poCT) {
    OGRPoint pointA(lonA, latA);
    OGRPoint pointB(lonB, latB);

    // Transform points to the metric projection
    if (pointA.transform(poCT) != OGRERR_NONE || pointB.transform(poCT) != OGRERR_NONE) {
        std::cerr << "Bearing transformation failed." << std::endl;
        return -1.0;
    }

    double deltaX = pointB.getX() - pointA.getX();
    double deltaY = pointB.getY() - pointA.getY();

    // Calculate angle clockwise from North (Y-axis)
    double bearingDeg = rad2deg(std::atan2(deltaX, deltaY));

    // Normalize to 0° - 360°
    if (bearingDeg < 0) {
        bearingDeg += 360.0;
    }

    return bearingDeg;
}

// =========================================================================
// FUNCTION 3: Find Destination Point from Source, Distance, and Bearing
// =========================================================================
bool Global::dropPointAtDistanceAndBearing(double startLon, double startLat,
                                   double distanceMeters, double bearingDegrees,
                                   OGRCoordinateTransformation* poForwardCT,
                                   OGRCoordinateTransformation* poInverseCT,
                                   double& outLon, double& outLat) {
    OGRPoint pointA(startLon, startLat);

    // Transform start point to metric projection
    if (pointA.transform(poForwardCT) != OGRERR_NONE) {
        std::cerr << "Forward projection failed for starting point." << std::endl;
        return false;
    }

    double bearingRad = deg2rad(bearingDegrees);

    // Calculate new Cartesian coordinates
    double newX = pointA.getX() + (distanceMeters * std::sin(bearingRad));
    double newY = pointA.getY() + (distanceMeters * std::cos(bearingRad));

    OGRPoint pointC(newX, newY);

    // Transform back to WGS84 Lat/Lon
    if (pointC.transform(poInverseCT) != OGRERR_NONE) {
        std::cerr << "Inverse transformation failed for destination point." << std::endl;
        return false;
    }

    outLon = pointC.getX();
    outLat = pointC.getY();
    return true;
}

// =========================================================================
// MAIN EXECUTION
// =========================================================================
int main() {
    GDALAllRegister();

    // Setup SRS Systems
    OGRSpatialReference srcSRS, targetSRS;
    srcSRS.importFromEPSG(4326); // WGS 84
    srcSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    targetSRS.importFromEPSG(7755); // India Zone I
    targetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    // Create Transformations (Pass these into your functions)
    OGRCoordinateTransformation* poCT = OGRCreateCoordinateTransformation(&srcSRS, &targetSRS);
    OGRCoordinateTransformation* poInvCT = OGRCreateCoordinateTransformation(&targetSRS, &srcSRS);

    if (!poCT || !poInvCT) {
        std::cerr << "Failed to initialize coordinate transforms." << std::endl;
        return 1;
    }

    // Sample Data (Mumbai & Delhi)
    double mumLon = 72.8777, mumLat = 19.0760;
    double delLon = 77.2090, delLat = 28.6139;

    // Call Function 1: Distance
    double dist = Global::computeDistance(mumLon, mumLat, delLon, delLat, poCT);
    std::cout << "Distance: " << (dist / 1000.0) << " km" << std::endl;

    // Call Function 2: Bearing
    double brng = Global::computeBearing(mumLon, mumLat, delLon, delLat, poCT);
    std::cout << "Bearing: " << brng << "°" << std::endl;

    // Call Function 3: Destination
    double destLon = 0.0, destLat = 0.0;
    double travelDist = 500000.0; // 500 km
    double travelHeading = 45.0;  // NE

    if (Global::dropPointAtDistanceAndBearing(mumLon, mumLat, travelDist, travelHeading, poCT, poInvCT, destLon, destLat)) {
        std::cout << "Destination Point -> Lon: " << destLon << ", Lat: " << destLat << std::endl;
    }

    // Cleanup transforms
    OGRCoordinateTransformation::DestroyCT(poCT);
    OGRCoordinateTransformation::DestroyCT(poInvCT);

    return 0;
}
