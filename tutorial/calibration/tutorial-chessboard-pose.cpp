#include <iostream>
#include <map>
#include <visp3/core/vpPoint.h>

#define eps 1e-6

// For std::map<vpPoint>
struct CompareObjectPointDegenerate {
  bool operator()(const vpPoint &point1, const vpPoint &point2) const
  {
    const double dist1 = point1.get_oX()*point1.get_oX() + point1.get_oY()*point1.get_oY() + point1.get_oZ()*point1.get_oZ();
    const double dist2 = point2.get_oX()*point2.get_oX() + point2.get_oY()*point2.get_oY() + point2.get_oZ()*point2.get_oZ();
    if (dist1 - dist2 < -3*eps*eps)
      return true;
    if (dist1 - dist2 > 3*eps*eps)
      return false;

    if (point1.oP[0] - point2.oP[0] < -eps)
      return true;
    if (point1.oP[0] - point2.oP[0] > eps)
      return false;

    if (point1.oP[1] - point2.oP[1] < -eps)
      return true;
    if (point1.oP[1] - point2.oP[1] > eps)
      return false;

    if (point1.oP[2] - point2.oP[2] < -eps)
      return true;
    if (point1.oP[2] - point2.oP[2] > eps)
      return false;

    return false;
  }
};

// For std::map<vpPoint>
struct CompareImagePointDegenerate {
  bool operator()(const vpPoint &point1, const vpPoint &point2) const
  {
    const double dist1 = point1.get_x()*point1.get_x() + point1.get_y()*point1.get_y();
    const double dist2 = point2.get_x()*point2.get_x() + point2.get_y()*point2.get_y();
    if (dist1 - dist2 < -2*eps*eps)
      return true;
    if (dist1 - dist2 > 2*eps*eps)
      return false;

    if (point1.p[0] - point2.p[0] < -eps)
      return true;
    if (point1.p[0] - point2.p[0] > eps)
      return false;

    if (point1.p[1] - point2.p[1] < -eps)
      return true;
    if (point1.p[1] - point2.p[1] > eps)
      return false;

    return false;
  }
};

int main()
{
  vpPoint pt1(-0.047064000000000002, 0.16838500000000001, -0.0010020000000000001);
  vpPoint pt2(-0.047063000100000002, 0.1683859999, -0.0010029999000000002);

  {
    std::ifstream f("debug_RANSAC_listOfPoints.data");
    double oX = 0, oY = 0, oZ = 0;
    double x = 0, y = 0;
    char c = ' ';

    std::vector<vpPoint> listOfPoints;
    while (f >> oX >> c >> oY >> c >> oZ >> c >> x >> c >> y) {
      vpPoint pt(oX, oY, oZ);
      pt.set_x(x);
      pt.set_y(y);

      listOfPoints.push_back(pt);
    }

    // Remove degenerate object points
    std::map<vpPoint, size_t, CompareObjectPointDegenerate> filterObjectPointMap;
    size_t index_pt = 0;
    for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end();
         ++it_pt, index_pt++) {
      const bool found = filterObjectPointMap.find(*it_pt) == filterObjectPointMap.end();
      const bool found1 = filterObjectPointMap.find(pt1) == filterObjectPointMap.end();
      const bool found2 = filterObjectPointMap.find(pt2) == filterObjectPointMap.end();
      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << it_pt->get_oX() << ", " << it_pt->get_oY() << ", " << it_pt->get_oZ()
                << " / " << it_pt->get_x() << ", " << it_pt->get_y() << " / " << found << " / " << found1 << " / " << found2 << std::endl;
      if (found) {
        filterObjectPointMap[*it_pt] = index_pt;
      }
    }
    std::cout << "\n\n\n\n" << std::endl;
    for (const auto& kv : filterObjectPointMap) {
      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << kv.first.get_oX() << ", " << kv.first.get_oY() << ", " << kv.first.get_oZ()
                << " / " << kv.first.get_x() << ", " << kv.first.get_y() << " / " << kv.second << std::endl;
    }

    std::cout << "\n\n\n\n" << std::endl;
    std::cout << "found pt1: " << (filterObjectPointMap.find(pt1) != filterObjectPointMap.end()) << std::endl;
    std::cout << "found pt2: " << (filterObjectPointMap.find(pt2) != filterObjectPointMap.end()) << std::endl;
  }

  return 0;
}
