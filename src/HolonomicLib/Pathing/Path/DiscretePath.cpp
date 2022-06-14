#include "DiscretePath.hpp"

namespace HolonomicLib{

DiscretePath::DiscretePath(const std::initializer_list<Point> &iwaypoint) : path(iwaypoint){}

DiscretePath::DiscretePath(const std::vector<Point> &iwaypoint) : path(iwaypoint){}

DiscretePath DiscretePath::operator+(const DiscretePath &rhs) const{
    DiscretePath result(path);
    result.path.insert(result.path.end(), rhs.path.begin(), rhs.path.end());
    return result;
}

DiscretePath DiscretePath::operator+(const Point &rhs) const{
    DiscretePath result(path);
    result.path.push_back(rhs);
    return result;
}

DiscretePath& DiscretePath::operator+=(const DiscretePath &rhs){
    path.insert(path.end(), rhs.path.begin(), rhs.path.end());
    return *this;
}

DiscretePath& DiscretePath::operator+=(const Point &rhs){
    path.push_back(rhs);
    return *this;
}

Point DiscretePath::getPoint(int index) const{
    if(index < 0 || index >= (int)path.size()) return Point(0 * okapi::meter, 0 * okapi::meter);
    return path[index];
}

Point DiscretePath::operator[](int index) const{
    return getPoint(index);
}


int DiscretePath::size() const{
    return (int)path.size();
}

}