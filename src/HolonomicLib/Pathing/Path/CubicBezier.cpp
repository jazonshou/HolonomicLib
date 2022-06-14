#include "CubicBezier.hpp"

namespace HolonomicLib{


CubicBezier::Knot::Knot(okapi::QLength ix, okapi::QLength iy, okapi::QAngle iangle, okapi::QLength imag){
    x = ix; y = iy; angle = iangle; mag = imag;
}

CubicBezier::CubicBezier(const Point &istart, const Point &icontrol1, const Point &icontrol2, const Point &iend){
    start = istart; control1 = icontrol1; control2 = icontrol2; end = iend;
}

CubicBezier::CubicBezier(const Knot &istart, const Knot &iend){
    start = Point(istart.x, istart.y);
    end = Point(iend.x, iend.y);
    control1 = start + Point(istart.mag * cos(istart.angle), istart.mag * sin(istart.angle));
    control2 = end + Point(iend.mag * cos(iend.angle + M_PI*okapi::radian), iend.mag * sin(iend.angle+M_PI*okapi::radian));
}

Point CubicBezier::getPoint(double t) const{
    return start*(1-t)*(1-t)*(1-t) + control1*3*(1-t)*(1-t)*t + control2*3*(1-t)*t*t + end*t*t*t;
}


DiscretePath CubicBezier::generate(int istep, bool iend) const{
    if(istep < 1){
        throw std::invalid_argument("CubicBezier::generate(): step cannot be smaller than 1");
    }

    std::vector<Point> path;
    path.reserve(istep);
    double step = 1.0 / (istep);
    for(int i = 0; i < istep; i++){
        double t = i * step;
        path.emplace_back(getPoint(t));
    }

    if(iend){
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

}