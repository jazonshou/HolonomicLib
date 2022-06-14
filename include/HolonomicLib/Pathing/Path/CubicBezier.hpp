#pragma once
#include "HolonomicLib/Pathing/Geometry/Translation.hpp"
#include "HolonomicLib/Pathing/Path/DiscretePath.hpp"
#include "HolonomicLib/Utility/Math.hpp"
#include "HolonomicLib/Utility/Units.hpp"
#include<vector>

namespace HolonomicLib{

class CubicBezier{
    public:
    struct Knot{
		okapi::QLength x, y; okapi::QAngle angle; okapi::QLength mag = 1 * okapi::meter;
		Knot(okapi::QLength ix, okapi::QLength iy, okapi::QAngle iangle, okapi::QLength imag = 1 * okapi::meter);
	};

    CubicBezier(const Point &istart, const Point &icontrol1, const Point &icontrol2, const Point &iend);

    CubicBezier(const Knot &istart, const Knot &iend);

    Point getPoint(double t) const;

    DiscretePath generate(int istep, bool iend = true) const;

    private:
    Point start, control1, control2, end;
};
}
