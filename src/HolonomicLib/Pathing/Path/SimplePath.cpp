#include "SimplePath.hpp"

namespace HolonomicLib{

SimplePath::SimplePath(const std::initializer_list<Point> &iwaypoint):waypoint(iwaypoint){}

SimplePath& SimplePath::generate(int istep, bool iend){
    if(istep < 1){
        throw std::invalid_argument("SimplePath::generate(): step cannot be smaller than one");
    }

    std::vector<Point> path;
    for(int i = 0; i < waypoint.size()-1; i++){
         Point diff = waypoint[i+1]-waypoint[i];
         Point inc = diff/istep;

        for(int j = 0; j < istep; j++){
            path.push_back(waypoint[i]+(inc*j));
        }
    }

    if(waypoint.size() != 0 && iend){
        path.push_back(waypoint[waypoint.size()-1]);
    }

    waypoint = path;

    return *this;
}

SimplePath& SimplePath::generate(okapi::QLength ilength, bool iend){
    if(ilength == 0 * okapi::meter){
        throw std::invalid_argument("SimplePath::generate(): length cannot be 0");
    }

    std::vector<Point> path;
    for(int i = 0; i < waypoint.size()-1; i++){
        int step = std::ceil((waypoint[i].distTo(waypoint[i+1]) / ilength).convert(okapi::number)); 
        Point diff = waypoint[i+1] - waypoint[i];
        Point inc = diff / step;

        for(int j = 0; j < step; j++){
            path.push_back(waypoint[i]+(inc*j));
        }
    }

    if(waypoint.size() != 0 && iend){
        path.push_back(waypoint[waypoint.size()-1]);
    }

    waypoint = path;

    return *this;
}

DiscretePath SimplePath::smoothen(double ismoothWeight, okapi::QLength itolerance){
    okapi::QLength change = itolerance;
    std::vector<Point> newPath = waypoint;
    double a = 1-ismoothWeight, b = ismoothWeight;

    while(change >= itolerance){
        change = 0 * okapi::meter;
        for(int i = 1; i < waypoint.size()-1; i++){
            Point aux = newPath[i];

            newPath[i].setX(newPath[i].X() + a * (waypoint[i].X() - newPath[i].X()) + b * (newPath[i-1].X() +
            newPath[i+1].X() - (2.0 * newPath[i].X())));

            newPath[i].setY(newPath[i].Y() + a * (waypoint[i].Y() - newPath[i].Y()) + b * (newPath[i-1].Y() +
            newPath[i+1].Y() - (2.0 * newPath[i].Y())));

            change += abs(aux.X() + aux.Y() - newPath[i].X() - newPath[i].Y());
        }   
    }

    return DiscretePath(newPath);
}

DiscretePath SimplePath::noSmoothen(){
    return DiscretePath(waypoint);
}
}
