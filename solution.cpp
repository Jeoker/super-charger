#include "solution.h"
#include "utm/datum.h"
#include "utm/utm.h"

#include <iomanip>
#include <iostream>
#include <cmath>


Solution::Solution() {
    // for (auto &node : network) {
		auto node = network[0];
        GridZone zone = GRID_AUTO;
        Hemisphere hemi = HEMI_AUTO;
        const Ellipse* e = standard_ellipse(EllipseID(ELLIPSE_WGS84));
        std::cout << node.lon << " " << node.lat << " ";
		double N=0,E=0;
        geographic_to_grid(e->a, e->e2, node.lat*180.f/M_PI, node.lon*180.f/M_PI, &zone, &hemi, &N, &E);
        std::cout << N << " " << E << std::endl;
    // }
}

void Solution::run(std::string start_charger, std::string end_charger) {
}