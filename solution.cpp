#include "solution.h"

#include <iomanip>
#include <iostream>
#include <cmath>
#include <array>


Solution::Solution(std::string start_charger, std::string end_charger) :
    m_start_charger(start_charger),
    m_end_charger(end_charger),
    m_start_point(0,0),
    m_end_point(0,0) {

    double direction;
    init_src_and_dst_(direction);
    init_candidates_map(direction);
}


void Solution::init_src_and_dst_(double& direction) {
    for (auto& charger : network) {
        if (charger.name == m_start_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, -1, true));
            m_start_point = m_charger_map[charger.name]->point;
        } else if (charger.name == m_end_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, -1));
            m_end_point = m_charger_map[charger.name]->point;
        }
    }

    direction = SphericalUtil::computeHeading(m_start_point, m_end_point);
}


bool Solution::is_direction_good(LatLng& start_point, LatLng& cur_point, double direction) {
    double rdirection = double(360) - direction;
    double cur_direction = SphericalUtil::computeHeading(start_point, cur_point);
    double error = fabs(cur_direction - direction);
    if (std::min(double(360) - error, error) > 40) {
        return false;
    }
    return true;
}


void Solution::init_candidates_map(const double& direction) {
    for (auto& charger : network) {
        LatLng cur_point = {charger.lat, charger.lon};
        if (is_direction_good(m_start_point, cur_point, direction) && 
            is_direction_good(m_end_point, cur_point, direction)) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, -1));
        }
    }
}


double Solution::search_best_route(std::string start_charger) {
    if (m_charger_map[start_charger]->best_time >= 0) {
        return m_charger_map[start_charger]->best_time;
    }
    double direction = SphericalUtil::computeHeading(
        m_charger_map[start_charger]->point, m_end_point);
    double cur_time = 1e9;
    for (auto& charger : m_charger_map) {
        if (charger.second->visited) {
            continue;
        }
        if (!is_direction_good(m_charger_map[start_charger]->point,
            charger.second->point, direction)) {
            continue;
        }
        double dist = SphericalUtil::computeDistanceBetween(
            m_charger_map[start_charger]->point, charger.second->point);
        if (dist > 320000) {
            continue;
        }
    }
}


void Solution::run() {
    search_best_route(m_start_charger);
    return;
}