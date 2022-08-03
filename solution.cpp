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
    init_candidates_map_(direction);
}


void Solution::init_src_and_dst_(double& direction) {
    for (auto& charger : network) {
        if (charger.name == m_start_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 1e9));
            m_start_point = m_charger_map[charger.name]->point;
        } else if (charger.name == m_end_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 0));
            m_end_point = m_charger_map[charger.name]->point;
            m_charger_map[charger.name]->best_stop = {0, charger.name};
        }
    }

    direction = SphericalUtil::computeHeading(m_start_point, m_end_point);
}


bool Solution::is_direction_good(LatLng& start_point, LatLng& cur_point, double direction) {
    double cur_direction = SphericalUtil::computeHeading(start_point, cur_point);
    double error = fabs(cur_direction - direction);
    if (std::min(double(360) - error, error) > 80) {
        return false;
    }
    return true;
}


void Solution::init_candidates_map_(const double& direction) {
    for (auto& charger : network) {
        LatLng cur_point = {charger.lat, charger.lon};
        if (is_direction_good(m_start_point, cur_point, direction) && 
            is_direction_good(cur_point, m_end_point, direction)) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 1e9));
        }
    }
}


void Solution::search_best_route(std::string start_charger) {
    if (m_charger_map[start_charger]->visited) {
        return;
    }
    // std::cout << "start with: " << start_charger << std::endl;
    m_charger_map[start_charger]->visited = true;
    double direction = SphericalUtil::computeHeading(
        m_charger_map[start_charger]->point, m_end_point);
    for (auto& charger : m_charger_map) {
        std::cout << "    checking: " << charger.first << std::endl;
        if (!is_direction_good(m_charger_map[start_charger]->point,
            charger.second->point, direction)) {
            // std::cout << "    bad direction: " << charger.first << std::endl;
            continue;
        }
        double dist = SphericalUtil::computeDistanceBetween(
            m_charger_map[start_charger]->point, charger.second->point);
        if (dist > 320000) {
            // std::cout << "    bad distance: " << dist << std::endl;
            continue;
        }
        double cur_charge_time = dist / m_charger_map[start_charger]->rate / 1000;
        if (charger.first == m_end_charger) {
            m_charger_map[start_charger]->best_time = cur_charge_time;
            m_charger_map[start_charger]->best_stop = {cur_charge_time, charger.first};
            std::cout << charger.first << std::endl;
            return;
        }
        std::cout << "    good candidate: " << charger.first << std::endl;
        search_best_route(charger.first);
        if (charger.second->best_time + cur_charge_time
            < m_charger_map[start_charger]->best_time) {
            m_charger_map[start_charger]->best_time =
                charger.second->best_time + cur_charge_time;
            m_charger_map[start_charger]->best_stop = {cur_charge_time, charger.first};
        }

    }
    m_charger_map[start_charger]->visited = false;
}


void Solution::print_path(std::string charger) {
    if (charger != m_charger_map[charger]->best_stop.second) {
        if (m_charger_map[charger]->best_stop.second == "") {
            std::cout << "\nAnswer Not Found!\n";
            return;
        }
        std::cout << m_charger_map[charger]->best_stop.second <<
            "," << m_charger_map[charger]->best_stop.first << ",";
        print_path(m_charger_map[charger]->best_stop.second);
    } else {
        std::cout << charger;
    }
}


void Solution::run() {
    search_best_route(m_start_charger);
    std::cout << "\"" << m_start_charger << ",";
    print_path(m_start_charger);
    std::cout << "\"";

    return;
}