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

    double direction = init_src_and_dst_();
    init_candidates_map_(direction);
}


double Solution::init_src_and_dst_() {
    for (auto& charger : network) {
        if (charger.name == m_end_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 0, true));
            m_end_point = m_charger_map[charger.name]->point;
            m_charger_map[charger.name]->best_stop = {0, charger.name};
        } else if (charger.name == m_start_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, mk_max_time));
            m_start_point = m_charger_map[charger.name]->point;
        }
    }

    return SphericalUtil::computeHeading(m_start_point, m_end_point);
}


bool Solution::is_direction_good_(LatLng& start_point, LatLng& cur_point, double direction) {
    auto cur_direction = SphericalUtil::computeHeading(start_point, cur_point);
    auto error = fabs(cur_direction - direction);
    if (std::min(double(360) - error, error) > 90) {
        return false;
    }
    return true;
}


void Solution::init_candidates_map_(const double direction) {
    for (auto& charger : network) {
        LatLng cur_point = {charger.lat, charger.lon};
        if (is_direction_good_(m_start_point, cur_point, direction) && 
            is_direction_good_(cur_point, m_end_point, direction)) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, mk_max_time));
        }
    }
}


void Solution::search_best_route_(std::string start_charger) {
    if (m_charger_map[start_charger]->visited) {
        return;
    }
    m_charger_map[start_charger]->visited = true;
    double direction = SphericalUtil::computeHeading(
        m_charger_map[start_charger]->point, m_end_point);
    for (auto& charger : m_charger_map) {
        if (!is_direction_good_(m_charger_map[start_charger]->point,
            charger.second->point, direction)) {
            continue;
        }
        double dist = SphericalUtil::computeDistanceBetween(
            m_charger_map[start_charger]->point, charger.second->point) / double(1000);
        if (dist > mk_max_range) {
            continue;
        }
        auto cur_charge_time = dist / m_charger_map[start_charger]->rate;
        if (charger.first == m_end_charger) {
            m_charger_map[start_charger]->best_time = cur_charge_time;
            m_charger_map[start_charger]->best_stop = {cur_charge_time, charger.first};
            return;
        }
        search_best_route_(charger.first);
        if (charger.second->best_time + cur_charge_time
            < m_charger_map[start_charger]->best_time) {
            m_charger_map[start_charger]->best_time =
                charger.second->best_time + cur_charge_time;
            m_charger_map[start_charger]->best_stop = {cur_charge_time, charger.first};
        }

    }
}


std::string Solution::refine_path_(std::string start_charger) {
    auto next_charger = m_charger_map[start_charger]->best_stop.second;
    auto range = mk_max_range;
    auto ans = start_charger;
    while (start_charger != m_end_charger) {
        if (next_charger.size() == 0) {
            ans += std::string("cannot not reach the destination");
            return ans;
        }
        auto charge_time = double(0);
        auto dist = SphericalUtil::computeDistanceBetween(
            m_charger_map[start_charger]->point, m_charger_map[next_charger]->point) /
            double(1000);
        if (m_charger_map[start_charger]->rate > m_charger_map[next_charger]->rate) {
            charge_time = (mk_max_range - range) / m_charger_map[start_charger]->rate;
            range = mk_max_range - dist;
        } else {
            charge_time = (dist - range) / m_charger_map[start_charger]->rate;
            range = 0;
        }
        if (charge_time > 0) {
            ans += ",";
            ans += std::to_string(charge_time);
        }
        ans += ",";
        ans += next_charger;
        start_charger = next_charger;
        next_charger = m_charger_map[start_charger]->best_stop.second;
    }
    return ans;
}


void Solution::run() {
    if (m_start_charger != m_end_charger) {
        search_best_route_(m_start_charger);
    } else {
        std::cout << "the start charger is the same as the end charger, quit" << std::endl;
        return;
    }
    std::cout << refine_path_(m_start_charger) << std::endl;

    return;
}