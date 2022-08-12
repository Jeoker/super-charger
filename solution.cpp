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
                Node(charger.lat, charger.lon, charger.rate, 0, 0, true));
            m_end_point = m_charger_map[charger.name]->point;
            m_charger_map[charger.name]->best_full_charge_stop =
                {0, {charger.name, Strategy::MIN}};
            m_charger_map[charger.name]->best_min_charge_stop =
                {0, {charger.name, Strategy::MIN}};
        } else if (charger.name == m_start_charger) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate));
            m_start_point = m_charger_map[charger.name]->point;
        }
    }

    return SphericalUtil::computeHeading(m_start_point, m_end_point);
}


bool Solution::is_direction_good_(LatLng& start_point, LatLng& cur_point, double direction) {
    auto cur_direction = SphericalUtil::computeHeading(start_point, cur_point);
    auto error = fabs(cur_direction - direction);
    if (std::min(double(360) - error, error) > 135) {
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
                Node(charger.lat, charger.lon, charger.rate));
        }
    }
}


bool Solution::charger_in_path_(std::string target_charger,
	std::string cur_charger, Strategy strategy) {
	auto next_charger = std::string();
	while (cur_charger != m_end_charger) {
        if (strategy == Strategy::MIN) {
            next_charger = m_charger_map[cur_charger]->best_min_charge_stop.second.first;
            strategy = m_charger_map[cur_charger]->best_min_charge_stop.second.second;
        } else {
            next_charger = m_charger_map[cur_charger]->best_full_charge_stop.second.first;
            strategy = m_charger_map[cur_charger]->best_full_charge_stop.second.second;
        }
        if (next_charger.size() == 0) {
            return false;
        } else if (next_charger == target_charger) {
			return true;
		}
        cur_charger = next_charger;
    }
	return false;
}


void Solution::search_best_route_(std::string start_charger) {
    if (m_charger_map[start_charger]->optimized) {
        return;
    }
    m_charger_map[start_charger]->optimized = true;
    double direction = SphericalUtil::computeHeading(
        m_charger_map[start_charger]->point, m_end_point);
    auto debug_start = &m_charger_map[start_charger];
    for (auto& next_charger : m_charger_map) {
        if (start_charger == next_charger.first ||
		 	m_charger_map[start_charger]->visited ||
            !is_direction_good_(m_charger_map[start_charger]->point,
                next_charger.second->point, direction)) {
            continue;
        }
        double dist = SphericalUtil::computeDistanceBetween(
            m_charger_map[start_charger]->point, next_charger.second->point) / double(1000);
        if (dist > mk_max_range) {
            continue;
        }
        if (next_charger.first == m_end_charger) {
            m_charger_map[start_charger]->best_min_charge_total_time =
                dist / m_charger_map[start_charger]->rate;
            m_charger_map[start_charger]->best_min_charge_stop =
                {dist, {next_charger.first, Strategy::MIN}};
            m_charger_map[start_charger]->best_full_charge_total_time =
                mk_max_range / m_charger_map[start_charger]->rate;
            m_charger_map[start_charger]->best_full_charge_stop =
                {dist, {next_charger.first, Strategy::MIN}};
            return;
        }
        m_charger_map[start_charger]->visited = true;
        search_best_route_(next_charger.first);
        m_charger_map[start_charger]->visited = false;
        if (next_charger.second->best_full_charge_total_time == mk_max_time &&
            next_charger.second->best_min_charge_total_time == mk_max_time) {
            continue;
        }
        if (m_charger_map[start_charger]->rate > m_charger_map[next_charger.first]->rate) {
            auto cur_fully_charge_time =
                mk_max_range / m_charger_map[start_charger]->rate -
                (mk_max_range - dist) / m_charger_map[next_charger.first]->rate;
            if (m_charger_map[next_charger.first]->best_min_charge_total_time <
                m_charger_map[next_charger.first]->best_full_charge_total_time) {
                cur_fully_charge_time += m_charger_map[next_charger.first]->best_min_charge_total_time;
                if (cur_fully_charge_time < m_charger_map[start_charger]->best_full_charge_total_time &&
					!charger_in_path_(start_charger, next_charger.first, Strategy::MIN)) {
                    m_charger_map[start_charger]->best_full_charge_total_time = cur_fully_charge_time;
                    m_charger_map[start_charger]->best_full_charge_stop =
                        {dist, {next_charger.first, Strategy::MIN}};
                }
            } else {
                cur_fully_charge_time += m_charger_map[next_charger.first]->best_full_charge_total_time;
                if (cur_fully_charge_time < m_charger_map[start_charger]->best_full_charge_total_time &&
					!charger_in_path_(start_charger, next_charger.first, Strategy::FULL)) {
                    m_charger_map[start_charger]->best_full_charge_total_time = cur_fully_charge_time;
                    m_charger_map[start_charger]->best_full_charge_stop =
                        {dist, {next_charger.first, Strategy::FULL}};
                }
            }
        } else {
            auto cur_min_charge_time =
                dist / m_charger_map[start_charger]->rate;
            if (m_charger_map[next_charger.first]->best_min_charge_total_time <
                m_charger_map[next_charger.first]->best_full_charge_total_time) {
                cur_min_charge_time += m_charger_map[next_charger.first]->best_min_charge_total_time;
                if (cur_min_charge_time < m_charger_map[start_charger]->best_min_charge_total_time &&
					!charger_in_path_(start_charger, next_charger.first, Strategy::MIN)) {
                    m_charger_map[start_charger]->best_min_charge_total_time = cur_min_charge_time;
                    m_charger_map[start_charger]->best_min_charge_stop =
                        {dist, {next_charger.first, Strategy::MIN}};
                }
            } else {
                cur_min_charge_time += m_charger_map[next_charger.first]->best_full_charge_total_time;
                if (cur_min_charge_time < m_charger_map[start_charger]->best_min_charge_total_time &&
					!charger_in_path_(start_charger, next_charger.first, Strategy::FULL)) {
                    m_charger_map[start_charger]->best_min_charge_total_time = cur_min_charge_time;
                    m_charger_map[start_charger]->best_min_charge_stop =
                        {dist, {next_charger.first, Strategy::FULL}};
                }
            }
        }
    }
}


std::string Solution::refine_path_(std::string start_charger) {
    auto next_charger = std::string();
    auto residue_range = mk_max_range;
    auto strategy = Strategy::MIN;
    if (m_charger_map[start_charger]->best_min_charge_total_time <
        m_charger_map[start_charger]->best_full_charge_total_time) {
        next_charger = m_charger_map[start_charger]->best_min_charge_stop.second.first;
        residue_range -= m_charger_map[start_charger]->best_min_charge_stop.first;
        strategy = m_charger_map[start_charger]->best_min_charge_stop.second.second;
    } else {
        next_charger = m_charger_map[start_charger]->best_full_charge_stop.second.first;
        residue_range -= m_charger_map[start_charger]->best_full_charge_stop.first;
        strategy = m_charger_map[start_charger]->best_full_charge_stop.second.second;
    }
    auto ans = start_charger + "," + next_charger;
    if (next_charger.size() == 0) {
        ans += std::string("starting point not going anywhere");
        return ans;
    }
    start_charger = next_charger;
    while (start_charger != m_end_charger) {
        auto charge_time = double(0);
        if (strategy == Strategy::MIN) {
            next_charger = m_charger_map[start_charger]->best_min_charge_stop.second.first;
            charge_time = (m_charger_map[start_charger]->best_min_charge_stop.first -
                residue_range) / m_charger_map[start_charger]->rate;
            residue_range = 0;
            strategy = m_charger_map[start_charger]->best_min_charge_stop.second.second;
        } else {
            next_charger = m_charger_map[start_charger]->best_full_charge_stop.second.first;
            charge_time = (mk_max_range - residue_range) / m_charger_map[start_charger]->rate;
            residue_range = mk_max_range -
                m_charger_map[start_charger]->best_full_charge_stop.first;
            strategy = m_charger_map[start_charger]->best_full_charge_stop.second.second;
        }
        if (next_charger.size() == 0) {
            ans += std::string("cannot not reach the destination");
            return ans;
        }
        ans += ",";
        ans += std::to_string(charge_time);
        ans += ",";
        ans += next_charger;
        start_charger = next_charger;
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