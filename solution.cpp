#include "solution.h"

#include <iomanip>
#include <iostream>
#include <cmath>


Solution::Solution(std::string start_charger, std::string end_charger) :
    m_start_charger(start_charger),
    m_end_charger(end_charger) {

    init_src_and_dst_();
    init_candidates_map();
}


void Solution::init_src_and_dst_() {
    for (auto& charger : network) {
        if (charger.name == m_start_charger) {
            m_start_lat = charger.lat;
            m_start_lon = charger.lon;
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 0));
        } else if (charger.name == m_end_charger) {
            m_end_lat = charger.lat;
            m_end_lon = charger.lon;
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, 0));
        }
    }

    m_max_time = SphericalUtil::computeDistanceBetween({m_start_lat, m_start_lon},
            {m_end_lat, m_end_lon}) / m_charger_map[m_start_charger]->rate / 1000;
    m_charger_map[m_start_charger]->straight_time = m_max_time;
    std::cout << "initial speed: " << m_charger_map[m_start_charger]->rate
        << " | " << "initial max time: " << m_max_time << std::endl;
}


void Solution::init_candidates_map() {
    for (auto& charger : network) {
        auto dst_time = 
            SphericalUtil::computeDistanceBetween({m_end_lat, m_end_lon},
                {charger.lat, charger.lon}) / charger.rate / 1000;
        auto src_time = 
            SphericalUtil::computeDistanceBetween({m_start_lat, m_start_lon},
                {charger.lat, charger.lon}) / charger.rate / 1000;
        if (dst_time + src_time < m_max_time) {
            m_charger_map[charger.name] = std::make_shared<Node>(
                Node(charger.lat, charger.lon, charger.rate, dst_time));
            std::cout << "transfer from " << charger.name
                << ":\t" << src_time << "\t" << dst_time
                << " total:\t" << (src_time + dst_time) << std::endl;
        }
    }
}


void Solution::run() {
    std::cout << m_charger_map.size();
    return;
}