#include "network.h"
#include "geo_utils/SphericalUtil.h"

#include <string>
#include <queue>
#include <map>
#include <memory>
#include <float.h>
#include <stack>

class Solution {
    public:
    Solution(std::string start_charger, std::string end_charger);
    void run();

    private:
    void init_src_and_dst_(double& direction);
    void init_candidates_map_(const double& direction);
    void search_best_route(std::string start_charger);
    bool is_direction_good(LatLng& start_point, LatLng& cur_point, double direction);
    void print_path(std::string charger);
    struct Node {
        LatLng point;
        double rate;
        double best_time;
        bool visited;
        // std::stack<std::pair<double, std::string>> best_solution;
        std::pair<double, std::string> best_stop;

        Node(double lat_, double lon_, double rate_, double best_time_, bool visited_ = false) :
            point(lat_, lon_),
            rate(rate_),
            best_time(best_time_),
            visited(visited_) {}

        // Node(const Node& node_) = default;
        // Node& operator=(const Node& other) = default;
    };
    std::map<std::string, std::shared_ptr<Node>> m_charger_map;
    double m_max_time{DBL_MAX};
    std::string m_start_charger, m_end_charger;
    LatLng m_start_point, m_end_point;
};
