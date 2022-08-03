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
    double init_src_and_dst_();
    void init_candidates_map_(const double direction);
    void search_best_route_(std::string start_charger);
    bool is_direction_good_(LatLng& start_point, LatLng& cur_point, double direction);
	std::string refine_path_(std::string charger);
    struct Node {
        LatLng point;
        double rate;
        double best_time;
        bool visited;
        std::pair<double, std::string> best_stop;

        Node(double lat_, double lon_, double rate_, double best_time_, bool visited_ = false) :
            point(lat_, lon_),
            rate(rate_),
            best_time(best_time_),
            visited(visited_) {}

    };
    
    static constexpr double mk_max_time{DBL_MAX};
	static constexpr double mk_max_range{320};
	std::map<std::string, std::shared_ptr<Node>> m_charger_map;
    std::string m_start_charger, m_end_charger;
    LatLng m_start_point, m_end_point;
};
