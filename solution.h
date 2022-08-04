#include "network.h"
#include "geo_utils/SphericalUtil.h"

#include <string>
#include <queue>
#include <map>
#include <memory>
#include <float.h>
#include <stack>

// at each charger, we either charge to full or charge just enough to next charger
enum class Strategy{FULL, MIN};
// Signpost = {distance between A and B, {B name, B charging strategy}}
using Signpost = std::pair<double, std::pair<std::string, Strategy>>;
class Solution {
    public:
    Solution(std::string start_charger, std::string end_charger);
    void run();

    private:
    static constexpr double mk_max_time{DBL_MAX};
	static constexpr double mk_max_range{320};
    double init_src_and_dst_();
    void init_candidates_map_(const double direction);
    void search_best_route_(std::string start_charger);
    bool is_direction_good_(LatLng& start_point, LatLng& cur_point, double direction);
	std::string refine_path_(std::string charger);
    struct Node {
        bool visited;
        LatLng point;
        double rate;
        double best_full_charge_total_time;
        Signpost best_full_charge_stop;
        double best_min_charge_total_time;
        Signpost best_min_charge_stop;

        Node(double lat_, double lon_, double rate_,
            double best_full_charge_total_time_ = mk_max_time,
            double best_min_charge_total_time_ = mk_max_time,
            bool visited_ = false) :
            point(lat_, lon_),
            rate(rate_),
            best_full_charge_total_time(best_full_charge_total_time_),
            best_min_charge_total_time(best_min_charge_total_time_),
            visited(visited_) {}
    };

	std::map<std::string, std::shared_ptr<Node>> m_charger_map;
    std::string m_start_charger, m_end_charger;
    LatLng m_start_point, m_end_point;
};
