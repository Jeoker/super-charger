#include "network.h"
#include "geo_utils/SphericalUtil.h"

#include <string>
#include <queue>
#include <map>
#include <memory>
#include <float.h>

class Solution {
    public:
    Solution(std::string start_charger, std::string end_charger);
    void run();

    private:
    void init_src_and_dst_();
    void init_candidates_map();
    struct Node {
        LatLng point;
        double rate;
        bool visited;

        Node(double lat_, double lon_, double rate_, bool visited_ = false) :
            point(lat_, lon_),
            rate(rate_),
            visited(visited_) {}

        // Node(const Node& node_) = default;
        // Node& operator=(const Node& other) = default;
    };
    struct Edge {
        std::string name;
        double north;
        double east;
    };
    std::map<std::string, std::shared_ptr<Node>> m_charger_map;
    double m_max_time{DBL_MAX};
    std::string m_start_charger, m_end_charger;
    double m_start_lat, m_start_lon;
    double m_end_lat, m_end_lon;
};
