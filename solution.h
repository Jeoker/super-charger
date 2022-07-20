#include <string>
#include <queue>
#include "network.h"

class Solution {
    public:
    Solution();
    void run(std::string start_charger, std::string end_charger);

    private:
    struct Node {
        std::string name;
        double north;
        double east;
    };
    struct Edge {
        std::string name;
        double north;
        double east;
    };
};
