#ifndef ROUTINGKIT_DCH_H
#define ROUTINGKIT_DCH_H

#include <routingkit/chlr.h>
#include <routingkit/chm.h>
#include <routingkit/label.h>

class DCHGraph {
public:
    std::vector<CHMNode> nodes;
    
    DCHGraph() = default;
    DCHGraph(CHLRGraph& graph);
    CHLRGraph to_chlr();

    void add_arc(CHMArc arc);
    void add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label);
    CHMArc& get(CHMArcPos pos);
    CHMArc& ref(CHMArc arc);

    std::vector<std::pair<CHMArc, CHMArc>> SCPPlus(CHMArc p1);

    void DCHPlus(CHMArcPos e_o_pos, unsigned w_n);
    void DCHMinus(CHMArcPos e_o_pos, unsigned w_n);

    std::pair<unsigned, std::vector<unsigned>> dijkstra(unsigned from, unsigned to, Label profile);
};


class DCHQueue {
    MinIDQueue queue;
    std::vector<std::queue<CHMArc>> unsigned_to_arc;
    std::set<CHMArc> is_in_queue; // another option would be to uniquely identify by CHMArcPos
public:
    DCHQueue(unsigned size) : queue(size), unsigned_to_arc(size) {}

    bool empty() const {
        return queue.empty();
    }

    void push(CHMArc arc, DCHGraph& graph);
    CHMArc pop();

    bool contains(const CHMArc& arc) const {
        return is_in_queue.find(arc) != is_in_queue.end();
    }
};


#endif
