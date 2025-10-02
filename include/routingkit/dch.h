#ifndef ROUTINGKIT_DCH_H
#define ROUTINGKIT_DCH_H

#include <routingkit/chlr.h>
#include <routingkit/chm.h>
#include <routingkit/label.h>

struct DCHArcPos {
    unsigned node_index;
    unsigned arc_index;
};

class DCHGraph {
public:
    std::vector<CHMNode> nodes;
    std::vector<std::vector<DCHArcPos>> garbage;
    std::vector<std::vector<DCHArcPos>> backward_garbage;
    
    DCHGraph() = default;
    DCHGraph(CHLRGraph& graph);
    CHLRGraph to_chlr();

    CHMArc add_arc(CHMArc arc);
    void add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label);
    CHMArc& get(CHMArcPos pos);
    CHMArc& ref(CHMArc arc);

    std::vector<std::pair<CHMArc, CHMArc>> SCPPlus(CHMArc p1);
    std::vector<std::pair<CHMArc, CHMArc>> SCPMinus(CHMArc child);
    unsigned compute_weight(CHMArc arc);
    Label compute_label(CHMArc arc);

    void DCHPlus(CHMArcPos e_o_pos, unsigned w_n);
    void DCHMinus(CHMArcPos e_o_pos, unsigned w_n);
    void DCHLabel(CHMArcPos e_o_pos, Label l_n);

    void DCHAlt(CHMArcPos e_o_pos, unsigned w_n);

    void DCH(CHMArcPos e_o_pos, unsigned weight, Label l_n);

    std::pair<unsigned, std::vector<unsigned>> dijkstra(unsigned from, unsigned to, Label profile);

    std::vector<CHMArc> Ne(CHMArc arc);
    CHMArc Np(CHMArc p1, CHMArc p2);

    void update(CHMArc arc, unsigned weight, Label label);
    void invalidate(CHMArc arc);

    std::vector<CHMArc> in_arcs(unsigned node) {
        return nodes[node].in_arcs;
    }
};

struct DCHQueueEntry {
    CHMArc arc;
    CHMArc p1;
    CHMArc p2;
    bool increment; // true for increment, false for decrement
};

class DCHQueue {
    MinIDQueue queue;
    std::vector<std::queue<DCHQueueEntry>> unsigned_to_arc;
    std::set<CHMArc> is_in_queue; // another option would be to uniquely identify by CHMArcPos
public:
    DCHQueue(unsigned size) : queue(size), unsigned_to_arc(size) {}

    bool empty() const {
        return queue.empty();
    }

    void push(DCHQueueEntry entry, DCHGraph& graph);
    DCHQueueEntry pop();

    bool contains(const CHMArc& arc) const {
        return is_in_queue.find(arc) != is_in_queue.end();
    }
};


#endif
