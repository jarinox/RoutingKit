#ifndef ROUTING_KIT_CHLRM_H
#define ROUTING_KIT_CHLRM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>

#include <vector>
#include <unordered_map>
#include <functional>
#include <set>
#include <memory>


class CHLRMArcPos {
public:
    CHLRMArcPos() : arc_index(0), node_index(0) {}; // Default constructor
    CHLRMArcPos(unsigned arc_index, unsigned node_index) 
        : arc_index(arc_index), node_index(node_index) {};
    
    unsigned arc_index;
    unsigned node_index;

    bool operator==(const CHLRMArcPos& other) const {
        return arc_index == other.arc_index && node_index == other.node_index;
    }
};

// Hash specializations must be defined before use
namespace std {
    template <>
    struct hash<CHLRMArcPos> {
        std::size_t operator()(const CHLRMArcPos& pos) const noexcept {
            return std::hash<unsigned>()(pos.node_index) ^ (std::hash<unsigned>()(pos.arc_index) << 1);
        }
    };

    template <>
    struct hash<std::pair<CHLRMArcPos, CHLRMArcPos>> {
        std::size_t operator()(const std::pair<CHLRMArcPos, CHLRMArcPos>& p) const noexcept {
            std::size_t h1 = std::hash<CHLRMArcPos>{}(p.first);
            std::size_t h2 = std::hash<CHLRMArcPos>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };
}

// Forward declaration to fix unknown type error
class CHLRMGraph;

class CHLRMArc {
public:
    unsigned from;
    unsigned mid_node;
    unsigned to;

    unsigned cnt;

    unsigned weight;
    Label label;

    std::vector<CHLRMArc&> dominant_shortcut_set;

    CHLRMArc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) :
        from(from), mid_node(mid_node), to(to), cnt(0), weight(weight), label(label) {};

    bool is_shortcut() { return mid_node != invalid_id; }
    unsigned rank(CHLRMGraph& graph);

    CHLRArc to_chlr() {
        return CHLRArc{to, mid_node, weight, label};
    }

    CHLRMArcPos get_pos(CHLRMGraph& graph) {
        unsigned arc_index = 0;
        for (const auto& arc : graph.nodes[from].arcs) {
            if (arc.from == from && arc.mid_node == mid_node && arc.to == to && arc.weight == weight && arc.label == label) {
                return CHLRMArcPos(arc_index, from);
            }
            ++arc_index;
        }

        return CHLRMArcPos(inf_weight, inf_weight);
    }
};

class CHLRMNode {
public:
    unsigned index;
    unsigned rank;

    float lat;
    float lon;

    std::vector<CHLRMArc> arcs;

    CHLRNode to_chlr() {
        CHLRNode node;
        node.rank = rank;
        node.lat = lat;
        node.lon = lon;
        node.out_arcs.reserve(arcs.size());
        
        for (auto& arc : arcs) {
            node.out_arcs.push_back(arc.to_chlr());
        }
        
        return node;
    }
};

class CHLRMGraph {
public:
    std::vector<CHLRMNode> nodes;

    // N+ maps the parent shortcuts e1 and e2 to their child shortcut e3
    std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos> N_plus;

    // N- maps the child shortcut e3 to its parent shortcuts e1 and e2
    std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>> N_minus;

    // N= maps a parent shortcut e1 to its partner shortcut e2
    std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>> N_equals;

    CHLRMGraph(CHLRGraph& graph);

    CHLRGraph to_chlr();

    CHLRMArc& get_arc(CHLRMArcPos arc_pos);
    void delete_arc(CHLRMArcPos arc_pos);

    unsigned calculate_weight(CHLRMArc& arc);
    void CHLRMGraph::keep_shortcut_dominance(CHLRMArc& arc, MinRankQueue& queue, bool increment);
    void CHLRMGraph::maintenance(CHLRMArc& e_o, unsigned w_n, Label l_n);
    void maintenance_optimized(CHLRMArcPos original_arc_pos, unsigned new_weight, Label new_label);
    unsigned arc_count() const {
        unsigned count = 0;
        for (const auto& node : nodes) {
            count += node.arcs.size();
        }
        return count;
    }

private:
    void build_neighbour_index();
};


class MinRankQueue {
    MinIDQueue queue;
    std::vector<std::pair<bool, CHLRMArc&>> unsigned_to_arc;
    std::set<std::pair<CHLRMArc, bool> > is_in_queue;
public:
    MinRankQueue(unsigned size) : queue(size), unsigned_to_arc(size) {}

    bool empty() const {
        return queue.empty();
    }

    void push(CHLRMArc& arc, bool increment, CHLRMGraph& graph) {
        queue.push({arc.rank(graph), static_cast<unsigned>(unsigned_to_arc.size())});
        is_in_queue.insert({arc, increment});
        unsigned_to_arc.push_back({increment, arc});
    }

    std::pair<bool, CHLRMArc&> pop() {
        auto pair = queue.pop();
        is_in_queue.erase({unsigned_to_arc[pair.key].second, unsigned_to_arc[pair.key].first});
        return unsigned_to_arc[pair.key];
    }

    bool contains(const CHLRMArc& arc, bool increment) const {
        return is_in_queue.find({arc, increment}) != is_in_queue.end();
    }
};

#endif // ROUTING_KIT_CHLRM_H
