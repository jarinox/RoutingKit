#ifndef ROUTING_KIT_CHLRM_H
#define ROUTING_KIT_CHLRM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>

#include <vector>
#include <unordered_map>
#include <functional>
#include <set>
#include <memory>
#include <queue>


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

    bool operator<(const CHLRMArcPos& other) const {
        return node_index < other.node_index || 
               (node_index == other.node_index && arc_index < other.arc_index);
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

    std::vector<CHLRMArc> dominant_shortcut_set;

    CHLRMArc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) :
        from(from), mid_node(mid_node), to(to), cnt(0), weight(weight), label(label) {};

    bool is_shortcut() { return mid_node != invalid_id; }
    unsigned rank(CHLRMGraph& graph);

    CHLRArc to_chlr() {
        return CHLRArc{to, mid_node, weight, label};
    }

    CHLRMArcPos get_pos(CHLRMGraph& graph);

    bool operator<(const CHLRMArc& other) const {
        return from < other.from ||
               (from == other.from && (mid_node < other.mid_node ||
               (mid_node == other.mid_node && (to < other.to ||
               (to == other.to && (weight < other.weight || 
               (weight == other.weight && label < other.label)))))));
    }

    bool operator==(const CHLRMArc& other) const {
        return from == other.from && mid_node == other.mid_node && to == other.to &&
               weight == other.weight && label == other.label;
    }
};

class MinRankQueue {
    MinIDQueue queue;
    std::vector<std::queue<std::pair<bool, CHLRMArc>>> unsigned_to_arc;
    std::set<std::pair<CHLRMArc, bool>> is_in_queue;
public:
    MinRankQueue(unsigned size) : queue(size), unsigned_to_arc(size) {}

    bool empty() const {
        return queue.empty();
    }

    void push(CHLRMArc arc, bool increment, CHLRMGraph& graph);
    std::pair<bool, CHLRMArc> pop();

    bool contains(const CHLRMArc arc, bool increment) const {
        return is_in_queue.find({arc, increment}) != is_in_queue.end();
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
private:
    CHLRMArc dummy_arc{invalid_id, invalid_id, invalid_id, inf_weight, Label()};
public:
    std::vector<CHLRMNode> nodes;

    // N+ maps the parent shortcuts e1 and e2 to their child shortcut e3
    std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos> N_plus;

    // N- maps the child shortcut e3 to its parent shortcuts e1 and e2
    std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>> N_minus;

    // N= maps a parent shortcut e1 to its partner shortcut e2
    std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>> N_equals;

    std::map<CHLRMArcPos, unsigned> weight;

    CHLRMGraph(CHLRGraph& graph);

    CHLRGraph to_chlr();

    void add_arc(CHLRMArc arc, bool check_duplicate = false) {
        if (check_duplicate) {
            for (const auto& existing_arc : nodes[arc.from].arcs) {
                if (existing_arc == arc) {
                    return; // Arc already exists, do not add
                }
            }
        }

        nodes[arc.from].arcs.push_back(arc);
    }

    CHLRMArc& find_arc(CHLRMArc& arc) {
        for (auto& node : nodes) {
            for (auto& existing_arc : node.arcs) {
                if (existing_arc == arc) {
                    return existing_arc;
                }
            }
        }
        return arc; // Return the original arc if not found
    }

    CHLRMArc& get_arc(CHLRMArcPos arc_pos);
    void delete_arc(CHLRMArcPos arc_pos);

    unsigned calculate_weight(CHLRMArc& arc);
    void keep_shortcut_dominance(CHLRMArc& arc, MinRankQueue& queue, bool increment);
    void maintenance(CHLRMArcPos e_o_pos, unsigned w_n, Label l_n);
    void maintenance_optimized(CHLRMArcPos original_arc_pos, unsigned new_weight, Label new_label);
    unsigned arc_count() const {
        unsigned count = 0;
        for (const auto& node : nodes) {
            count += node.arcs.size();
        }
        return count;
    }

    unsigned original_arc_weight(CHLRMArc& arc);

    std::vector<std::pair<CHLRMArc&, CHLRMArc&>> Nm(CHLRMArc& arc) {
        std::vector<std::pair<CHLRMArc&, CHLRMArc&>> result;
        //assert(arc.is_shortcut());

        for(CHLRMArc& e1 : nodes[arc.from].arcs) {
            if(e1.to == arc.to) continue;
            unsigned rank_mid = nodes[e1.to].rank;
            if(rank_mid >= nodes[arc.from].rank) continue;
            if(rank_mid >= nodes[arc.to].rank) continue;

            for(CHLRMArc& e2 : nodes[e1.to].arcs) {
                if(e2.to != arc.to) continue;
                if(e1.label.unite(e2.label) != arc.label) continue;
                
                result.emplace_back(e1, e2);
            }
        }

        return result;
    }   

    CHLRMArc& Np(CHLRMArc& e1, CHLRMArc& e2) {
        assert(e1.to == e2.from);

        for(CHLRMArc& arc : nodes[e1.from].arcs) {
            if(!arc.is_shortcut()) continue;
            if(arc.to != e2.to) continue;
            if(arc.label != e1.label.unite(e2.label)) continue;

            unsigned from_rank = nodes[e1.from].rank;
            unsigned to_rank = nodes[e2.to].rank;
            unsigned mid_rank = nodes[e1.to].rank;

            if(from_rank <= mid_rank || to_rank <= mid_rank) continue;

            return arc;
        }

        //assert(false && "No parent arc found for the given child arcs.");
        return dummy_arc;
    }

    bool have_child(CHLRMArc& e1, CHLRMArc& e2) {
        assert(e1.to == e2.from);

        for(CHLRMArc& arc : nodes[e1.from].arcs) {
            if(arc.to != e2.to) continue;
            if(arc.label != e1.label.unite(e2.label)) continue;
            if(nodes[arc.from].rank <= nodes[e1.to].rank) continue;
            if(nodes[arc.to].rank <= nodes[e2.from].rank) continue;

            return true;
        }
        return false;
    }

    std::vector<CHLRMArc*> Ne(CHLRMArc& e2) {
        std::vector<CHLRMArc*> result;

        for(CHLRMNode& node : nodes) {
            for(CHLRMArc& e1 : node.arcs) {
                if(e1.to != e2.from) continue;
                if(!have_child(e1, e2)) continue;

                result.push_back(&e1);
            }
        }

        return result;
    }

private:
    void build_neighbour_index();
};



#endif // ROUTING_KIT_CHLRM_H
