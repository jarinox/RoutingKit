#include <routingkit/chm.h>

unsigned CHMGraph::calculate_weight(CHMArc arc) {
    unsigned k = inf_weight;
    if(!arc.is_shortcut() && arc.weight < inf_weight) {
        k = arc.weight;
    }

    auto nm = Nm(arc);
    for (const auto& parents : nm) {
        unsigned comb_weight = parents.first.weight + parents.second.weight;
        if (comb_weight < k) {
            k = comb_weight;
        }
    }

    return k;
}

std::vector<std::pair<CHMArc, CHMArc>> CHMGraph::Nm(CHMArc child) {
    std::vector<std::pair<CHMArc, CHMArc>> result;

    for(const CHMArc& e1 : nodes[child.from].arcs) {
        if(e1.to == child.to) continue; // Skip if parallel arc

        unsigned rank_mid = nodes[e1.to].rank;
        if(rank_mid >= nodes[child.from].rank) continue;
        if(rank_mid >= nodes[child.to].rank) continue;

        for(const CHMArc& e2 : nodes[e1.to].arcs) {
            if(e2.to != child.to) continue;
            if(e1.label.unite(e2.label) != child.label) continue;

            result.emplace_back(e1.get_pos(), e2.get_pos());
        }
    }

    return result;
}

std::vector<CHMArc> CHMGraph::Ne(CHMArc arc) {
    std::vector<CHMArc> result;

    for(const CHMNode& node : nodes) {
        for(const CHMArc& other : node.arcs) {
            if(other.to == arc.from && other.from != arc.to) {
                if(nodes[other.from].rank > nodes[other.to].rank
                && nodes[arc.to].rank > nodes[other.to].rank) {
                    result.push_back(other);
                }
            }

            if(arc.to == other.from && arc.from != other.to) {
                if(nodes[other.to].rank > nodes[other.from].rank
                && nodes[arc.from].rank > nodes[other.from].rank) {
                    result.push_back(other);
                }
            }
        }
    }

    return result;
}

CHMArc CHMGraph::Np(CHMArc e1, CHMArc e2) {
    if (e1.to == e2.from) {
        std::swap(e1, e2);
    }

    assert(e1.to == e2.from);

    for (CHMArc& arc : nodes[e1.from].arcs) {
        if(arc.to != e2.to) continue;
        if(arc.label != e1.label.unite(e2.label)) continue;

        unsigned from_rank = nodes[e1.from].rank;
        unsigned to_rank = nodes[e2.to].rank;
        unsigned mid_rank = nodes[e1.to].rank;

        if(from_rank <= mid_rank || to_rank <= mid_rank) continue;

        return arc;
    }

    return CHMArc{e1.from, e1.to, e2.to, inf_weight, e1.label.unite(e2.label)};
}
