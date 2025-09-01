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

void CHMGraph::keep_shortcut_dominance(CHMArc arc, bool increment, MinRankQueue& queue) {
    CHMArc& e = arc.ref(*this);

    bool exists = false;
    for (auto _e_ : nodes[e.from].arcs) {
        if (_e_.to != e.to) continue;
        if (!_e_.label.is_subset_of(e.label)) continue;
        if (_e_.weight > e.weight) continue;
        if(_e_ == e) continue;

        auto& e_ = _e_.ref(*this);
        e_.weight = inf_weight;
        add_dominant_shortcut(e_, e);
        exists = true;
    }

    if (exists) return;

    if (increment) {
        for (auto e_ : get_dominant_shortcuts(e)) {
            unsigned k = calculate_weight(e_);
            if (k < e.weight) {
                if (!queue.contains(e_, false)) {
                    queue.push(e_, false, *this);
                }

                auto& e_ref = e_.ref(*this);
                e_ref.weight = k;
                remove_dominant_shortcut(e, e_);
                add_arc(e_ref, true);
            }
        }
    } else {
        for (auto e_ : nodes[e.from].arcs) {
            if (e_.to != e.to) continue;
            if (!e.label.is_subset_of(e_.label)) continue;
            if (e_.weight < e.weight) continue;
            if (e_ == e) continue;
            
            if (!queue.contains(e_, true)) {
                queue.push(e_, true, *this);
            }

            auto& e_ref = e_.ref(*this);
            e_ref.weight = inf_weight;
            add_dominant_shortcut(e, e_);
        }
    }
}

void CHMGraph::maintenance(CHMArcPos e_o_pos, unsigned w_n, Label l_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;
    Label l_o = e_o.label;

    dominant_shortcuts.clear();
    MinRankQueue queue(nodes.size() * 100 + 64);

    if (l_n != l_o) {
        e_o.weight = inf_weight;
        CHMArc e_n = CHMArc{e_o.from, e_o.mid_node, e_o.to, w_n, l_n};
        add_arc(e_n, true);

        e_o = get(e_o_pos);
        e_o.weight = calculate_weight(e_o);

        
    }
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

            result.emplace_back(e1, e2);
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

void CHMGraph::add_dominant_shortcut(CHMArc arc, CHMArc shortcut) {
    if(dominant_shortcuts.find(arc) == dominant_shortcuts.end()) {
        dominant_shortcuts[arc] = std::vector<CHMArc>{shortcut};
    } else if(!has_dominant_shortcut(arc, shortcut)) {
        dominant_shortcuts[arc].push_back(shortcut);
    }
}

bool CHMGraph::has_dominant_shortcut(CHMArc arc, CHMArc shortcut) {
    if(dominant_shortcuts.find(arc) == dominant_shortcuts.end()) {
        return false;
    }
    const auto& shortcuts = dominant_shortcuts[arc];
    return std::find(shortcuts.begin(), shortcuts.end(), shortcut) != shortcuts.end();
}

std::vector<CHMArc> CHMGraph::get_dominant_shortcuts(CHMArc arc) {
    if(dominant_shortcuts.find(arc) == dominant_shortcuts.end()) {
        return std::vector<CHMArc>{};
    }
    return dominant_shortcuts[arc];
}

void CHMGraph::remove_dominant_shortcut(CHMArc arc, CHMArc shortcut) {
    if(dominant_shortcuts.find(arc) == dominant_shortcuts.end()) {
        return;
    }
    auto& shortcuts = dominant_shortcuts[arc];
    shortcuts.erase(std::remove(shortcuts.begin(), shortcuts.end(), shortcut), shortcuts.end());
    if(shortcuts.empty()) {
        dominant_shortcuts.erase(arc);
    }
}

unsigned CHMArc::rank(CHMGraph& graph) {
    unsigned from_rank = graph.nodes[from].rank;
    unsigned to_rank = graph.nodes[to].rank;
    return std::min(from_rank, to_rank);
}

void MinRankQueue::push(CHMArc arc, bool increment, CHMGraph& graph) {
    unsigned arc_priority = arc.rank(graph)*100+__builtin_popcount(arc.label.get_label());

    if(queue.contains_id(arc_priority)) {
        if(is_in_queue.find({arc, increment}) != is_in_queue.end()) {
            return;
        }
        unsigned_to_arc[queue.get_key(arc_priority)].push({increment, arc});
    } else {
        queue.push({arc_priority, static_cast<unsigned>(unsigned_to_arc.size())});
        unsigned_to_arc.push_back(std::queue<std::pair<bool, CHMArc>>{});
        unsigned_to_arc.back().push({increment, arc});
    }

    is_in_queue.insert({arc, increment});
}

std::pair<bool, CHMArc> MinRankQueue::pop() {
    auto vec = queue.peek();

    auto pair = unsigned_to_arc[vec.key].front();
    unsigned_to_arc[vec.key].pop();

    is_in_queue.erase({pair.second, pair.first});

    if(unsigned_to_arc[vec.key].empty()) {
        queue.pop();
    }

    return pair;
}
