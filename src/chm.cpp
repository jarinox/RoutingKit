#include <routingkit/chm.h>


CHMGraph::CHMGraph(CHLRGraph& graph) {
    nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        nodes[i].node_index = i;
        nodes[i].rank = graph.nodes[i].rank;
        nodes[i].lat = graph.nodes[i].lat;
        nodes[i].lon = graph.nodes[i].lon;

        for (unsigned j = 0; j < graph.nodes[i].out_arcs.size(); ++j) {
            const auto& arc = graph.nodes[i].out_arcs[j];
            auto new_arc = CHMArc{i, arc.mid_node, arc.other_node, arc.weight, arc.label};
            add_arc(new_arc);
        }
    }
}

CHLRGraph CHMGraph::to_chlr() {
    CHLRGraph chlr_graph;
    chlr_graph.nodes.resize(nodes.size());

    for (unsigned i = 0; i < nodes.size(); ++i) {
        chlr_graph.nodes[i].rank = nodes[i].rank;
        chlr_graph.nodes[i].lat = nodes[i].lat;
        chlr_graph.nodes[i].lon = nodes[i].lon;

        for (const auto& arc : nodes[i].arcs) {
            chlr_graph.add_arc(arc.from, arc.mid_node, arc.to, arc.weight, arc.label);
        }

        chlr_graph.nodes[i].sort_arcs_for_weight();
    }

    return chlr_graph;
}

std::pair<unsigned, std::vector<unsigned>> CHMGraph::dijkstra(unsigned from, unsigned to, Label profile) {
    std::vector<unsigned> first_out;
    std::vector<unsigned> head;
    std::vector<unsigned> tail;
    std::vector<Label> labels;
    std::vector<unsigned> weights;

    first_out.resize(nodes.size() + 1);
    first_out[0] = 0;
    unsigned arc_count = 0;
    for (unsigned i = 0; i < nodes.size(); ++i) {
        auto arcs = nodes[i].arcs;

        std::sort(arcs.begin(), arcs.end(), [](const CHMArc& a, const CHMArc& b) {
            return a.weight < b.weight;
        });

        for (const auto& arc : arcs) {
            if(arc.is_shortcut()) continue;
            if(arc.weight >= inf_weight) continue;
            head.push_back(arc.to);
            tail.push_back(arc.from);
            labels.push_back(arc.label);
            assert(arc.weight > 0);
            weights.push_back(arc.weight);
            arc_count++;
        }
        first_out[i + 1] = arc_count;
    }

    Dijkstra dij = Dijkstra(first_out, tail, head);

    dij.add_source(from).set_labels(labels).set_profile(profile);

    while(!dij.is_finished()){
        auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
            return weights[arc];
        });
        if(settle_result.node == to){
            break;
        }
    }

    if(dij.get_distance_to(to) == 0u){
        return {inf_weight, {}};
    }

    auto path = dij.get_node_path_to(to);
    if(path.empty()){
        return {inf_weight, {}};
    }

    return {dij.get_distance_to(to), path};
}

std::pair<unsigned, unsigned> CHMGraph::calculate_weight(CHMArc arc) {
    unsigned k = inf_weight;
    arc.ref(*this).cnt = 0;
    unsigned mid_node = arc.mid_node;

    if(!arc.is_shortcut() && arc.weight < inf_weight) {
        return {arc.weight, arc.mid_node};
    }

    auto nm = Nm(arc);
    for (const auto& parents : nm) {
        if(parents.first.weight >= inf_weight || parents.second.weight >= inf_weight) continue;
        unsigned comb_weight = parents.first.weight + parents.second.weight;
        if (comb_weight < k) {
            k = comb_weight;
            mid_node = parents.first.to;
            arc.ref(*this).cnt = 1;

            assert(parents.first.to == parents.second.from);
        } else if (comb_weight == k) {
            arc.ref(*this).cnt++;
        }
    }

    return {k, mid_node};
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
        e_.cnt = 0;
        add_dominant_shortcut(e_, e);
        exists = true;
    }

    if (exists) return;

    if (increment) {
        for (auto e_ : get_dominant_shortcuts(e)) {
            auto [k, mid_node] = calculate_weight(e_);
            if (k < e.weight) {
                if (!queue.contains(e_, false)) {
                    queue.push(e_, false, *this);
                }

                auto& e_ref = e_.ref(*this);
                e_ref.weight = k;
                e_ref.mid_node = mid_node;
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
            e_ref.cnt = 0;
            add_dominant_shortcut(e, e_);
        }
    }
}

void CHMGraph::add_or_reduce_arc(CHMArc arc) {
    for (auto& existing_arc : nodes[arc.from].arcs) {
        if (existing_arc.to == arc.to) {
            if (existing_arc.weight > arc.weight) {
                if(arc.label.is_subset_of(existing_arc.label)) {
                    existing_arc.weight = arc.weight;
                    existing_arc.label = arc.label;
                    existing_arc.mid_node = arc.mid_node;
                    return;
                }
                // new arc is shorter but has more restrictions, continue search or add new arc
            } else {
                if(existing_arc.label.is_subset_of(arc.label)) {
                    // Existing arc is shorter and has fewer restrictions, do not add new shortcut
                    return;
                }
            }
        }
    }

    add_arc(arc);
}

void CHMGraph::maintenance_alt(CHMArcPos e_o, unsigned w_n, Label l_n) {
    unsigned w_o = get(e_o).weight;
    Label l_o = get(e_o).label;

    if(l_n != l_o) {
        maintenance_alt(e_o, inf_weight, l_o); // invalidate old shortcuts
        CHMArc e_n = get(e_o);
        e_n.weight = inf_weight;
        e_n.label = l_n;
        add_arc(e_n);
        maintenance_alt(e_n.get_pos(), w_n, l_n); // reduce weight of new arc and generate new shortcuts
        return;
    }

    dominant_shortcuts.clear();
    MinRankQueue queue(nodes.size() * 100 + 64);

    if(w_n > w_o) {
        queue.push(get(e_o), true, *this);
        get(e_o).weight = w_n;
    } else {
        get(e_o).weight = w_n;
        queue.push(get(e_o), false, *this);
    }

    while(!queue.empty()) {
        auto [increment, e] = queue.pop();
        if(e.weight >= inf_weight) continue;

        auto partners = Ne(e);
        std::sort(partners.begin(), partners.end(), [](const CHMArc& a, const CHMArc& b) {
            return a.weight < b.weight;
        });

        if(increment) {
            std::vector<std::pair<std::pair<unsigned, unsigned>, unsigned>> paths_to_check;
            for(const auto& in_arc : in_arcs(e.from)) {
                if(in_arc.from == e.to) continue; // Skip loops
                if(in_arc.weight >= inf_weight) continue;
                paths_to_check.push_back({{in_arc.from, e.to}, e.from});
            }   
            
            for(const auto& out_arc : nodes[e.to].arcs) {
                if(out_arc.to == e.from) continue; // Skip loops
                if(out_arc.weight >= inf_weight) continue;
                paths_to_check.push_back({{e.from, out_arc.to}, e.to});
            }


            for(const auto& [path, mid_node] : paths_to_check) {
                unsigned from_id = path.first;
                unsigned to_id = path.second;

                assert(from_id != to_id);

                // Check if a valley path between from_id and to_id exists
                for(const auto& e1 : nodes[from_id].arcs) {
                    if(e1.weight >= inf_weight) continue;
                    if(e1.to == mid_node) continue; // skip as it is contracted at the same time
                    if(e1.to == to_id) continue; // skip direct path
                    if(e1.to == from_id) continue; // skip loops

                    if(nodes[e1.to].rank >= nodes[from_id].rank) continue; // ensure valley property
                    if(nodes[e1.to].rank >= nodes[to_id].rank) continue;

                    for(const auto& e2 : nodes[e1.to].arcs) {
                        if(e2.weight >= inf_weight) continue;
                        if(e2.to != to_id) continue;

                        unsigned total_weight = e1.weight + e2.weight;
                        if(total_weight < w_n) {
                            CHMArc shortcut{from_id, e1.to, to_id, total_weight, e1.label.unite(e2.label)};
                            add_arc(shortcut); // TODO: could also reduce existing arc, but not necessary for correctness
                            queue.push(shortcut, true, *this);
                        }
                    }
                }
            }
        }

        
        for (auto e_ : partners) {
            if(e_.weight >= inf_weight) continue;

            if (increment) {
                auto e__ = Np(e_, e, true);
                auto& e__ref = e__.ref(*this);
                
                
            } else { // decrement
                auto e__ = Np(e_, e);
                if (e__.weight >= inf_weight) continue;
                if (e__.weight >= e.weight + e_.weight) {
                    add_arc(e__, true);
                    auto& e__ref = e__.ref(*this);
                    e__ref.weight = e.weight + e_.weight;
                    e__ref.mid_node = e.to == e_.from ? e.to : e.from;
                    //add_or_reduce_arc(e__ref);
                    //if (!queue.contains(e__, false)) {
                        queue.push(e__ref, false, *this);
                    //}
                }
            }
        }
    }
}

void CHMGraph::maintenance(CHMArcPos e_o, unsigned w_n, Label l_n) {
    unsigned w_o = get(e_o).weight;
    Label l_o = get(e_o).label;

    if(l_o == l_n) {
        maintenance_alt(e_o, w_n, l_n);
        return;
    }

    dominant_shortcuts.clear();
    MinRankQueue queue(nodes.size() * 100 + 64);

    if (l_n != l_o) {
        CHMArc& e = get(e_o);
        e.weight = inf_weight;
        queue.push(e, w_n > w_o, *this); // invalidate or update old shortcuts

        CHMArc e_n = get(e_o);
        e_n.weight = w_n;
        e_n.label = l_n;
        add_arc(e_n);

        queue.push(e_n, w_n > w_o, *this); // generate new shortcuts
    } else {
        get(e_o).weight = w_n;
        queue.push(get(e_o), w_n > w_o, *this);
    }

    while(!queue.empty()) {
        auto [increment, e] = queue.pop();
        //std::cout << "Processing arc: " << e.from << " -> " << e.to << " w: " << e.weight << " inc: " << increment << std::endl;
        auto partners = Ne(e);

        for (auto e_ : partners) {
            auto e__ = Np(e_, e);
            if (increment && e__.weight < inf_weight) {
                auto [k, mid_node] = calculate_weight(e__);

                if (e__.weight <= k && !queue.contains(e__, true)) {
                    add_arc(e__, true);  // ensure arc exists in graph as it might have been created by Np but not added to graph

                    auto& e__ref = e__.ref(*this);
                    e__ref.mid_node = mid_node;
                    add_arc(e__ref, true);
                    queue.push(e__ref, true, *this);
                    keep_shortcut_dominance(e__ref, true, queue);
                }
            }

            if (!increment) {
                if(e.weight >= inf_weight || e_.weight >= inf_weight) continue;
                if (e__.weight > e.weight + e_.weight) {
                    if (e__.weight > e.weight + e_.weight) {
                        auto& e__ref = e__.ref(*this);
                        e__ref.weight = e.weight + e_.weight;
                        e__ref.mid_node = e.to == e_.from ? e.to : e.from;
                        add_arc(e__ref, true);
                        if (!queue.contains(e__ref, false)) {
                            queue.push(e__ref, false, *this);
                            keep_shortcut_dominance(e__ref, false, queue);
                        }
                    }
                }
            }
        }
    }
}

unsigned CHMGraph::w(unsigned from, unsigned to, Label label) {
    unsigned k = inf_weight;

    for (const auto& arc : nodes[from].arcs) {
        if (arc.to != to) continue;
        if (!arc.label.is_subset_of(label)) continue;
        if (k < arc.weight) continue;
        k = arc.weight;
    }

    return k;
}

void CHMGraph::defragment() {
    CHMGraph new_graph;
    new_graph.nodes.resize(nodes.size());

    for (unsigned i = 0; i < nodes.size(); ++i) {
        new_graph.nodes[i].node_index = i;
        new_graph.nodes[i].lat = nodes[i].lat;
        new_graph.nodes[i].lon = nodes[i].lon;
        new_graph.nodes[i].rank = nodes[i].rank;

        for(const CHMArc& arc : nodes[i].arcs) {
            if (arc.weight >= inf_weight || arc.label == Label::fully_restricted()) continue;
            new_graph.add_arc(arc);
        }

        std::sort(new_graph.nodes[i].arcs.begin(), new_graph.nodes[i].arcs.end(), [](const CHMArc& a, const CHMArc& b) {
            return a.weight < b.weight;
        });
    }

    std::swap(nodes, new_graph.nodes);
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

CHMArc CHMGraph::Np(CHMArc e1, CHMArc e2, bool with_mid_node_check) {
    if (e1.to != e2.from) {
        std::swap(e1, e2);
    }

    assert(e1.to == e2.from);

    unsigned from_rank = nodes[e1.from].rank;
    unsigned to_rank = nodes[e2.to].rank;
    unsigned mid_rank = nodes[e1.to].rank;

    assert(from_rank > mid_rank && to_rank > mid_rank);

    for (CHMArc arc : nodes[e1.from].arcs) {
        if(arc.to != e2.to) continue;
        if(with_mid_node_check && arc.mid_node != e1.to) continue;
        if(arc.label != e1.label.unite(e2.label)) continue;

        return arc;
    }

    return CHMArc{e1.from, e1.to, e2.to, e1.weight + e2.weight, e1.label.unite(e2.label)};
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

void CHMGraph::add_arc(CHMArc arc, bool avoid_duplicated) {
    if(avoid_duplicated) {
        for (const auto& existing_arc : nodes[arc.from].arcs) {
            if (existing_arc == arc) {
                return; // Arc already exists, avoid duplication
            }
        }
    }

    nodes[arc.from].arcs.push_back(arc);
    nodes[arc.from].arcs.back().arc_index = nodes[arc.from].arcs.size() - 1;
}

void CHMGraph::add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    CHMArc arc{from, mid_node, to, weight, label};
    add_arc(arc);
}

unsigned CHMGraph::witness_search(unsigned from, unsigned to, unsigned weight, Label label) {
    MinIDQueue queue(nodes.size());
    std::vector<unsigned> distances(nodes.size(), inf_weight);
    std::vector<bool> visited(nodes.size(), false);
    queue.push({from, 0});
    distances[from] = 0;

    while (!queue.empty()) {
        auto [current, dist] = queue.pop();
        visited[current] = true;

        if(distances[current] > weight) {
            break;
        }

        if (current == to) {
            return dist;
        }

        for (const auto& arc : nodes[current].arcs) {
            if(visited[arc.to]) continue;

            if (dist + arc.weight < distances[arc.to]) {
                if (!arc.label.is_subset_of(label)) continue;
                if (arc.weight >= inf_weight) continue;

                distances[arc.to] = dist + arc.weight;

                if (queue.contains_id(arc.to)) {
                    queue.decrease_key({arc.to, distances[arc.to]});
                } else {
                    queue.push({arc.to, distances[arc.to]});
                }                
            }
        }
    }

    return inf_weight;
}


std::vector<CHMArc> CHMGraph::SCSp(CHMArc e) {
    std::vector<CHMArc> result;

    // Collect relevant arcs
    for(auto node : nodes) {
        for (auto arc : node.arcs) {
            if(arc.to == e.from || arc.from == e.from || arc.to == e.to || arc.from == e.to) {
                if (!arc.is_shortcut()) continue;
                if (arc.weight >= inf_weight) continue;
                
                result.push_back(arc);
            }
        }
    }

    return result;
}


unsigned CHMGraph::cal_sc_weight(unsigned from, unsigned mid, unsigned to) {
    unsigned w_e1 = inf_weight;
    unsigned w_e2 = inf_weight;

    for (const auto& arc : nodes[from].arcs) {
        if (arc.to == mid) {
            if(arc.weight < w_e1) {
                w_e1 = arc.weight;
            }
        }
    }

    for (const auto& arc : nodes[mid].arcs) {
        if (arc.to == to) {
            if(arc.weight < w_e2) {
                w_e2 = arc.weight;
            }
        }
    }

    return w_e1 + w_e2;
}


std::vector<CHMArc> CHMGraph::in_arcs(unsigned node) {
    std::vector<CHMArc> result;

    for (const auto& n : nodes) {
        for (const auto& arc : n.arcs) {
            if (arc.to == node) {
                result.push_back(arc);
            }
        }
    }

    return result;
}
