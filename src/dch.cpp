#include <routingkit/dch.h>

CHLRGraph DCHGraph::to_chlr() {
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

DCHGraph::DCHGraph(CHLRGraph& graph) {
    nodes.resize(graph.nodes.size());
    garbage.resize(graph.nodes.size());
    backward_garbage.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        nodes[i].node_index = i;
        nodes[i].rank = graph.nodes[i].rank;
        nodes[i].lat = graph.nodes[i].lat;
        nodes[i].lon = graph.nodes[i].lon;

        for (unsigned j = 0; j < graph.nodes[i].out_arcs.size(); ++j) {
            const auto& arc = graph.nodes[i].out_arcs[j];
            add_arc(i, arc.mid_node, arc.other_node, arc.weight, arc.label);
        }
    }
}

CHMArc DCHGraph::add_arc(CHMArc arc) {
    arc.in_graph = true;
    CHMArc forward = arc;

    if(!garbage[arc.from].empty()) {
        auto pos = garbage[arc.from].back();
        assert(pos.node_index == arc.from);
        garbage[arc.from].pop_back();

        forward.arc_index = pos.arc_index; // set forward arc
        nodes[arc.from].arcs[pos.arc_index] = forward;
    } else {
        forward.arc_index = nodes[arc.from].arcs.size();
        nodes[arc.from].arcs.push_back(forward);
    }

    if(!backward_garbage[arc.to].empty()) {
        auto pos = backward_garbage[arc.to].back();
        assert(pos.node_index == arc.to);
        backward_garbage[arc.to].pop_back();

        nodes[arc.from].arcs[forward.arc_index].twin = {pos.node_index, pos.arc_index};
        arc.twin = {arc.from, forward.arc_index};
        arc.arc_index = pos.arc_index;
        nodes[arc.to].in_arcs[pos.arc_index] = arc;
    } else {
        arc.arc_index = nodes[arc.to].in_arcs.size();
        nodes[arc.from].arcs[forward.arc_index].twin = {arc.to, arc.arc_index};
        arc.twin = {arc.from, forward.arc_index};
        nodes[arc.to].in_arcs.push_back(arc);
    }


    return forward;
}

void DCHGraph::update(CHMArc arc, unsigned weight, Label label) {
    CHMArcPos pos = {arc.from, arc.arc_index};
    CHMArcPos tpos = arc.twin;

    CHMArc& orig = get(pos);
    CHMArc& twin = nodes[tpos.node_index].in_arcs[tpos.arc_index];

    orig.weight = weight;
    orig.label = label;
    twin.weight = weight;
    twin.label = label;
}

void DCHGraph::add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    add_arc(CHMArc{from, mid_node, to, weight, label});
}

CHMArc& DCHGraph::get(CHMArcPos pos) {
    return nodes[pos.node_index].arcs[pos.arc_index];
}

CHMArc& DCHGraph::ref(CHMArc arc) {
    assert(arc.in_graph);
    return get(arc.get_pos());
}

void DCHQueue::push(DCHQueueEntry entry, DCHGraph& graph) {
    unsigned arc_rank = std::min(graph.nodes[entry.arc.from].rank, graph.nodes[entry.arc.to].rank);
    unsigned arc_priority = arc_rank*100+__builtin_popcount(entry.arc.label.get_label());

    if(queue.contains_id(arc_priority)) {
        if(is_in_queue.find(entry.arc) != is_in_queue.end()) {
            return;
        }
        unsigned_to_arc[queue.get_key(arc_priority)].push(entry);
    } else {
        queue.push({arc_priority, static_cast<unsigned>(unsigned_to_arc.size())});
        unsigned_to_arc.push_back(std::queue<DCHQueueEntry>{});
        unsigned_to_arc.back().push(entry);
    }

    is_in_queue.insert(entry.arc);
}

DCHQueueEntry DCHQueue::pop() {
    auto vec = queue.peek();

    auto entry = unsigned_to_arc[vec.key].front();
    unsigned_to_arc[vec.key].pop();

    is_in_queue.erase(entry.arc);

    if(unsigned_to_arc[vec.key].empty()) {
        queue.pop();
    }

    return entry;
}

std::pair<unsigned, std::vector<unsigned>> DCHGraph::dijkstra(unsigned from, unsigned to, Label profile) {
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

/// Computes the upward shortcut pairs <p2, child> of p1 as defined by Zhang and Yu (2022)
std::vector<std::pair<CHMArc, CHMArc>> DCHGraph::SCPPlus(CHMArc p1) {
    std::vector<std::pair<CHMArc, CHMArc>> pairs;

    std::vector<CHMArc> ne = Ne(p1);

    for (CHMArc p2 : ne) {
        CHMArc _p1 = p1;
        CHMArc _p2 = p2;

        if(_p1.from == _p2.to) {
            // make sure that Phi(w) > Phi(u)
            if (nodes[p1.from].rank > nodes[p2.from].rank) continue;

            std::swap(_p1, _p2);
        } else {
            // make sure that Phi(w) > Phi(u)
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue;
        }

        assert(_p1.to == _p2.from);


        // Find child of p1 and p2
        for (CHMArc child : nodes[_p1.from].arcs) {
            if (child.to != _p2.to) continue;         // correct direction
            if (child.mid_node != _p1.to) continue;   // correct mid node
            if (child.weight >= inf_weight) continue; // handle infinite weight
            if (child.label != _p1.label.unite(_p2.label)) continue; // correct label

            pairs.push_back({p2, child});
        }
    }

    return pairs;
}

std::vector<CHMArc> DCHGraph::Ne(CHMArc p1) {
    std::vector<CHMArc> ne;
    
    if (nodes[p1.from].rank > nodes[p1.to].rank) {
        // p1 is a downward arc
        for (CHMArc p2 : nodes[p1.to].arcs) {
            assert(p1.to == p2.from);
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue;
            if (p2.to == p1.from) continue; // skip loops
            ne.push_back(p2);
        }
    } else {
        // p1 is an upward arc
        for (CHMArc p2 : in_arcs(p1.from)) {
            assert(p2.to == p1.from);
            if (p2.from == p1.to) continue; // skip loops
            if (nodes[p2.from].rank < nodes[p1.from].rank) continue;
            ne.push_back(p2);
        }
    }

    return ne;
}

CHMArc DCHGraph::Np(CHMArc p1, CHMArc p2) {
    if(p1.to != p2.from) {
        std::swap(p1, p2);
    }

    assert(p1.to == p2.from);
    assert(p1.weight < inf_weight && p2.weight < inf_weight);
    assert(p1.from != p2.to); // no loops

    for(const auto& arc : nodes[p1.from].arcs) {
        if(arc.to == p2.to && arc.mid_node == p1.to && arc.label == p1.label.unite(p2.label) && arc.in_graph) {
            return arc;
        }
    }

    CHMArc new_arc = add_arc(CHMArc{p1.from, p1.to, p2.to, inf_weight, p1.label.unite(p2.label)});
    return new_arc;
}

/// Computes the downward shortcut pairs <p1, p2> of child as defined by Zhang and Yu (2022)
std::vector<std::pair<CHMArc, CHMArc>> DCHGraph::SCPMinus(CHMArc child) {
    std::vector<std::pair<CHMArc, CHMArc>> pairs;

    for (CHMArc p1 : nodes[child.from].arcs) {
        if (nodes[child.from].rank < nodes[p1.to].rank) continue; // ensure p1 is downward
        if (child.mid_node != p1.to) continue; // ensure p1 is a part of the child

        for (CHMArc p2 : nodes[p1.to].arcs) {
            if (p2.to != child.to) continue;
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue; // ensure p2 is upward
            if (p1.label.unite(p2.label) != child.label) continue;

            pairs.push_back({p1, p2});
        }
    }

    return pairs;
}

/// Decrease the weight of an original arc in the Contraction Hierarchy. Maintains a valid CH index.
void DCHGraph::DCHMinus(CHMArcPos e_o_pos, unsigned w_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;

    assert(w_o > w_n && "DCHMinus can only be used to decrease weights");

    DCHQueue queue(nodes.size()*100+64);

    e_o.weight = w_n;
    queue.push(DCHQueueEntry{e_o, CHMArc(), CHMArc(), false}, *this);

    while(!queue.empty()) {
        auto entry = queue.pop();
        auto p1 = entry.arc;
        if(p1.weight == inf_weight) continue;

        auto ne = Ne(p1);

        for(const auto& p2 : ne) {
            if(p2.weight == inf_weight) continue;
            CHMArc child = ref(Np(p1, p2));

            if (p1.weight + p2.weight < child.weight) {
                update(child, p1.weight + p2.weight, child.label);
                queue.push(DCHQueueEntry{child, p1, p2, false}, *this);
            }
        }
    }
}


/// Increase the weight of an original arc in the Contraction Hierarchy. Maintains a valid CH index.
void DCHGraph::DCHPlus(CHMArcPos e_o_pos, unsigned w_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;

    assert(w_o < w_n && "DCHPlus can only be used to increase weights");

    DCHQueue queue(nodes.size()*100+64);

    queue.push(DCHQueueEntry{e_o, CHMArc(), CHMArc(), true}, *this);
    e_o.weight = w_n;

    std::vector<DCHArcPos> possible_garbage;

    while(!queue.empty()) {
        auto entry = queue.pop();
        auto arc = entry.arc;

        for(std::pair<CHMArc, CHMArc> scp : SCPPlus(arc)) {
            CHMArc p2 = scp.first;
            CHMArc child = scp.second;

            if (arc.weight + p2.weight == child.weight) {
                queue.push(DCHQueueEntry{child, arc, p2, true}, *this);
            }
        }

        unsigned new_weight = compute_weight(arc);

        if(new_weight == inf_weight) {
            possible_garbage.push_back({arc.from, arc.arc_index});
        }
        
        update(arc, new_weight, ref(arc).label);
    }

    for(const auto& pos : possible_garbage) {
        if(nodes[pos.node_index].arcs[pos.arc_index].weight == inf_weight) {
            invalidate(nodes[pos.node_index].arcs[pos.arc_index]);
        }
    }
}


unsigned DCHGraph::compute_weight(CHMArc arc) {
    unsigned k = inf_weight;

    if(arc.mid_node == invalid_id) {
        return ref(arc).weight;
    }

    for(std::pair<CHMArc, CHMArc> scp : SCPMinus(arc)) {
        CHMArc p1 = scp.first;
        CHMArc p2 = scp.second;

        if (p1.weight + p2.weight < k) {
            k = p1.weight + p2.weight;
        }
    }

    return k;
}

Label DCHGraph::compute_label(CHMArc arc) {
    Label k = Label::fully_restricted();

    if(arc.mid_node == invalid_id) {
        return ref(arc).label;
    }

    for(std::pair<CHMArc, CHMArc> scp : SCPMinus(arc)) {
        CHMArc p1 = scp.first;
        CHMArc p2 = scp.second;

        if(p1.weight + p2.weight != arc.weight) continue;

        if (p1.label.unite(p2.label).is_subset_of(k)) {
            k = p1.label.unite(p2.label);
        }
    }

    return k;
}


void DCHGraph::DCHAlt(CHMArcPos e_o_pos, unsigned w_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;
    
    DCHQueue queue(nodes.size()*100+64);

    bool inc = w_n > w_o;

    if(inc) {
        queue.push(DCHQueueEntry{e_o, CHMArc(), CHMArc(), true}, *this);
        e_o.weight = w_n;
    } else {
        e_o.weight = w_n;
        queue.push(DCHQueueEntry{e_o, CHMArc(), CHMArc(), false}, *this);
    }
    
    while(!queue.empty()) {
        auto entry = queue.pop();
        auto e = entry.arc;
        auto increment = entry.increment;

        if(increment) {
            // Handle possible recontraction
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

                std::vector<DCHQueueEntry> potential_shortcuts;

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

                        // TODO: do we need a witness search to check if the shortcut is necessary?
                        // Could potentially help to reduce the number of shortcuts
                        unsigned total_weight = e1.weight + e2.weight;
                        if(total_weight < w_n) {
                            CHMArc shortcut{from_id, e1.to, to_id, total_weight, e1.label.unite(e2.label)};
                            potential_shortcuts.push_back(DCHQueueEntry{shortcut, e1, e2, true});
                        }
                    }
                }

                for(const auto& shortcut : potential_shortcuts) {
                    CHMArc added_arc = add_arc(shortcut.arc); // TODO: could also reduce existing arc, but not necessary for correctness
                    queue.push(DCHQueueEntry{added_arc, shortcut.p1, shortcut.p2, true}, *this);
                }
            }


            // DCH+
            for(std::pair<CHMArc, CHMArc> scp : SCPPlus(e)) {
                CHMArc p2 = scp.first;
                CHMArc child = scp.second;

                if (e.weight + p2.weight == child.weight) {
                    queue.push(DCHQueueEntry{child, e, p2, true}, *this);
                }
            }

            ref(e).weight = compute_weight(e);

        } else { // decrement

            // DCH-
            for(std::pair<CHMArc, CHMArc> scp : SCPPlus(e)) {
                CHMArc p2 = scp.first;
                CHMArc& child = ref(scp.second);

                if (e.weight + p2.weight < child.weight) {
                    child.weight = e.weight + p2.weight;
                    queue.push(DCHQueueEntry{child, e, p2, false}, *this);
                }
            }

        }
    }
}

void DCHGraph::invalidate(CHMArc arc) {
    CHMArc& e = ref(arc);
    e.weight = inf_weight;
    e.in_graph = false;
    
    garbage[e.from].push_back({e.from, e.arc_index});
    assert(e.to == e.twin.node_index);
    backward_garbage[e.to].push_back({e.twin.node_index, e.twin.arc_index});
}
