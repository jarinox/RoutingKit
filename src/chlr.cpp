#include <routingkit/chlr.h>

CHLRGraph::CHLRGraph(unsigned node_count, const std::vector<unsigned> &tail,
                     const std::vector<unsigned> &head,
                     const std::vector<unsigned> &weight,
                     const std::vector<float> &latitude,
                     const std::vector<float> &longitude,
                     std::vector<Label> &label) {
    nodes.resize(node_count);

    for(unsigned i = 0; i < node_count; ++i){
        nodes[i].lat = latitude[i];
        nodes[i].lon = longitude[i];
    }

    for (unsigned i = 0; i < tail.size(); ++i) {
        add_arc(tail[i], invalid_id, head[i], weight[i], label[i]);
    }
}

void CHLR::build() {
    CHLRGraph contraction_graph = this->graph;
    order.resize(graph.nodes.size());

    MinIDQueue queue(graph.nodes.size());
    unsigned contracted_node_count = 0;

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        queue.push({i, estimate_node_importance(graph, i)});
    }

    while (!queue.empty()) {
        auto p = queue.pop();
        auto node_id = p.id;
        order[contracted_node_count++] = node_id;
        graph.nodes[node_id].rank = contracted_node_count;
        contraction_graph.nodes[node_id].rank = contracted_node_count;

        CHLRNode &node = graph.nodes[node_id];

        // Raise neighbour levels for all arcs connected to this node
        // This is used for estimating node importance
        std::set<unsigned> neighbors;
        for (const auto &arc : node.in_arcs) {
            neighbors.insert(arc.other_node);
            graph.nodes[arc.other_node].neighbour_level++;
        }
        for (const auto &arc : node.out_arcs) {
            if (neighbors.find(arc.other_node) == neighbors.end()) {
                graph.nodes[arc.other_node].neighbour_level++;
                neighbors.insert(arc.other_node);
            }
        }
        for (const auto &neighbor : neighbors) {
            if (neighbor == node_id) continue;  // Skip self-loops
            if (!queue.contains_id(neighbor)) continue;

            unsigned new_importance = estimate_node_importance(graph, neighbor);
            unsigned current_importance = queue.get_key(neighbor);

            if (new_importance < current_importance) {
                queue.decrease_key({neighbor, new_importance});
            } else if (new_importance > current_importance) {
                queue.increase_key({neighbor, new_importance});
            }
        }

        assert(!queue.contains_id(node_id));

        std::cout << "Contracting, queue left " << queue.size() << " arc combinations " << graph.nodes[node_id].in_arcs.size() * graph.nodes[node_id].out_arcs.size() << std::endl;


        // Contract the node
        node.sort_arcs_for_weight();
        for (const auto &in_arc : graph.nodes[node_id].in_arcs) {
            if (in_arc.other_node == node_id) continue;  // Skip self-loops
            if (!queue.contains_id(in_arc.other_node)) // TODO: slow, may be optimized
                continue;  // Ensure rank(other_node) > rank(node_id)

            DijkstraLR dijkstra(graph, in_arc.other_node);

            for (const auto &out_arc : graph.nodes[node_id].out_arcs) {
                if (in_arc.other_node == out_arc.other_node)
                    continue;  // Skip self-loops
                if (!queue.contains_id(out_arc.other_node)) // TODO: slow, may be optimized
                    continue;  // Ensure rank(other_node) > rank(node_id)
                

                Label newLabel = in_arc.label.unite(out_arc.label);
                Label R = newLabel;
                R.invert();

                unsigned shortcut_weight = in_arc.weight + out_arc.weight;

                unsigned witness_weight = dijkstra.witness_search(
                    out_arc.other_node, R, [&](unsigned bypass_node) {
                        return (node_id != bypass_node) && (graph.nodes[bypass_node].rank > graph.nodes[node_id].rank);
                    });

                if (shortcut_weight < witness_weight) {
                    auto need_add = graph.add_or_reduce_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);
                    if(need_add)
                        contraction_graph.add_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);

                    //graph.add_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);
                    //contraction_graph.add_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);
                }
            }
        }

        graph.remove_incident_arcs(node_id);
    }

    graph = std::move(contraction_graph);
}

unsigned estimate_node_importance(const CHLRGraph &graph, unsigned node_id) {
    const CHLRNode &node = graph.nodes[node_id];

    unsigned in_deg = node.in_arcs.size();
    unsigned out_deg = node.out_arcs.size();

    unsigned estimated_shortcuts = in_deg * out_deg;
    unsigned arcs_removed = in_deg + out_deg;
    unsigned edge_difference = estimated_shortcuts > arcs_removed ? estimated_shortcuts - arcs_removed : 0;

    // Combine factors (weights can be tuned)
    return 1000 * edge_difference + 100 * node.neighbour_level + in_deg + out_deg;
}

unsigned DijkstraLR::witness_search(
    unsigned end_node, Label restriction,
    const std::function<bool(unsigned)> &is_valid_node) {

    if(storage.find(restriction) == storage.end()){
        storage[restriction] = {
            MinIDQueue(graph.nodes.size()),
            std::vector<unsigned>(graph.nodes.size(), inf_weight),
            std::set<unsigned>()};
        
        storage[restriction].queue.push({start_node, 0});
        storage[restriction].tentative_distance[start_node] = 0;
    }

    auto& queue = storage[restriction].queue;
    auto& tentative_distance = storage[restriction].tentative_distance;
    auto& visited_nodes = storage[restriction].visited_nodes;

    previous_restriction = restriction;

    if(tentative_distance[end_node] != inf_weight) {
        return tentative_distance[end_node];
    }

    unsigned pop_count = 0;
    while (!queue.empty() && pop_count < max_pop_count) {
        auto p = queue.pop();
        unsigned current_node = p.id;
        unsigned current_distance = p.key;

        if (current_node == end_node) {
            return current_distance;
        }

        visited_nodes.insert(current_node);

        for (const auto &arc : graph.nodes[current_node].out_arcs) {
            if (!arc.label.is_allowed(restriction)) continue;
            if (!is_valid_node(arc.other_node)) continue;

            unsigned new_distance = current_distance + arc.weight;
            if (new_distance >= tentative_distance[arc.other_node]) continue;

            tentative_distance[arc.other_node] = new_distance;
            if (!queue.contains_id(arc.other_node)) {
                queue.push({arc.other_node, new_distance});
            } else {
                queue.decrease_key({arc.other_node, new_distance});
            }
        }
    }

    return std::numeric_limits<unsigned>::max();
}


void CHLRGraph::remove_incident_arcs(unsigned node_id) {
    auto &node = nodes[node_id];

    // Collect affected nodes for in_arcs and out_arcs separately
    std::unordered_set<unsigned> in_affected_nodes;
    std::unordered_set<unsigned> out_affected_nodes;

    for (const auto &in_arc : node.in_arcs)
        in_affected_nodes.insert(in_arc.other_node);
    for (const auto &out_arc : node.out_arcs)
        out_affected_nodes.insert(out_arc.other_node);

    // Remove arcs from in_affected_nodes' out_arcs
    for (unsigned other_id : in_affected_nodes) {
        auto &out_arcs = nodes[other_id].out_arcs;
        out_arcs.erase(
            std::remove_if(out_arcs.begin(), out_arcs.end(),
                           [&](const CHLRArc &arc) { return arc.other_node == node_id; }),
            out_arcs.end());
    }

    // Remove arcs from out_affected_nodes' in_arcs
    for (unsigned other_id : out_affected_nodes) {
        auto &in_arcs = nodes[other_id].in_arcs;
        in_arcs.erase(
            std::remove_if(in_arcs.begin(), in_arcs.end(),
                           [&](const CHLRArc &arc) { return arc.other_node == node_id; }),
            in_arcs.end());
    }

    node.in_arcs.clear();
    node.out_arcs.clear();
    node.in_arcs.shrink_to_fit();
    node.out_arcs.shrink_to_fit();
}


void CHLRGraph::add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    nodes[from].out_arcs.push_back({
        .other_node = to,
        .mid_node = mid_node,
        .weight = weight,
        .label = label,
    });

    nodes[to].in_arcs.push_back({
        .other_node = from,
        .mid_node = mid_node,
        .weight = weight,
        .label = label,
    });
}

CHLRArc& CHLRGraph::get_reverse_arc(CHLRArc &arc, unsigned start_node) {
    for (auto &in_arc : nodes[arc.other_node].in_arcs) {
        if (in_arc.mid_node == arc.mid_node && in_arc.other_node == start_node && arc.label == in_arc.label) {
            return in_arc;
        }
    }
    throw std::runtime_error("Reverse arc not found");
}

bool CHLRGraph::add_or_reduce_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    for (auto &arc : nodes[from].out_arcs) {
        if (arc.other_node == to) {
            if (arc.weight > weight) {
                if(label.is_subset_of(arc.label)) {
                    // New arc is shorter and has fewer restrictions, replace existing shortcut
                    auto& reversed_arc = get_reverse_arc(arc, from);

                    arc.weight = weight;
                    arc.label = label;

                    reversed_arc.weight = weight;
                    reversed_arc.label = label;
                    return true;
                }
                // new arc is shorter but has more restrictions, continue search or add new arc
            } else {
                if(arc.label.is_subset_of(label)) {
                    // Existing arc is shorter and has fewer restrictions, do not add new shortcut
                    return false;
                }
            }
        }
    }

    add_arc(from, mid_node, to, weight, label);
    return true;
}

void CHLRQuery::extract_directional_graphs() {
    forward.nodes.resize(graph.nodes.size());
    backward.nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        // Copy node metadata to forward and backward graphs
        forward.nodes[i].lat = graph.nodes[i].lat;
        forward.nodes[i].lon = graph.nodes[i].lon;
        forward.nodes[i].rank = graph.nodes[i].rank;
        backward.nodes[i].lat = graph.nodes[i].lat;
        backward.nodes[i].lon = graph.nodes[i].lon;
        backward.nodes[i].rank = graph.nodes[i].rank;
        
        // Clean up existing arcs
        forward.nodes[i].out_arcs.clear();
        backward.nodes[i].out_arcs.clear();
    }

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        unsigned from_rank = graph.nodes[i].rank;

        // Add arcs to forward/backward graph
        for (auto arc : graph.nodes[i].out_arcs) {
            unsigned to_rank = graph.nodes[arc.other_node].rank;

            if(from_rank == to_rank) {
                continue; // skip self-loops
            }

            if (from_rank < to_rank) {
                forward.nodes[i].out_arcs.push_back(arc);
            } else {
                // reverse backward arcs
                backward.nodes[arc.other_node].out_arcs.push_back({
                    .other_node = i,
                    .mid_node = arc.mid_node,
                    .weight = arc.weight,
                    .label = arc.label,
                });
            }
        }
    }

    for (unsigned i = 0; i < forward.nodes.size(); ++i) {
        forward.nodes[i].sort_arcs_for_weight();
        backward.nodes[i].sort_arcs_for_weight();
        graph.nodes[i].sort_arcs_for_weight();
    }
}

void CHLRQuery::run() {
    MinIDQueue forward_queue = MinIDQueue(graph.nodes.size());
    MinIDQueue backward_queue = MinIDQueue(graph.nodes.size());
    std::vector<unsigned> forward_distance;
    std::vector<unsigned> backward_distance;
    TimestampFlags was_forward_pushed(graph.nodes.size());
    TimestampFlags was_backward_pushed(graph.nodes.size());
    std::vector<unsigned> forward_predecessor_node;
    std::vector<unsigned> backward_predecessor_node;
    std::vector<unsigned> forward_predecessor_arc;
    std::vector<unsigned> backward_predecessor_arc;

    meeting_node = invalid_id;
    unsigned best_distance = std::numeric_limits<unsigned>::max();

    forward_distance.resize(graph.nodes.size(), std::numeric_limits<unsigned>::max());
    backward_distance.resize(graph.nodes.size(), std::numeric_limits<unsigned>::max());
    forward_predecessor_node.resize(graph.nodes.size(), invalid_id);
    backward_predecessor_node.resize(graph.nodes.size(), invalid_id);
    forward_predecessor_arc.resize(graph.nodes.size(), invalid_id);
    backward_predecessor_arc.resize(graph.nodes.size(), invalid_id);

    forward_distance[start_node] = 0;
    backward_distance[end_node] = 0;

    forward_queue.push({start_node, 0});
    backward_queue.push({end_node, 0});

    was_forward_pushed.set(start_node);
    was_backward_pushed.set(end_node);

    bool search_forward = true;

    unsigned min_forward = forward_queue.peek().key;
    unsigned min_backward = backward_queue.peek().key;

    bool forward_finished = false;
    bool backward_finished = false;

    while(true) {
        if(forward_queue.empty()) 
            forward_finished = true;
        else
            min_forward = forward_queue.peek().key;
        
        if(backward_queue.empty()) 
            backward_finished = true;
        else
            min_backward = backward_queue.peek().key;
        
        if (min_forward >= best_distance)
            forward_finished = true;

        if (min_backward >= best_distance)
            backward_finished = true;

        if(forward_finished)
            search_forward = false;
        
        if(backward_finished)
            search_forward = true;
        
        if(forward_finished && backward_finished)
            break;

        if(search_forward) {
            settle(
                forward_queue, forward_distance, forward_predecessor_node,
                forward_predecessor_arc, was_forward_pushed,
                backward_queue, backward_distance, backward_predecessor_node,
                backward_predecessor_arc, was_backward_pushed, meeting_node,
                search_forward, forward, restriction, best_distance
            );
        } else {
            settle(
                backward_queue, backward_distance, backward_predecessor_node,
                backward_predecessor_arc, was_backward_pushed,
                forward_queue, forward_distance, forward_predecessor_node,
                forward_predecessor_arc, was_forward_pushed, meeting_node,
                search_forward, backward, restriction, best_distance
            );
        }

        
        search_forward = !search_forward;
    }

    _forward_predecessor_arc = std::move(forward_predecessor_arc);
    _backward_predecessor_arc = std::move(backward_predecessor_arc);
    _forward_predecessor_node = std::move(forward_predecessor_node);
    _backward_predecessor_node = std::move(backward_predecessor_node);
}


void CHLRQuery::settle(
    MinIDQueue &queue, std::vector<unsigned> &distance,
    std::vector<unsigned> &predecessor_node,
    std::vector<unsigned> &predecessor_arc,
    TimestampFlags &was_pushed,
    MinIDQueue &other_queue, std::vector<unsigned> &other_distance,
    std::vector<unsigned> &other_predecessor_node,
    std::vector<unsigned> &other_predecessor_arc,
    TimestampFlags &other_was_pushed, unsigned &meeting_node,
    bool &search_forward, CHLRGraph &graph, Label restriction, unsigned &best_distance) {

    auto p = queue.pop();
    unsigned current_node = p.id;
    unsigned current_distance = p.key;
    assert(current_distance == distance[current_node]);

    was_pushed.set(current_node);

    if (other_was_pushed.is_set(current_node)) {
        if(current_distance + other_distance[current_node] < best_distance) {
            best_distance = current_distance + other_distance[current_node];
            meeting_node = current_node;
        }
    }

    for (unsigned j = 0; j < graph.nodes[current_node].out_arcs.size(); ++j) {
        const auto &arc = graph.nodes[current_node].out_arcs[j];
        if (!arc.label.is_allowed(restriction)) continue;

        unsigned next_node = arc.other_node;
        unsigned new_distance = current_distance + arc.weight;

        if (new_distance < distance[next_node]) {
            distance[next_node] = new_distance;
            predecessor_node[next_node] = current_node;
            predecessor_arc[next_node] = j;

            if (!queue.contains_id(next_node)) {
                queue.push({next_node, new_distance});
            } else {
                queue.decrease_key({next_node, new_distance});
            }
        }
    }
}

CHLRArc reversed(unsigned start, CHLRArc arc) {
    return {
        .other_node = start,
        .mid_node = arc.mid_node,
        .weight = arc.weight,
        .label = arc.label
    };
}

CHLRArc search_arc(CHLRGraph &graph, unsigned from, unsigned to, Label restriction, bool backward) {
    /*unsigned i = 0, j = 0;

    while(i < graph.nodes[from].out_arcs.size() && j < graph.nodes[to].in_arcs.size()) {
        const auto &out_arc = graph.nodes[from].out_arcs[i];
        const auto &in_arc = graph.nodes[to].in_arcs[j];

        if (out_arc.other_node == to && out_arc.label.is_subset_of(restriction)) {
            return out_arc;
        } else if (in_arc.other_node == from && in_arc.label.is_subset_of(restriction)) {
            return reversed(to, in_arc);
        }

        if (out_arc.weight < in_arc.weight) {
            i++;
        } else {
            j++;
        }
    }*/

    if(!backward) {
        for (const auto &arc : graph.nodes[from].out_arcs) {
            if (arc.other_node == to && arc.label.is_subset_of(restriction)) {
                return arc;
            }
        }
    } else {
        for (const auto &arc : graph.nodes[to].in_arcs) {
            if (arc.other_node == from && arc.label.is_subset_of(restriction)) {
                return reversed(to, arc);
            }
        }
    }

    throw std::runtime_error("Arc not found from " + std::to_string(from) + " to " + std::to_string(to));
}

void expand_arc(CHLRGraph &graph, std::vector<CHLRArc> &path, unsigned start, CHLRArc arc, bool backward) {
    if(arc.mid_node == invalid_id) {
        path.push_back(arc);
    } else {
        expand_arc(graph, path, start, search_arc(graph, start, arc.mid_node, arc.label, backward), backward);
        expand_arc(graph, path, arc.mid_node, search_arc(graph, arc.mid_node, arc.other_node, arc.label, backward), backward);
    }
}

std::vector<CHLRArc> CHLRQuery::get_arc_path() {
    std::vector<CHLRArc> path;
    if (meeting_node == invalid_id) return path;

    unsigned current_node = meeting_node;
    while (current_node != start_node) {
        unsigned predecessor = _forward_predecessor_node[current_node];
        unsigned arc_index = _forward_predecessor_arc[current_node];

        if (predecessor == invalid_id || arc_index == invalid_id) break;

        CHLRArc arc = forward.nodes[predecessor].out_arcs[arc_index];
        path.push_back(arc);
        
        current_node = predecessor;
    }

    unsigned swap_direction = path.size();
    std::reverse(path.begin(), path.end());

    current_node = meeting_node;
    while (current_node != end_node) {
        unsigned predecessor = _backward_predecessor_node[current_node];
        unsigned arc_index = _backward_predecessor_arc[current_node];

        if (predecessor == invalid_id || arc_index == invalid_id) break;

        CHLRArc arc = backward.nodes[predecessor].out_arcs[arc_index];
        assert(arc.other_node == current_node);
        path.push_back(reversed(predecessor, arc));

        current_node = predecessor;
    }

    std::vector<CHLRArc> expanded_path;
    unsigned from = start_node;
    unsigned i = 0;
    for (auto &arc : path) {
        bool fw = i++ < swap_direction;
        expand_arc(graph, expanded_path, from, arc, !fw);
        from = arc.other_node;
    }

    return expanded_path;
}
