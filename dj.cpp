

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include <pugixml.hpp>
#include <limits>
#include <unordered_set>


struct NodeInfo {
    double x;
    double y;
};

// derlemek iç,n :::::::     g++ dj.cpp -lpugixml -o Djikstra 

std::map<std::string, NodeInfo> parseGraphML(const std::string& file_path, std::map<std::string, std::vector<std::pair<std::string, double>>>& edges) {
    std::map<std::string, NodeInfo> nodes;
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    if (!result) {
        std::cerr << "Error loading file: " << result.description() << std::endl;
        return nodes;
    }

    pugi::xml_node graph = doc.child("graphml").child("graph");

    for (pugi::xml_node node = graph.child("node"); node; node = node.next_sibling("node")) {
        std::string id = node.attribute("id").value();
        double x = std::stod(node.child("data").text().get());
        double y = std::stod(node.child("data").next_sibling("data").text().get());
        nodes[id] = {x, y};
    }

    for (pugi::xml_node edge = graph.child("edge"); edge; edge = edge.next_sibling("edge")) {
        std::string source = edge.attribute("source").value();
        std::string target = edge.attribute("target").value();

        double weight = 1.0; 
        edges[source].emplace_back(target, weight);
    }

    return nodes;
}


std::vector<std::string> dijkstra(const std::string& start, const std::string& goal, const std::map<std::string, NodeInfo>& nodes, const std::map<std::string, std::vector<std::pair<std::string, double>>>& edges, const std::unordered_set<std::string>& exclude_nodes) {
    std::map<std::string, double> distances;
    std::map<std::string, std::string> previous;
    std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> pq;

    for (const auto& [node, _] : nodes) {
        if (exclude_nodes.find(node) == exclude_nodes.end()) {
            distances[node] = std::numeric_limits<double>::infinity();
            previous[node] = "";
        }
    }
    distances[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty()) {
        double current_distance = pq.top().first;
        std::string u = pq.top().second;
        pq.pop();

        if (u == goal) break;

        if (current_distance > distances[u]) continue;

        for (const auto& [v, weight] : edges.at(u)) {
            if (exclude_nodes.find(v) != exclude_nodes.end()) continue;

            double distance_through_u = current_distance + weight;
            if (distance_through_u < distances[v]) {
                distances[v] = distance_through_u;
                previous[v] = u;
                pq.push({distance_through_u, v});
            }
        }
    }

    std::vector<std::string> path;
    for (std::string at = goal; !at.empty(); at = previous[at]) {
        path.push_back(at);
        if (at == start) break;
    }
    std::reverse(path.begin(), path.end());

    return path;
}


std::vector<std::tuple<std::string, std::string, bool>> createSourceTargetDictionary(const std::vector<std::string>& path) {
    std::vector<std::tuple<std::string, std::string, bool>> source_target;

    size_t pathlen = path.size();
    for (size_t i = 0; i < pathlen - 1; ++i) {
        source_target.emplace_back(path[i], path[i + 1], true);
    }

    return source_target;
}

int main() {
    std::map<std::string, std::vector<std::pair<std::string, double>>> edges;

    std::string file_path = "/home/f/cppconvert/g.graphml"; 
    std::map<std::string, NodeInfo> nodes = parseGraphML(file_path, edges);

    std::string start_node = "1";
    std::string goal_node = "48";  

    std::unordered_set<std::string> exclude_nodes = {"10"}; //dont use 

    std::vector<std::string> shortest_path = dijkstra(start_node, goal_node, nodes, edges, exclude_nodes);

    std::cout << "Shortest path from " << start_node << " to " << goal_node << ":" << std::endl;//source target list sollama henüz yok 
    for (const std::string& node : shortest_path) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

 
    std::vector<std::tuple<std::string, std::string, bool>> source_target_dict = createSourceTargetDictionary(shortest_path);

    std::cout << "Source-Target Dictionary:" << std::endl;
    for (const auto& [source, target, flag] : source_target_dict) {
        std::cout << "(" << source << ", " << target << ", " << (flag ? "true" : "false") << ")" << std::endl;
    }

  
    std::string node_id = "10"; //id ile  nodun x ve y sini kullanma 
    if (nodes.find(node_id) != nodes.end()) {
        NodeInfo node_info = nodes[node_id];
        std::cout << "Node " << node_id << ": x = " << node_info.x << ", y = " << node_info.y << std::endl;
    } else {
        std::cout << "Node " << node_id << " not found." << std::endl;
    }

    return 0;
}
