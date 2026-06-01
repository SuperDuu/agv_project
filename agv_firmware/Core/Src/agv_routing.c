#include "agv_routing.h"
#include <stddef.h>

void Map_Init(AGV_Map_t *map) {
    map->total_nodes = MAX_NODES;
    for (int i = 0; i < MAX_NODES; i++) {
        map->nodes[i].node_id = i;
        map->nodes[i].edge_count = 0;
    }
}

void Map_AddEdge(AGV_Map_t *map, uint16_t from_id, uint16_t to_id, uint16_t dist, AGV_Heading_t hdg) {
    if (from_id >= MAX_NODES || to_id >= MAX_NODES) return;
    
    Node_t *node = &map->nodes[from_id];
    if (node->edge_count < MAX_EDGES_PER_NODE) {
        node->edges[node->edge_count].target_node_id = to_id;
        node->edges[node->edge_count].distance = dist;
        node->edges[node->edge_count].heading = hdg;
        node->edge_count++;
    }
}

bool Routing_Dijkstra(AGV_Map_t *map, uint16_t start_id, uint16_t target_id, uint16_t *out_path, uint16_t *out_path_len) {
    uint32_t dist[MAX_NODES];
    int16_t prev[MAX_NODES];
    bool visited[MAX_NODES];
    
    for (int i = 0; i < MAX_NODES; i++) {
        dist[i] = INF_DIST;
        prev[i] = -1;
        visited[i] = false;
    }
    
    dist[start_id] = 0;
    
    for (int i = 0; i < MAX_NODES; i++) {
        uint32_t min_dist = INF_DIST;
        int16_t u = -1;
        
        for (int j = 0; j < MAX_NODES; j++) {
            if (!visited[j] && dist[j] < min_dist) {
                min_dist = dist[j];
                u = j;
            }
        }
        
        if (u == -1 || u == target_id) break;
        visited[u] = true;
        
        Node_t *node = &map->nodes[u];
        for (int e = 0; e < node->edge_count; e++) {
            uint16_t v = node->edges[e].target_node_id;
            uint32_t weight = node->edges[e].distance;
            
            if (!visited[v] && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
            }
        }
    }
    
    // Nếu không tìm thấy đường đi tới đích
    if (dist[target_id] == INF_DIST) {
        *out_path_len = 0;
        return false;
    }
    
    // Tái cấu trúc lại mảng Path từ điểm cuối ngược về điểm đầu
    uint16_t temp_path[MAX_NODES];
    int count = 0;
    int16_t curr = target_id;
    
    while (curr != -1) {
        temp_path[count++] = curr;
        curr = prev[curr];
    }
    
    // Đảo ngược mảng để có thứ tự từ đầu đến cuối
    for (int i = 0; i < count; i++) {
        out_path[i] = temp_path[count - 1 - i];
    }
    *out_path_len = count;
    
    return true;
}

AGV_Heading_t Routing_GetHeading(AGV_Map_t *map, uint16_t current_id, uint16_t next_id) {
    if (current_id >= MAX_NODES) return HEAD_NONE;
    
    Node_t *node = &map->nodes[current_id];
    for (int e = 0; e < node->edge_count; e++) {
        if (node->edges[e].target_node_id == next_id) {
            return node->edges[e].heading;
        }
    }
    return HEAD_NONE;
}
