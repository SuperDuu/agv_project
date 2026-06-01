#ifndef AGV_ROUTING_H
#define AGV_ROUTING_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_NODES 100          // Hỗ trợ tối đa 100 điểm QR
#define MAX_EDGES_PER_NODE 4   // Mỗi ngã tư nối tối đa 4 điểm khác
#define INF_DIST 99999         // Giá trị vô cùng cho thuật toán Dijkstra

// Định nghĩa bí danh (Alias) cho 100 điểm để code dễ đọc
#define N00 0
#define N01 1
#define N02 2
#define N03 3
#define N04 4
#define N05 5
#define N06 6
#define N07 7
#define N08 8
#define N09 9
#define N10 10
#define N11 11
#define N12 12
#define N13 13
#define N14 14
#define N15 15
#define N16 16
#define N17 17
#define N18 18
#define N19 19
#define N20 20
// ... Bạn có thể tự định nghĩa thêm N21 đến N99 nếu cần ...

// Định nghĩa Hướng rẽ
typedef enum {
    DIR_NONE = 0,
    DIR_LEFT = 1,
    DIR_RIGHT = 2,
    DIR_STRAIGHT = 3,
    DIR_STOP = 4,
    DIR_REVERSE = 5
} AGV_Direction_t;

// Định nghĩa 1 Nút giao (Cạnh)
typedef struct {
    uint16_t target_node_id;   // Nút đích
    uint16_t distance;         // Khoảng cách (Trọng số)
    AGV_Direction_t direction; // Hướng rẽ để đi tới nút đích này
} Edge_t;

// Định nghĩa 1 Điểm quét QR (Nút)
typedef struct {
    uint16_t node_id;          // ID của điểm QR
    uint8_t edge_count;        // Số lượng đường rẽ từ điểm này
    Edge_t edges[MAX_EDGES_PER_NODE]; 
} Node_t;

// Bản đồ tổng thể
typedef struct {
    Node_t nodes[MAX_NODES];
    uint16_t total_nodes;
} AGV_Map_t;

// API Functions
void Map_Init(AGV_Map_t *map);
void Map_AddEdge(AGV_Map_t *map, uint16_t from_id, uint16_t to_id, uint16_t dist, AGV_Direction_t dir);
bool Routing_Dijkstra(AGV_Map_t *map, uint16_t start_id, uint16_t target_id, uint16_t *out_path, uint16_t *out_path_len);
AGV_Direction_t Routing_GetDirection(AGV_Map_t *map, uint16_t current_id, uint16_t next_id);

#endif /* AGV_ROUTING_H */
