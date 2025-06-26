# Routing Design Document

## Algorithm

本路径规划系统基于泛型搜索图实现了高性能的最短路径算法，支持时间依赖的路径规划和多模态交通网络。

### 核心数据结构

#### SearchGraph
```go
type SearchGraph[NT any, ET any] struct {
    edges []map[int]edge[ET]  // 邻接表存储图结构
    nodes []node[NT]          // 节点数组
    isTD  bool               // 是否为时间依赖图
    h     IHeuristics[NT, ET] // 启发式函数接口
    w     IEdgeWeight[ET]     // 边权重计算接口
    mu    *xsync.RBMutex     // 读写锁保证并发安全
}
```

SearchGraph是核心的搜索图结构，采用泛型设计支持不同类型的节点属性(NT)和边属性(ET)。支持时间依赖的路径规划，每条边可存储多个时间片的权重。

#### Node和Edge结构
```go
type node[T any] struct {
    p     geometry.Point  // 节点地理坐标
    attr  T              // 节点属性（可为驾车、步行、公交等不同类型）
    noOut bool           // 是否为终点节点（无出边）
}

type edge[T any] struct {
    v    []float64  // 不同时间片的边权重
    attr T          // 边属性（道路ID、公交线路等）
}
```

### 路径规划算法

#### A*算法实现
```go
func (g *SearchGraph[NT, ET]) ShortestPathAStar(start, end int, curTime float64) ([]PathItem[NT, ET], float64)
```

核心的A*最短路径算法实现，具有以下特点：
- 使用优先队列（PriorityQueue）管理待探索节点
- 支持时间依赖的边权重计算
- 采用启发式函数加速搜索
- 使用读写锁保证并发安全

算法流程：
1. 初始化开放集合（openSet）和已访问节点的gScore
2. 计算启发式距离fScore = gScore + heuristic
3. 从优先队列中取出fScore最小的节点进行扩展
4. 对每个邻居节点计算新的路径代价
5. 更新邻居节点的gScore和fScore，维护优先队列
6. 重复直到找到目标节点或开放集合为空

#### TAZ区域路径规划
```go
func (g *SearchGraph[NT, ET]) ShortestPathAStarToTaz(start int, endTaz TazPair, endP geometry.Point, sameTazDistance float64, curTime float64, availableSublineTypes []mapv2.SublineType) ([]PathItem[NT, ET], float64)
```

针对公交网络的特殊路径规划算法，支持到达指定交通分析区（TAZ）的路径规划：
- 终点为区域而非固定点位
- 支持公交线路类型过滤
- 使用专门的公交启发式函数

### 时间依赖性支持

#### 时间片管理
```go
func TimeToIndex(time float64) int
const TIME_SLICE_INTERVAL = 300  // 5分钟时间片
const TIME_SLICE_LENGTH = 288    // 一天288个时间片
```

系统将一天划分为288个5分钟的时间片，支持动态的交通状况建模：
- 每条边可存储不同时间片的权重
- 根据当前时间选择对应时间片的权重
- 支持公交发车时间表建模

#### 边权重动态计算
```go
type IEdgeWeight[ET any] interface {
    GetRuntimeEdgeWeight(ET, []float64, int, []mapv2.SublineType) float64
}
```

通过接口抽象边权重计算逻辑，支持：
- 基于时间片的动态权重
- 公交线路可用性检查
- 实时交通状况调整

### 启发式函数

#### 接口定义
```go
type IHeuristics[NT any, ET any] interface {
    HeuristicEuclidean(geometry.Point, geometry.Point) float64
    HeuristicBus(NT, []ET, geometry.Point, float64) float64
}
```

支持两种启发式函数：
- **HeuristicEuclidean**: 基于欧几里得距离的启发式函数，用于驾车和步行路径规划
- **HeuristicBus**: 专门针对公交网络的启发式函数，考虑换乘成本和线路连通性

### 优先队列实现

```go
type PriorityQueue []*Item
type Item struct {
    Value    int     // 节点ID
    Priority float64 // 优先级（fScore）
    Index    int     // 在堆中的索引
}
```

基于Go标准库heap接口实现的最小堆优先队列：
- 支持O(log n)的插入和删除操作
- 支持O(log n)的优先级更新操作
- 用于A*算法中管理待探索节点

### 路径重构

```go
func (g *SearchGraph[NT, ET]) reconstructPath(cameFrom map[int]int, curNode int, curTime float64) ([]PathItem[NT, ET], float64)
```

从目标节点回溯到起始节点重构完整路径：
- 记录每个节点的前驱节点
- 反向遍历构建路径
- 计算路径总代价
- 返回包含节点和边属性的路径序列

### 并发安全

系统使用读写锁（xsync.RBMutex）保证并发安全：
- 图结构在运行时保持稳定（边的连接关系不变）
- 边权重可能动态更新（需要写锁保护）
- 路径查询可以并发进行（使用读锁）

### 错误处理

定义了专门的错误类型：
```go
var (
    ErrOutOfTimeSlice = errors.New("out of time slice, should be less than 288")
    ErrNoTDGraph = errors.New("no time dependent graph, should set edge length with length 1")
)
```

### 工具函数

#### TAZ相关函数
```go
func PointToTaz(p geometry.Point, xStep float64, yStep float64, xMin float64, yMin float64) TazPair
func TazDistance(taz1 TazPair, taz2 TazPair, xStep float64, yStep float64) float64
```

支持地理坐标与交通分析区域的转换和距离计算。

#### 公交时刻表处理
```go
func StationTimeDependentCosts(cost float64, departureTimes []float64) []float64
func InServiceTime(cost TazCost, time float64) bool
```

处理公交发车时间和服务时间的相关计算。

## Driving routing

驾车路径规划模块基于道路网络构建专门的驾车图，支持时间依赖的动态路径规划，考虑了转弯惩罚、路况变化和AOI连接等实际驾车场景。

### 系统常量

#### 转弯时间惩罚
```go
const (
    TURN_STRAIGHT_TIME_PENALTY = 15     // 直行额外时间
    TURN_LEFT_TIME_PENALTY     = 60     // 左转额外时间（基于文献研究）
    TURN_RIGHT_TIME_PENALTY    = 5      // 右转额外时间
    TURN_AROUND_TIME_PENALTY   = 90     // 掉头额外时间（1.5倍左转）
)
```

转弯惩罚基于交通工程学研究，特别是左转延迟基于Stamatiadis等人的研究，保护左转相位会带来60-70秒的延迟。

#### 系统参数
```go
const (
    ROAD_JUNCTION_SPLIT = 3_0000_0000   // 道路ID与路口ID的分界线
    VEHICLE_SPEED = 60 / 3.6            // 默认车辆速度（m/s）
)
```

### 驾车专用数据结构

#### 启发式函数实现
```go
type DriveHeuristics struct {}

func (h DriveHeuristics) HeuristicEuclidean(p1, p2 geometry.Point) float64 {
    return geometry.Distance(p1, p2) / VEHICLE_SPEED
}

func (h DriveHeuristics) HeuristicBus(nodeAttr algo.DriveNodeAttr, fromEdgeAttrs []algo.DriveEdgeAttr, pEnd geometry.Point, time float64) float64 {
    return math.Inf(0)  // 驾车模式不支持公交启发式
}
```

驾车启发式函数基于欧几里得距离除以车辆速度，提供乐观的时间估计。

#### 边权重计算
```go
type DriveEdgeWeight struct {}

func (w DriveEdgeWeight) GetRuntimeEdgeWeight(edgeAttr algo.DriveEdgeAttr, edgeV []float64, tIndex int, availableSublineTypes []mapv2.SublineType) float64 {
    return edgeV[tIndex]
}
```

直接返回对应时间片的边权重，支持动态路况变化。

### 图构建过程

#### 主图构建函数
```go
func (r *Router) buildDriveGraph()
```

完整的驾车图构建过程，包括：
1. **道路节点和边的创建**
2. **路口连接的处理**
3. **AOI接入点的集成**

##### 道路处理
```go
// 为每条道路创建头尾节点
road.DriveHeadNodeId = driveGraph.InitNode(startP, algo.DriveNodeAttr{ID: road.Id}, false)
road.DriveTailNodeId = driveGraph.InitNode(endP, algo.DriveNodeAttr{ID: road.Id}, false)

// 计算平均通行时间（考虑多车道）
aveTimeCosts := make([]float64, algo.TIME_SLICE_LENGTH)
for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
    for j := 0; j < len(road.DrivingLanes); j++ {
        aveTimeCosts[i] += timeCosts[j][i] / count
    }
}
```

道路处理特点：
- 每条道路有头尾两个节点
- 支持多车道的平均速度计算
- 考虑不同时间片的动态速度
- 如果有时序速度数据则使用，否则使用最大限速

##### 路口处理
```go
// 处理路口车道连接
switch lane.Turn {
case mapv2.LaneTurn_LANE_TURN_STRAIGHT:
    penaltyCost = TURN_STRAIGHT_TIME_PENALTY
case mapv2.LaneTurn_LANE_TURN_LEFT:
    penaltyCost = TURN_LEFT_TIME_PENALTY
case mapv2.LaneTurn_LANE_TURN_RIGHT:
    penaltyCost = TURN_RIGHT_TIME_PENALTY
case mapv2.LaneTurn_LANE_TURN_AROUND:
    penaltyCost = TURN_AROUND_TIME_PENALTY
}

costs[i] = lane.Length/lane.MaxSpeed + penaltyCost
```

路口处理特点：
- 根据转弯类型添加不同的时间惩罚
- 路口车道不考虑路况输入，使用最大限速
- 连接前一道路的尾节点到后一道路的头节点

##### AOI集成
```go
// 为每个AOI创建入节点和出节点
aoi.DriveInNodeId = driveGraph.InitNode(aoi.CenterPoint, algo.DriveNodeAttr{ID: aoi.Id, IsAoi: true}, true)
aoi.DriveOutNodeId = driveGraph.InitNode(aoi.CenterPoint, algo.DriveNodeAttr{ID: aoi.Id, IsAoi: true}, false)

// 连接AOI到道路
// 道路头 -> AOI入节点
driveGraph.InitEdge(road.DriveHeadNodeId, aoi.DriveInNodeId, headTimeCosts, algo.DriveEdgeAttr{ID: road.Id})
// AOI出节点 -> 道路尾
driveGraph.InitEdge(aoi.DriveOutNodeId, road.DriveTailNodeId, tailTimeCosts, algo.DriveEdgeAttr{ID: road.Id})
```

AOI处理特点：
- 每个AOI有独立的入节点（目的地）和出节点（起点）
- 根据AOI在车道上的位置计算到达时间
- 支持一个AOI连接多个道路位置

### 位置转换函数

#### 起点节点转换
```go
func (r *Router) toDriveStartNode(pb *geov2.Position) (nodeId int)
```

将地理位置转换为图中的起点节点：
- **AOI位置**: 返回AOI的出节点（DriveOutNodeId）
- **车道位置**: 返回所在道路的尾节点（DriveTailNodeId）

#### 终点节点转换
```go
func (r *Router) toDriveEndNode(pb *geov2.Position) (nodeId int)
```

将地理位置转换为图中的终点节点：
- **AOI位置**: 返回AOI的入节点（DriveInNodeId）
- **车道位置**: 返回所在道路的头节点（DriveHeadNodeId）

### 路径搜索核心功能

#### 主搜索函数
```go
func (r *Router) SearchDriving(start, end *geov2.Position, time float64) (roadIDs []int32, cost float64, err error)
```

核心的驾车路径搜索功能，实现以下逻辑：

##### 同路段优化
```go
if startRoadId == endRoadId && startS < endS {
    return []int32{startRoadId}, (endS - startS) / VEHICLE_SPEED, nil
}
```

当起终点在同一道路且方向正确时，直接返回结果，无需图搜索。

##### 图搜索流程
1. **位置转换**: 将起终点位置转换为图节点
2. **A*搜索**: 调用底层图算法进行路径搜索
3. **路径解析**: 将节点序列转换为道路ID序列
4. **结果过滤**: 过滤掉路口ID，只保留道路ID

##### 错误处理
```go
defer func() {
    if e := recover(); e != nil {
        roadIDs = []int32{}
        cost = math.Inf(0)
        err = fmt.Errorf("panic: SearchDriving %v with input start=%v, end=%v, time=%v", e, start, end, time)
        log.Errorln(err)
    }
}()
```

包含完整的panic恢复机制，确保系统稳定性。

### 动态权重管理

#### 获取道路成本
```go
func (r *Router) GetRoadCost(roadId int32, time *float64) (float64, error)
```

获取指定道路在特定时间的通行成本：
- 支持指定时间查询或默认时间（0点）
- 返回对应时间片的边权重
- 处理道路不存在的错误情况

#### 设置道路成本
```go
func (r *Router) SetRoadCost(roadId int32, cost float64, time *float64) error
```

动态更新道路通行成本，支持两种模式：
- **指定时间**: 更新特定时间片的权重
- **全天更新**: 将所有时间片设置为相同权重

```go
if time != nil {
    // 更新指定时间片
    return r.driveGraph.SetEdgeLength(road.DriveHeadNodeId, road.DriveTailNodeId, algo.TimeToIndex(*time), cost)
} else {
    // 更新所有时间片
    costs := make([]float64, algo.TIME_SLICE_LENGTH)
    for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
        costs[i] = cost
    }
    return r.driveGraph.SetEdgeLengths(road.DriveHeadNodeId, road.DriveTailNodeId, costs)
}
```

### 设计特点

1. **时间依赖性**: 支持288个5分钟时间片的动态权重
2. **转弯建模**: 基于交通工程学的转弯延迟模型
3. **多车道支持**: 计算多车道道路的平均通行时间
4. **AOI集成**: 无缝连接道路网络与兴趣点
5. **错误恢复**: 完整的异常处理和错误恢复机制
6. **性能优化**: 同路段直接计算，避免不必要的图搜索

### 限制说明

- 暂不支持修改AOI到道路连接边的权重
- 路口车道的多lane group可能存在相互覆盖问题
- 转弯惩罚为固定值，未考虑信号灯实时状态

## Walking routing

步行路径规划模块基于步行车道网络构建专门的步行图，支持双向行走、AOI连接和精确位置定位，考虑了实际步行场景中的灵活性和连通性需求。

### 系统常量

#### 步行速度
```go
const (
    PERSON_SPEED = 1.1  // 行人步行速度（m/s）
)
```

基于人体工程学的标准步行速度，约4km/h。

### 步行专用数据结构

#### 启发式函数实现
```go
type WalkHeuristics struct {}

func (h WalkHeuristics) HeuristicEuclidean(p1, p2 geometry.Point) float64 {
    return geometry.Distance(p1, p2) / PERSON_SPEED
}

func (h WalkHeuristics) HeuristicBus(attr algo.WalkNodeAttr, fromEdgeAttrs []*routingv2.WalkingRouteSegment, pEnd geometry.Point, time float64) float64 {
    return math.Inf(0)  // 步行模式不支持公交启发式
}
```

步行启发式函数基于欧几里得距离除以步行速度，提供乐观的时间估计。

#### 边权重计算
```go
type WalkEdgeWeight struct {}

func (w WalkEdgeWeight) GetRuntimeEdgeWeight(edgeAttr *routingv2.WalkingRouteSegment, edgeV []float64, tIndex int, availableSublineTypes []mapv2.SublineType) float64 {
    return edgeV[tIndex]
}
```

直接返回对应时间片的边权重，步行模式通常使用固定权重。

#### 步行搜索节点
```go
type walkSearchNode struct {
    nodeId        int                                // 图中节点ID
    extraEdgeAttr *routingv2.WalkingRouteSegment    // 额外边属性
    extraCost     float64                           // 额外成本
}
```

扩展的搜索节点结构，支持从任意位置开始/结束的步行搜索：
- **nodeId**: 图中的基础节点
- **extraEdgeAttr**: 到达/离开该节点的路径段信息
- **extraCost**: 从精确位置到图节点的额外步行时间

#### 车道节点结构
```go
type WalkLaneNode struct {
    S      float64  // 距离起点的距离
    NodeId int      // 在graph中的nodeId
}
```

车道上的离散节点，用于将连续的车道离散化为图节点。

### 图构建过程

#### 主图构建函数
```go
func (r *Router) buildWalkGraph()
```

完整的步行图构建过程，包括：
1. **步行车道的节点合并**
2. **AOI接入点的集成**
3. **双向连接的建立**

##### 步行车道处理
```go
// 处理walking lane的nodeId，进行合并
walkingLanes := make(map[int32]*Lane)
for _, lane := range r.lanes {
    if lane.Type == mapv2.LaneType_LANE_TYPE_WALKING {
        walkingLanes[lane.Id] = lane
    }
}
```

步行车道处理特点：
- 只处理类型为WALKING的车道
- 通过连接关系合并相邻车道的端点
- 避免重复创建相同位置的节点

##### 节点合并逻辑
```go
// 遍历所有前驱，如果对接的点已经分配了NodeId，则采用之
for _, c := range lane.Predecessors {
    if c.Type == mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD {
        if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.HEAD]; ok {
            lane.WalkNodeIds[algo.HEAD] = v
            headOk = true
            break
        }
    } else {
        if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.TAIL]; ok {
            lane.WalkNodeIds[algo.HEAD] = v
            headOk = true
            break
        }
    }
}
```

节点合并特点：
- 检查前驱和后继车道的连接情况
- 复用已存在的节点ID，避免重复创建
- 确保连接点的一致性

##### AOI集成
```go
// 为每个AOI创建入节点和出节点
aoi.WalkOutNodeId = walkGraph.InitNode(aoi.CenterPoint, algo.WalkNodeAttr{ID: aoi.Id, IsAoi: true}, false)
aoi.WalkInNodeId = walkGraph.InitNode(aoi.CenterPoint, algo.WalkNodeAttr{ID: aoi.Id, IsAoi: true}, true)

// 在AOI位置插入中间节点
nodeId := walkGraph.InitNode(lane.GetPositionByS(p.S), algo.WalkNodeAttr{ID: lane.Id}, false)
lane.Nodes = append(lane.Nodes, algo.WalkLaneNode{S: p.S, NodeId: nodeId})

// 连接AOI与中间节点
walkGraph.InitEdge(aoi.WalkOutNodeId, nodeId, []float64{0.1}, nil)
walkGraph.InitEdge(nodeId, aoi.WalkInNodeId, []float64{0.1}, nil)
```

AOI处理特点：
- 每个AOI有独立的入节点和出节点
- 在AOI所在车道位置插入中间节点
- 使用极小的权重(0.1)连接AOI与车道
- 支持一个AOI连接多个车道位置

##### 双向连接建立
```go
// 对Lane中的node进行排序
sort.Slice(lane.Nodes, func(i, j int) bool {
    return lane.Nodes[i].S < lane.Nodes[j].S
})

// 在node间建立无向图
for i := 0; i < len(lane.Nodes)-1; i++ {
    cost := (lane.Nodes[i+1].S - lane.Nodes[i].S) / PERSON_SPEED
    // 正向边
    walkGraph.InitEdge(lane.Nodes[i].NodeId, lane.Nodes[i+1].NodeId, []float64{cost}, 
        &routingv2.WalkingRouteSegment{
            LaneId: lane.Id,
            MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_FORWARD,
        })
    // 反向边
    walkGraph.InitEdge(lane.Nodes[i+1].NodeId, lane.Nodes[i].NodeId, []float64{cost},
        &routingv2.WalkingRouteSegment{
            LaneId: lane.Id,
            MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_BACKWARD,
        })
}
```

双向连接特点：
- 对车道节点按距离排序
- 相邻节点间建立双向边
- 记录行走方向信息（前进/后退）
- 根据实际距离和步行速度计算权重

### 位置转换函数

#### 起点搜索节点转换
```go
func (r *Router) toWalkStartSearchNode(pb *geov2.Position) []walkSearchNode
```

将地理位置转换为可搜索的起点节点集合：

##### AOI位置处理
```go
if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
    aoiNode := r.aois[aoiPosition.GetAoiId()].WalkOutNodeId
    return []walkSearchNode{{nodeId: aoiNode, extraEdgeAttr: nil, extraCost: 0}}
}
```

##### 车道位置处理
```go
// 在lane的.Nodes中找到s所在位置前后的两个node
ind := sort.Search(len(lane.Nodes), func(i int) bool {
    return lane.Nodes[i].S >= s
})
return []walkSearchNode{
    {
        nodeId: lane.Nodes[ind-1].NodeId,
        extraEdgeAttr: &routingv2.WalkingRouteSegment{
            LaneId: lane.Id,
            MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_BACKWARD,
        },
        extraCost: (s - lane.Nodes[ind-1].S) / PERSON_SPEED,
    },
    {
        nodeId: lane.Nodes[ind].NodeId,
        extraEdgeAttr: &routingv2.WalkingRouteSegment{
            LaneId: lane.Id,
            MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_FORWARD,
        },
        extraCost: (lane.Nodes[ind].S - s) / PERSON_SPEED,
    },
}
```

车道位置处理特点：
- 使用二分搜索定位精确位置
- 返回前后两个候选节点
- 计算到达候选节点的额外成本
- 记录到达方向信息

#### 终点搜索节点转换
```go
func (r *Router) toWalkEndSearchNode(pb *geov2.Position) []walkSearchNode
```

类似起点转换，但方向相反：
- AOI位置返回入节点
- 车道位置的方向信息与起点相反

### 路径搜索核心功能

#### 主搜索函数
```go
func (r *Router) SearchWalking(start, end *geov2.Position, time float64) (segments []*routingv2.WalkingRouteSegment, cost float64, err error)
```

核心的步行路径搜索功能，实现以下逻辑：

##### 多起终点搜索
```go
startNodes := r.toWalkStartSearchNode(start)
endNodes := r.toWalkEndSearchNode(end)
var bestPath []algo.PathItem[algo.WalkNodeAttr, *routingv2.WalkingRouteSegment]
bestCost := math.Inf(0)

for _, startNode := range startNodes {
    for _, endNode := range endNodes {
        pt, cost := r.walkGraph.ShortestPath(startNode.nodeId, endNode.nodeId, time)
        // 添加额外成本
        if len(pt) > 1 {
            if startNode.extraEdgeAttr != nil {
                cost += startNode.extraCost
            }
            if endNode.extraEdgeAttr != nil {
                cost += endNode.extraCost
            }
        }
        if cost < bestCost {
            bestCost = cost
            bestPath = pt
            bestStartNode = startNode
            bestEndNode = endNode
        }
    }
}
```

多起终点搜索特点：
- 枚举所有起点-终点组合
- 选择总成本最小的路径
- 考虑精确位置到图节点的额外成本
- 对原地不动的情况特殊处理

##### 路径段合并
```go
ret := make([]*routingv2.WalkingRouteSegment, 0, len(bestPath)+1)
if bestStartNode.extraEdgeAttr != nil {
    ret = append(ret, protoutil.Clone(bestStartNode.extraEdgeAttr))
}
for _, attrs := range bestPath[:len(bestPath)-1] {
    edgeAttr := attrs.EdgeAttr
    if edgeAttr == nil {
        continue
    }
    // 合并同一车道的连续段
    if len(ret) > 0 && edgeAttr.LaneId == ret[len(ret)-1].LaneId {
        ret[len(ret)-1].MovingDirection = edgeAttr.MovingDirection
        continue
    }
    ret = append(ret, protoutil.Clone(edgeAttr))
}
if bestEndNode.extraEdgeAttr != nil {
    ret = append(ret, protoutil.Clone(bestEndNode.extraEdgeAttr))
}
```

路径段合并特点：
- 合并同一车道的连续路径段
- 保留最终的行走方向
- 添加起点和终点的额外路径段
- 过滤空的边属性

##### 错误处理
```go
defer func() {
    if e := recover(); e != nil {
        segments = []*routingv2.WalkingRouteSegment{}
        cost = math.Inf(0)
        err = fmt.Errorf("panic: SearchWalking %v with input start=%v, end=%v, time=%v", e, start, end, time)
        log.Errorln(err)
    }
}()
```

包含完整的panic恢复机制，确保系统稳定性。

### 设计特点

1. **双向连通性**: 步行车道支持双向行走，提高路径选择灵活性
2. **精确定位**: 支持车道上任意位置的起终点定位
3. **节点合并**: 智能合并相邻车道端点，减少冗余节点
4. **AOI集成**: 无缝连接步行网络与兴趣点
5. **多候选优化**: 通过多起终点搜索选择最优路径
6. **路径简化**: 自动合并同车道连续段，简化结果

### 与驾车模式的差异

1. **图结构**: 步行图为无向图，驾车图为有向图
2. **节点密度**: 步行图节点更密集，支持AOI中间插入
3. **权重计算**: 步行使用固定速度，驾车考虑动态路况
4. **搜索策略**: 步行支持多候选起终点，驾车使用单一节点
5. **路径表示**: 步行返回路径段序列，驾车返回道路ID序列

### 限制说明

- 不支持时间依赖的步行速度变化
- AOI连接权重为固定值(0.1)
- 未考虑步行的垂直高度差异
- 节点合并可能在复杂路网中存在边界情况

## Public transport routing

公交路径规划模块是最复杂的路径规划系统，集成了步行、公交、地铁等多种交通方式，支持基于TAZ（交通分析区）的换乘优化、时刻表建模和多模态路径搜索。

### 系统常量

#### 换乘和惩罚参数
```go
const (
    STATION_TRANSFER_PENALTY      = 1500  // 换乘惩罚时间（秒）
    SUBWAY_STATION_TRANSFER_RATIO = 1.0   // 地铁换乘比例
    BUS_STATION_TRANSFER_RATIO    = 1.3   // 公交换乘比例
    STATION_PASSBY_TIME           = 15    // 车站停车时间（秒）
    WALK_LANE_TRANSFER_PENALTY    = 300   // 车站间步行换乘惩罚（秒）
)
```

换乘惩罚基于实际交通体验设计，地铁换乘相对便利，公交换乘需要更多时间。

#### 交通工具速度
```go
const (
    BUS_SPEED    = 40 / 3.6   // 公交速度（m/s）约40km/h
    SUBWAY_SPEED = 100 / 3.6  // 地铁速度（m/s）约100km/h
)
```

#### 系统限制参数
```go
const (
    TRANSFER_SUBLINE_ID           = math.MaxInt32  // 车站内换乘边的特殊ID
    SAME_TAZ_STATION_NODES        = 10            // 同TAZ内候选车站数量
    MAX_WALKING_TOLERATE_TIME     = 60 * 35       // 最大可接受步行时间（秒）
    MAX_BUS_TOLERATE_TIME         = 3600 * 6      // 最大可接受公交时间（秒）
    MAX_TRANSFER_WALKING_DISTANCE = 200           // 车站间最大步行换乘距离（米）
)
```

#### TAZ距离候选
```go
var SAME_TAZ_DISTANCES = [7]float64{
    600, 840, 1080, 1320, 1560, 1800,  // 不同的TAZ内搜索距离（米）
}
```

### 公交专用数据结构

#### 启发式函数实现
```go
type BusHeuristics struct {}

func (h BusHeuristics) HeuristicEuclidean(p1, p2 geometry.Point) float64 {
    return math.Inf(0)  // 公交模式不使用欧几里得启发式
}

func (h BusHeuristics) HeuristicBus(attrNode algo.BusNodeAttr, fromEdgeAttrs []*algo.BusEdgeAttr, pEnd geometry.Point, time float64) float64
```

公交启发式函数是最复杂的启发式实现，考虑：
- **换乘惩罚检测**: 检查是否引入新的线路ID
- **TAZ直达查询**: 优先使用能直达目标TAZ的线路
- **最近TAZ策略**: 当无法直达时选择最近TAZ下车后步行
- **换乘比例调整**: 根据交通工具类型调整启发值

#### 边权重计算
```go
type BusEdgeWeight struct {}

func (w BusEdgeWeight) GetRuntimeEdgeWeight(edgeAttr *algo.BusEdgeAttr, edgeV []float64, tIndex int, availableSublineTypes []mapv2.SublineType) float64 {
    // 检查线路类型是否可用
    isAvailable := false
    for _, t := range availableSublineTypes {
        if t == edgeAttr.SublineType {
            isAvailable = true
            break
        }
    }
    if isAvailable {
        return edgeV[tIndex]
    } else {
        return math.Inf(0)
    }
}
```

支持动态过滤可用的公交线路类型（公交、地铁等）。

### 图构建过程

#### 主图构建函数
```go
func (r *Router) buildBusGraph()
```

公交图构建包括：
1. **车站节点创建**
2. **线路连接建立**
3. **换乘边添加**

##### 车站节点处理
```go
// 为每个车站创建入节点和出节点
station.StationInNodeId = busGraph.InitNode(station.CenterPoint, algo.BusNodeAttr{
    ID: stationID, 
    StationTAZCosts: nodeStationTazCosts1, 
    SublineTazCosts: nodeSublineTazCosts1, 
    CenterPoint: station.CenterPoint
}, false)

station.StationOutNodeId = busGraph.InitNode(station.CenterPoint, algo.BusNodeAttr{
    ID: stationID, 
    StationTAZCosts: nodeStationTazCosts2, 
    SublineTazCosts: nodeSublineTazCosts2, 
    CenterPoint: station.CenterPoint
}, false)

// 车站内部连接（进站到出站）
busGraph.InitEdge(station.StationInNodeId, station.StationOutNodeId, costs, 
    &algo.BusEdgeAttr{
        FromID: stationID, 
        ToID: stationID, 
        SublineID: TRANSFER_SUBLINE_ID, 
        SublineType: mapv2.SublineType_SUBLINE_TYPE_UNSPECIFIED
    })
```

车站节点特点：
- 每个车站有独立的入节点和出节点
- 携带完整的TAZ成本信息用于启发式计算
- 使用特殊的换乘线路ID连接进出节点

##### 线路连接建立
```go
// 处理线路上相邻车站的连接
for index, toStationID := range stationIDs[1:] {
    fromStation := r.aois[fromStationID]
    toStation := r.aois[toStationID]
    stationCost := stationCosts[index]
    costs := algo.StationTimeDependentCosts(stationCost, departureTimes)
    
    busGraph.InitEdge(fromStation.StationOutNodeId, toStation.StationInNodeId, costs,
        &algo.BusEdgeAttr{
            FromID: fromStationID, 
            ToID: toStationID, 
            SublineID: subline.Id, 
            SublineType: subline.PublicTransportSubline.Type
        })
}
```

线路连接特点：
- 连接前一车站的出节点到后一车站的入节点
- 使用时刻表生成时间依赖的边权重
- 记录线路ID和类型信息

##### 车站间换乘连接
```go
// 同TAZ内车站的换乘连接
for _, endStationID := range tazStationIDs {
    if startStationID == endStationID {
        continue
    }
    walkDistance := geometry.Distance(startStation.CenterPoint, endStation.CenterPoint)
    if walkDistance > MAX_TRANSFER_WALKING_DISTANCE {
        continue
    }
    
    // 建立双向换乘边
    costs := []float64{walkDistance/PERSON_SPEED + WALK_LANE_TRANSFER_PENALTY}
    busGraph.InitEdge(startStation.StationOutNodeId, endStation.StationInNodeId, costs, transferEdgeAttr)
    busGraph.InitEdge(endStation.StationOutNodeId, startStation.StationInNodeId, costs, transferEdgeAttr)
}
```

换乘连接特点：
- 限制在同一TAZ内的车站
- 距离不超过MAX_TRANSFER_WALKING_DISTANCE
- 建立双向连接支持任意方向换乘
- 包含步行时间和换乘惩罚

### 核心搜索功能

#### 主搜索函数
```go
func (r *Router) SearchBus(start, end *geov2.Position, time float64, availableSublineTypes []mapv2.SublineType) ([]*routingv2.WalkingRouteSegment, float64, []*routingv2.TransferSegment, float64, []*routingv2.WalkingRouteSegment, float64, error)
```

返回完整的多模态路径：
- **起点步行段**: 从起点到起始车站的步行路径
- **公交换乘段**: 车站间的公交/地铁路径
- **终点步行段**: 从终点车站到终点的步行路径

##### 同TAZ优化
```go
if r.isStartEndInSameTAZ(start, end) {
    startWalkSegments, startWalkCost, err := r.SearchWalking(start, end, time)
    return startWalkSegments, startWalkCost, nil, 0.0, nil, 0.0, err
}
```

同TAZ内直接使用步行导航，避免不必要的公交绕行。

#### 最佳起点车站搜索
```go
func (r *Router) findBestStartStation(start *geov2.Position, pEnd geometry.Point, time float64) (startStationIds []int32)
```

智能选择起点车站的策略：

##### 车道AOI优先
```go
if lanePosition := start.GetLanePosition(); lanePosition != nil {
    aoiIDs := r.lanes[lanePosition.GetLaneId()].AoiIds
    for _, aoiID := range aoiIDs {
        if r.aois[aoiID].IsStation {
            stationIDs = append(stationIDs, aoiID)
        }
    }
    if len(stationIDs) > 0 {
        return stationIDs[:min(len(stationIDs), SAME_TAZ_STATION_NODES)]
    }
}
```

##### 多TAZ候选策略
```go
for _, startTazPair := range algo.PointToNearTAZs(pStart, xStep, yStep, xMin, yMin) {
    // 对每个候选TAZ计算到终点的最小成本
    for _, stationID := range startTaz.StationIds {
        // 计算启发式成本 + 步行到车站的成本
        minCost += geometry.Distance(station.CenterPoint, pStart) / PERSON_SPEED
        candidateStationIDs = append(candidateStationIDs, stationID)
        candidateCosts = append(candidateCosts, minCost)
    }
}

// 按成本排序并去重
sort.Slice(candidateStationIDs, func(i, j int) bool {
    return candidateCosts[i] < candidateCosts[j]
})
```

#### 车站间换乘搜索
```go
func (r *Router) searchBusTransfer(startStation *Aoi, end *geov2.Position, endP geometry.Point, sameTazDistance float64, time float64, availableSublineTypes []mapv2.SublineType) (transferSegments []*routingv2.TransferSegment, timeCost float64, err error)
```

##### 同线路优化
```go
// 检查起终点是否在同一线路上
for _, sublineID := range startStation.SublineIds {
    followAoiIDs := subline.AoiIds[startIndex+1:]
    for index, followAoiID := range followAoiIDs {
        transferCost += (subline.Schedules.OffsetTimes[index+startIndex] + STATION_PASSBY_TIME)
        if followAoiID == endStationID {
            return sameLineTransferSegments, transferCost, nil
        }
    }
}
```

##### TAZ区域搜索
```go
path, cost := r.busGraph.ShortestTAZPath(startStationNodeID, 
    algo.PointToTaz(endP, xStep, yStep, xMin, yMin), 
    endP, sameTazDistance, time, availableSublineTypes)
```

使用专门的TAZ区域搜索算法，终点为区域而非固定点。

##### 路径段重构
```go
// 合并连续的同线路路径段
if edgeAttr.SublineID == currentLineID {
    endNodeID = edgeAttr.ToID
} else if stationPairs[StationIdPairs{endNodeID, edgeAttr.ToID}] || inSublineFlag {
    endNodeID = edgeAttr.ToID
} else {
    // 创建新的换乘段
    curSegment := &routingv2.TransferSegment{
        SublineId: currentLineID, 
        StartStationId: startNodeID, 
        EndStationId: endNodeID
    }
    transferSegments = append(transferSegments, curSegment)
}
```

##### 终点站优化
```go
// 寻找线路后续更近的下车站点
for _, aoiID := range followAoiIDs {
    followStation := r.aois[aoiID]
    if followDistance := geometry.Distance(followStation.CenterPoint, endP); 
       followDistance < endDistance && len(followStation.WalkingPositions) > 0 {
        endDistance = followDistance
        endStationID = aoiID
    }
}
```

### 辅助功能函数

#### 车站步行路径搜索
```go
func (r *Router) searchStationWalkRoute(routeStation *Aoi, position *geov2.Position, aoiId int32, time float64, routeToAoi bool) (walkSegments []*routingv2.WalkingRouteSegment, walkCost float64, ok bool)
```

处理车站与起终点间的步行连接：
- 检查步行时间是否在可接受范围内
- 支持双向路径搜索
- 自动过滤不可达的车站

#### 位置转换和检查函数
```go
func (r *Router) isStartServiceStation(start *geov2.Position, time float64) (bool, bool, *Aoi)
func (r *Router) aoiStationID(pb *geov2.Position) (int32, bool)
func (r *Router) positionToPoint(pb *geov2.Position) geometry.Point
func (r *Router) isStartEndInSameTAZ(start, end *geov2.Position) bool
```

提供位置分析和转换的基础功能。

### 设计特点

1. **多模态集成**: 无缝集成步行、公交、地铁多种交通方式
2. **TAZ智能搜索**: 基于交通分析区的区域搜索优化
3. **时刻表建模**: 完整支持公交时刻表和服务时间
4. **换乘优化**: 智能换乘策略和惩罚机制
5. **多候选搜索**: 同时考虑多个起点车站和距离阈值
6. **路径简化**: 自动合并同线路连续段，优化结果表示
7. **实时过滤**: 支持动态过滤可用的交通线路类型

### 复杂度管理

1. **分阶段搜索**: 起点车站选择 → 车站间换乘 → 终点步行
2. **启发式优化**: 复杂的公交专用启发式函数加速搜索
3. **缓存机制**: TAZ成本信息预计算和缓存
4. **阈值限制**: 多重时间和距离阈值避免搜索爆炸

### 限制说明

- 同线路检查可能在复杂换乘场景中有遗漏
- 车站TAZ成本信息需要预计算，更新成本较高
- 多候选搜索在密集车站区域可能影响性能
- 换乘惩罚为固定值，未考虑实际车站设施差异

## Server

路径规划服务器是系统的对外接口层，基于 gRPC/Connect 协议提供路径规划服务，支持多种路径类型、动态成本管理和服务控制功能。

### 核心数据结构

#### RoutingServer 结构体
```go
type RoutingServer struct {
    routingv2connect.UnimplementedRoutingServiceHandler
    router *router.Router    // 路径规划核心引擎
    ok     bool             // 服务开启/关闭状态
    cond   *sync.Cond       // 条件变量，用于暂停/恢复机制
}
```

服务器结构体特点：
- **嵌入式接口**: 实现 Connect/gRPC 服务接口
- **核心引擎**: 封装底层路径规划算法
- **状态管理**: 支持动态暂停和恢复服务
- **并发安全**: 使用条件变量管理并发访问

### 服务器初始化

#### 构造函数
```go
func NewRoutingServer(mongoURI string, mapPath, roadStatusPath *Path, cacheDir string) *RoutingServer
```

服务器初始化流程：

##### 数据库连接管理
```go
lazyClient := func() *mongo.Client {
    if client == nil {
        client = mongoutil.NewClient(mongoURI)
    }
    return client
}
defer func() {
    if client != nil {
        client.Disconnect(context.Background())
    }
}()
```

使用懒加载模式管理 MongoDB 连接，确保资源的合理使用。

##### 地图数据加载
```go
mapData, err := cache.LoadWithCache(cacheDir, mapPath, func() *mapv2.Map {
    pb, errs := mongoutil.DownloadPbFromMongo[mapv2.Map](
        context.Background(), mongoutil.GetMongoColl(lazyClient(), mapPath), nil, nil,
    )
    if len(errs) > 0 {
        log.Panicf("failed to download map from %s: %v", mapPath, errs)
    }
    return pb
})
```

地图数据加载特点：
- **缓存机制**: 使用本地缓存避免重复下载
- **失败处理**: 下载失败时程序终止，确保数据完整性
- **泛型支持**: 使用泛型简化 Protobuf 数据处理

##### 路况数据加载（可选）
```go
var roadStatuses *routingv2.RoadStatuses
if roadStatusPath != nil {
    roadStatuses, err = cache.LoadWithCache(cacheDir, roadStatusPath, func() *routingv2.RoadStatuses {
        // 类似地图数据的加载逻辑
    })
}
```

支持可选的实时路况数据，增强路径规划的准确性。

##### 路由引擎初始化
```go
r := router.New(mapData, roadStatuses)
return &RoutingServer{
    router: r,
    ok:     true, 
    cond:   sync.NewCond(&sync.Mutex{})
}
```

### 核心服务接口

#### 主路径查询接口
```go
func (s *RoutingServer) GetRoute(ctx context.Context, req *connect.Request[routingv2.GetRouteRequest]) (*connect.Response[routingv2.GetRouteResponse], error)
```

统一的路径查询接口，支持多种交通方式。

##### 暂停/恢复机制
```go
s.cond.L.Lock()
for !s.ok {
    // 暂停中，等待恢复信号
    s.cond.Wait()
}
s.cond.L.Unlock()
```

服务暂停机制特点：
- **非阻塞检查**: 快速检查服务状态
- **条件等待**: 暂停期间阻塞新请求
- **优雅恢复**: 恢复时唤醒所有等待的请求

##### 位置数据验证
```go
if err := CheckPosition(start); err != nil {
    return nil, err
}
if err := CheckPosition(end); err != nil {
    return nil, err
}
```

使用专门的验证函数确保输入数据格式正确。

##### ID 存在性检查
```go
if aoiPosition := start.GetAoiPosition(); aoiPosition != nil {
    if !s.router.HasAoiID(aoiPosition.AoiId) {
        return nil, connect.NewError(connect.CodeInvalidArgument, 
            fmt.Errorf("no start Aoi ID: %v", aoiPosition.AoiId))
    }
}
```

确保引用的 AOI 和 Lane ID 在系统中存在，避免无效查询。

##### 多路径类型支持

###### 驾车路径 (DRIVING/TAXI)
```go
case routingv2.RouteType_ROUTE_TYPE_DRIVING, routingv2.RouteType_ROUTE_TYPE_TAXI:
    var journeyType routingv2.JourneyType
    if in.GetType() == routingv2.RouteType_ROUTE_TYPE_DRIVING {
        journeyType = routingv2.JourneyType_JOURNEY_TYPE_DRIVING
    } else {
        journeyType = routingv2.JourneyType_JOURNEY_TYPE_BY_TAXI
    }
    
    if roadIDs, cost, err := s.router.SearchDriving(start, end, in.Time); err != nil {
        return connect.NewResponse(&routingv2.GetRouteResponse{}), nil
    } else {
        ret.Journeys = append(ret.Journeys, &routingv2.Journey{
            Type: journeyType,
            Driving: &routingv2.DrivingJourneyBody{
                RoadIds: roadIDs,
                Eta:     cost,
            },
        })
    }
```

驾车路径特点：
- **统一处理**: 驾车和出租车使用相同算法
- **道路验证**: 确保起点在有效的行车道上
- **错误容忍**: 无路径时返回空结果而非错误

###### 步行路径 (WALKING)
```go
case routingv2.RouteType_ROUTE_TYPE_WALKING:
    if route, cost, err := s.router.SearchWalking(start, end, in.Time); err != nil {
        return connect.NewResponse(&routingv2.GetRouteResponse{}), nil
    } else {
        ret.Journeys = append(ret.Journeys, &routingv2.Journey{
            Type: routingv2.JourneyType_JOURNEY_TYPE_WALKING,
            Walking: &routingv2.WalkingJourneyBody{
                Route: route,
                Eta:   cost,
            },
        })
    }
```

###### 公交路径 (BUS/SUBWAY/BUS_SUBWAY)
```go
case routingv2.RouteType_ROUTE_TYPE_BUS, routingv2.RouteType_ROUTE_TYPE_SUBWAY, routingv2.RouteType_ROUTE_TYPE_BUS_SUBWAY:
    var availableSublineTypes []mapv2.SublineType
    // 根据路径类型设置可用的线路类型
    if in.GetType() == routingv2.RouteType_ROUTE_TYPE_BUS {
        availableSublineTypes = []mapv2.SublineType{
            mapv2.SublineType_SUBLINE_TYPE_BUS, 
            mapv2.SublineType_SUBLINE_TYPE_UNSPECIFIED
        }
    }
    
    startWalkSegments, startWalkCost, transferSegment, transferCost, endWalkSegments, endWalkCost, err := s.router.SearchBus(start, end, in.Time, availableSublineTypes)
    
    // 构建多段路径
    if startWalkSegments != nil {
        ret.Journeys = append(ret.Journeys, /* 起点步行段 */)
    }
    if transferSegment != nil {
        ret.Journeys = append(ret.Journeys, /* 公交换乘段 */)
    }
    if endWalkSegments != nil {
        ret.Journeys = append(ret.Journeys, /* 终点步行段 */)
    }
```

公交路径特点：
- **类型过滤**: 根据查询类型过滤可用线路
- **多段组合**: 自动组合步行和公交段
- **灵活配置**: 支持纯公交、纯地铁或混合模式

### 动态成本管理

#### 设置道路成本
```go
func (s *RoutingServer) SetDrivingCosts(ctx context.Context, req *connect.Request[routingv2.SetDrivingCostsRequest]) (*connect.Response[routingv2.SetDrivingCostsResponse], error)
```

批量更新道路通行成本：
```go
for _, c := range in.Costs {
    if err := s.router.SetRoadCost(c.Id, c.Cost, c.Time); err != nil {
        return nil, connect.NewError(connect.CodeNotFound, err)
    }
}
```

特点：
- **批量处理**: 支持一次更新多条道路
- **即时生效**: 更新后立即影响后续路径查询
- **错误处理**: 任何道路ID不存在都会返回错误

#### 查询道路成本
```go
func (s *RoutingServer) GetDrivingCosts(ctx context.Context, req *connect.Request[routingv2.GetDrivingCostsRequest]) (*connect.Response[routingv2.GetDrivingCostsResponse], error)
```

批量查询道路通行成本：
```go
out := &routingv2.GetDrivingCostsResponse{Costs: in.Costs}
for _, c := range out.Costs {
    if cost, err := s.router.GetRoadCost(c.Id, c.Time); err != nil {
        return nil, connect.NewError(connect.CodeNotFound, err)
    } else {
        c.Cost = cost
    }
}
```

### 服务控制功能

#### 暂停服务
```go
func (s *RoutingServer) Suspend() {
    s.cond.L.Lock()
    defer s.cond.L.Unlock()
    s.ok = false
}
```

优雅暂停服务，新请求将被阻塞。

#### 恢复服务
```go
func (s *RoutingServer) Resume() {
    s.cond.L.Lock()
    defer s.cond.L.Unlock()
    s.ok = true
    s.cond.Broadcast()  // 唤醒所有等待的请求
}
```

恢复服务并唤醒所有等待的请求。

#### 关闭服务
```go
func (s *RoutingServer) Close() {
    s.router.Close()
}
```

彻底关闭服务，释放资源。

### 辅助功能

#### 位置数据验证
```go
func CheckPosition(pb *geov2.Position) error {
    if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
        return nil
    } else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
        return nil
    } else {
        return fmt.Errorf("no position data in request")
    }
}
```

确保位置数据包含有效的 AOI 或 Lane 位置信息。

### 设计特点

1. **统一接口**: 单一接口支持所有路径类型查询
2. **优雅降级**: 查询失败时返回空结果而非错误
3. **实时管理**: 支持动态更新道路成本
4. **并发安全**: 完整的并发控制和状态管理
5. **资源管理**: 自动管理数据库连接和缓存
6. **错误处理**: 详细的参数验证和错误返回
7. **服务控制**: 支持服务的暂停、恢复和关闭

### 性能优化

1. **懒加载**: 数据库连接按需创建
2. **本地缓存**: 地图数据本地缓存减少网络开销
3. **条件变量**: 高效的线程同步机制
4. **批量操作**: 成本查询和更新支持批量处理

### 限制说明

- 服务暂停期间所有请求都会被阻塞
- 地图数据更新需要重启服务
- 成本更新是同步操作，可能影响并发性能
- 错误处理采用快速失败策略，部分场景可能需要重试机制


## Main

主程序模块是路径规划系统的启动入口，负责配置管理、服务初始化、HTTP/gRPC服务器启动和优雅关闭等核心功能。采用现代化的服务架构，支持性能分析、基准测试和信号处理。

### 命令行参数配置

#### 核心配置参数
```go
var (
    mongoURI          = flag.String("mongo_uri", "", "mongo db uri")
    mapPathStr        = flag.String("map", "", "map database and collection [format: {fspath} or {db}.{col}]")
    roadStatusPathStr = flag.String("road-statuses", "", "roadstatuses database and collection, can be empty [format: {fspath} or {db}.{col}]")
    cacheDir          = flag.String("cache", "", "input cache dir path (empty means disable cache)")
    grpcEndpoint      = flag.String("listen", "localhost:52101", "gRPC listening address")
    logLevel          = flag.String("log-level", "info", "log level [debug, info, warn, error, fatal, panic]")
)
```

配置参数说明：
- **mongoURI**: MongoDB 数据库连接字符串
- **mapPathStr**: 地图数据路径，支持文件系统路径或数据库集合
- **roadStatusPathStr**: 路况数据路径（可选）
- **cacheDir**: 本地缓存目录路径
- **grpcEndpoint**: gRPC 服务监听地址
- **logLevel**: 日志级别控制

#### 性能测试参数
```go
var (
    benchmark = flag.Bool("benchmark", false, "benchmark mode")
    pprofAddr = flag.String("pprof", "localhost:52102", "pprof listening address")
)
```

支持独立的性能分析和基准测试功能。

#### 日志级别映射
```go
var LOG_LEVELS = map[string]logrus.Level{
    "debug": logrus.DebugLevel,
    "info":  logrus.InfoLevel,
    "warn":  logrus.WarnLevel,
    "error": logrus.ErrorLevel,
    "fatal": logrus.FatalLevel,
    "panic": logrus.PanicLevel,
}
```

提供灵活的日志级别配置。

### 主函数执行流程

#### 程序入口
```go
func main()
```

主函数包含完整的服务启动和管理流程。

##### 日志系统初始化
```go
logrus.SetFormatter(&easy.Formatter{
    TimestampFormat: "2006-01-02 15:04:05.0000",
    LogFormat:       "[%module%] [%time%] [%lvl%] %msg%\n",
})
flag.Parse()
if level, ok := LOG_LEVELS[*logLevel]; ok {
    logrus.SetLevel(level)
} else {
    logrus.Fatalf("invalid log level: %s", *logLevel)
}
```

日志系统特点：
- **统一格式**: 使用 easy.Formatter 提供一致的日志格式
- **时间戳精度**: 精确到毫秒的时间戳
- **模块化标识**: 支持模块名称标识
- **级别控制**: 动态配置日志输出级别

##### 路径配置解析
```go
mapPath, err := NewPath(*mapPathStr)
if err != nil {
    logrus.Fatalf("invalid map path: %s", err)
}
roadStatusPath, err := NewPath(*roadStatusPathStr)
if err != nil {
    logrus.Fatalf("invalid road status path: %s", err)
}
```

路径配置特点：
- **灵活格式**: 支持文件系统路径和数据库路径
- **错误处理**: 配置错误时立即终止程序
- **可选路况**: 路况数据为可选配置

##### 路径规划服务器启动
```go
server := NewRoutingServer(*mongoURI, mapPath, roadStatusPath, *cacheDir)
```

使用配置参数创建核心路径规划服务器实例。

##### 性能分析支持
```go
if *pprofAddr != "" {
    // 启动pprof
    startHTTPDebugger(*pprofAddr)
}
```

可选的 pprof 性能分析服务，用于生产环境性能监控。

##### 基准测试模式
```go
if *benchmark {
    // 性能测试
    runBenchmark(server)
    return
}
```

独立的基准测试模式，用于性能评估和优化。

### HTTP/gRPC 服务器配置

#### 服务处理器注册
```go
mux := http.NewServeMux()
mux.Handle(routingv2connect.NewRoutingServiceHandler(server))
```

注册核心的路径规划服务处理器。

#### gRPC 反射支持
```go
reflector := grpcreflect.NewStaticReflector(routingv2connect.RoutingServiceName)
mux.Handle(grpcreflect.NewHandlerV1(reflector))
mux.Handle(grpcreflect.NewHandlerV1Alpha(reflector))
```

gRPC 反射特点：
- **接口发现**: 支持客户端动态发现服务接口
- **调试便利**: 便于 Postman 等工具调试
- **版本兼容**: 支持 V1 和 V1Alpha 版本

#### HTTP/2 服务器启动
```go
s := &http.Server{
    Addr:    addr,
    Handler: h2c.NewHandler(mux, &http2.Server{}),
}
```

服务器特点：
- **HTTP/2 协议**: 使用现代化的 HTTP/2 协议
- **无TLS模式**: 使用 h2c (HTTP/2 Cleartext) 降低部署复杂度
- **高性能**: 支持多路复用和服务器推送

### 优雅关闭机制

#### 信号监听设置
```go
signalCh := make(chan os.Signal, 1)
signal.Notify(signalCh, syscall.SIGINT, syscall.SIGTERM)
```

监听标准的进程终止信号。

#### 优雅关闭处理
```go
go func() {
    <-signalCh
    log.Info("stopping...")
    go func() {
        <-signalCh
        os.Exit(1) // 强制结束
    }()
    // 退出connect-go
    s.Close()
    // 退出导航服务
    server.Close()
    os.Exit(0)
}()
```

优雅关闭特点：
- **信号处理**: 响应 SIGINT (Ctrl+C) 和 SIGTERM 信号
- **两级退出**: 第一次信号优雅关闭，第二次信号强制退出
- **资源清理**: 按顺序关闭 HTTP 服务器和路径规划服务
- **状态通知**: 记录关闭过程的日志信息

#### 服务器启动和监听
```go
log.Infof("server listening at %v", s.Addr)
if err := s.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
    log.Fatalf("failed to serve: %v", err)
}
time.Sleep(1 * time.Second) // 延迟等待"优雅退出"
log.Info("routing closes")
```

启动流程特点：
- **状态通知**: 记录服务器启动地址
- **错误过滤**: 区分正常关闭和异常错误
- **优雅延迟**: 等待清理操作完成
- **完成确认**: 记录服务最终关闭状态

### 程序架构特点

1. **模块化设计**: 清晰分离配置、初始化、服务和关闭逻辑
2. **灵活配置**: 丰富的命令行参数支持不同部署场景
3. **现代协议**: 采用 HTTP/2 和 gRPC Connect 提高性能
4. **完善监控**: 集成 pprof 性能分析和基准测试
5. **优雅管理**: 完整的服务生命周期管理
6. **错误处理**: 快速失败策略确保系统稳定性
7. **生产就绪**: 支持信号处理和优雅关闭

### 部署和运维

#### 典型启动命令
```bash
# 基础启动
./routing --mongo_uri="mongodb://localhost:27017" --map="db.collection" --listen=":52101"

# 带缓存和性能监控
./routing --mongo_uri="mongodb://localhost:27017" --map="db.collection" --cache="/tmp/cache" --pprof=":52102" --log-level="debug"

# 基准测试模式
./routing --mongo_uri="mongodb://localhost:27017" --map="db.collection" --benchmark
```

#### 配置文件格式支持
- **数据库路径**: `database.collection` 格式
- **文件系统路径**: 绝对或相对文件路径
- **可选参数**: 路况数据和缓存目录均为可选

### 性能和监控

1. **内存分析**: 通过 pprof 端口提供实时内存分析
2. **CPU分析**: 支持 CPU 使用情况监控
3. **基准测试**: 独立的性能测试模式
4. **日志监控**: 结构化日志支持运维监控
5. **健康检查**: 通过 gRPC 反射提供服务状态

### 限制说明

- 配置错误时程序立即终止，不支持热重载
- 基准测试模式下不启动正常服务
- pprof 端口需要独立配置，默认不启用
- 优雅关闭有1秒延迟，可能影响快速重启场景
