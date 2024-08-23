package types

import (
	"container/heap"
	"math"

	"github.com/paulmach/osm"
)

type RandomNode struct {
	lat float64
	lng float64
}
type Node struct {
	ID          int64
	Lat         float64
	Lon         float64
	IsCoastline bool
}
type Edge struct {
	Startpoint [2]float64
	Endpoint   [2]float64
	WayId      osm.NodeID
}
type WayComplete struct {
	WayMapKey  osm.NodeID
	IsFinished bool
}

// Point represents a geographical point with latitude and longitude
type Point struct {
	Lat float64 `json:"lat"`
	Lng float64 `json:"lng"`
}

// QueueItem represents an item in the priority queue.
type QueueItem struct {
	Node     int // The node index
	Priority int // The priority of the node
	Index    int // The index of the item in the heap
}

// PriorityQueue is a wrapper to handle QueueItem with heap.Interface methods
type PriorityQueue []*QueueItem

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].Priority < pq[j].Priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*QueueItem)
	item.Index = n
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil // avoid memory leak
	item.Index = -1
	*pq = old[0 : n-1]
	return item
}

// Comment out the unused method
// func (pq *PriorityQueue) update(item *QueueItem, priority float32) {
// 	item.Priority = priority
// 	heap.Fix(pq, item.Index)
// }

// GraphData holds the initialization data for the graph
type GraphData struct {
	Dist    []int
	Prev    []int
	Visited []bool
	PQ      *PriorityQueue
}

type Job struct {
	Points     [][3]float64
	Neighbours [][3]float64
	WrapAround bool
}

type EdgeWithDistance struct {
	Edge     [2]int
	Distance int
}

type Result struct {
	Algorithm    string
	ShortestPath []Point
	TimeTaken    int64
}

// NewGraphData initializes and returns GraphData
func NewGraphData(numNodes int, src int) *GraphData {
	dist := make([]int, numNodes)
	prev := make([]int, numNodes)
	prio := make([]int, numNodes)
	visited := make([]bool, numNodes)
	for i := range dist {
		dist[i] = math.MaxInt32
		prio[i] = math.MaxInt32
		prev[i] = -1
	}
	dist[src] = 0
	prio[src] = 0

	pq := &PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &QueueItem{Node: src, Priority: 0})

	return &GraphData{
		Dist:    dist,
		Prev:    prev,
		Visited: visited,
		PQ:      pq,
	}
}
