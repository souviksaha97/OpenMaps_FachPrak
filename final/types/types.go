package types

import (
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
type QueueItem struct {
	Node     int
	Priority int
}
type PriorityQueue []*QueueItem

// Point represents a geographical point with latitude and longitude
type Point struct {
	Lat float64 `json:"lat"`
	Lng float64 `json:"lng"`
}

func (pq PriorityQueue) Len() int { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].Priority < pq[j].Priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
}

func (pq *PriorityQueue) Push(x interface{}) {
	*pq = append(*pq, x.(*QueueItem))
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}
