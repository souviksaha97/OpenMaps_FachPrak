package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"
	// "github.com/gookit/slog"
)

// Djikstra implements the Dijkstra algorithm
func Djikstra(nodes [][2]float64, edges [][2]int, edgeweights []int, startindicesmap []int, src int, dst int) ([]int, []int, int) {

	data := types.NewGraphData(len(nodes), src)
	popCounter := 0
	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		popCounter += 1
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
			break
		}

		startindex := startindicesmap[currentNode]
		endindex := startindicesmap[currentNode+1]
		for i := startindex; i < endindex; i++ {
			neighbor := edges[i][1]
			if neighbor < 0 {
				continue
			}
			newDist := data.Dist[currentNode] + edgeweights[i]
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
			}
		}
	}
	// fmt.Println("Time taken for Djikstra: ", time.Since(timeStart))

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		// Build the path in reverse order
		for at := dst; at != -1; at = data.Prev[at] {
			path = append(path, at)
		}
	}

	return path, data.Dist, popCounter
}

func AlgoDijkstra(Start types.Point, End types.Point, graphNodes [][2]float64, gridNodes [][][][3]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int) ([]types.Point, int, int) {
	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	distpointStart := math.MaxInt64
	distpointEnd := math.MaxInt64

	// Find the nearest start and end nodes
	// for k, node := range graphNodes {
	// 	distStart := generator.Haversine(node[0], node[1], nearestnodeStart[0], nearestnodeStart[1])
	// 	distEnd := generator.Haversine(node[0], node[1], nearestnodeEnd[0], nearestnodeEnd[1])

	// 	if distStart < distpointStart {
	// 		nearestpointStartIndex = k
	// 		distpointStart = distStart
	// 	}
	// 	if distEnd < distpointEnd {
	// 		nearpointEndIndex = k
	// 		distpointEnd = distEnd
	// 	}
	// 	if distpointStart < 30000 && distpointEnd < 30000 {
	// 		break
	// 	}
	// }

	a, b := generator.FindRowAndColumnInGrid(180, 360, Start.Lat, Start.Lng)
	possiblestartandendpoints := gridNodes[a][b]
	c, d := generator.FindRowAndColumnInGrid(180, 360, End.Lat, End.Lng)
	possiblestartandendpoints = append(possiblestartandendpoints, gridNodes[c][d]...)

	for _, node := range possiblestartandendpoints {
		distStart := generator.Haversine(node[0], node[1], nearestnodeStart[0], nearestnodeStart[1])
		distEnd := generator.Haversine(node[0], node[1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distStart < distpointStart {
			nearestpointStartIndex = int(node[2])
			distpointStart = distStart
		}
		if distEnd < distpointEnd {
			nearpointEndIndex = int(node[2])
			distpointEnd = distEnd
		}
	}

	path, dist, popCounter := Djikstra(graphNodes, graphEdges, distancesEdges, startIndices, nearestpointStartIndex, nearpointEndIndex)

	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}
	return shortestPath, dist[nearpointEndIndex], popCounter
}
