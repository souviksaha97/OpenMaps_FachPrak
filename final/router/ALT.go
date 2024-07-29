package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
	"math/rand"

	"github.com/gookit/slog"
)

const numLandmarks = 40

func ALT(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, landmarks [][2]float64, src int, dst int) (int, []int) {
	dist := make(map[int]int)
	prev := make(map[int]int)
	found := false
	for node := range nodes {
		dist[node] = math.MaxInt32
	}
	dist[src] = 0

	pq := &types.PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &types.QueueItem{Node: src, Priority: 0})

	for pq.Len() > 0 {
		current := heap.Pop(pq).(*types.QueueItem)
		currentNode := current.Node

		if currentNode == dst {
			found = true
			break
		}

		for i, neighbor := range edges[currentNode] {
			if neighbor < 0 {
				break
			}
			heuristic := 0
			for _, landmark := range landmarks {
				heuristic = int(math.Max(float64(heuristic), generator.Haversine(nodes[neighbor][0], nodes[neighbor][1], landmark[0], landmark[1])-generator.Haversine(nodes[dst][0], nodes[dst][1], landmark[0], landmark[1])))
			}
			newDist := dist[currentNode] + edgeweights[currentNode][i] + heuristic
			if newDist < dist[neighbor] {
				dist[neighbor] = newDist
				prev[neighbor] = currentNode
				priority := newDist
				heap.Push(pq, &types.QueueItem{Node: neighbor, Priority: priority})
			}

		}

	}

	path := []int{}
	if found {
		for at := dst; at != src; at = prev[at] {
			path = append([]int{at}, path...)
		}
		path = append([]int{src}, path...)
	} else {
		path = append([]int{dst}, path...)
		path = append([]int{src}, path...)
		return -1, path
	}
	return dist[dst], path
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, landmarks [][2]float64, grid [][][]int) []types.Point {

	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	distpointStart := 100000000.0
	distpointEnd := 100000000.0
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	for k := range graphNodes {
		distpointStartNew := generator.Haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeStart[0], nearestnodeStart[1])
		distpointENdNew := generator.Haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distpointStartNew < float64(distpointStart) {
			nearestpointStartIndex = k
			distpointStart = distpointStartNew
		}
		if distpointENdNew < distpointEnd {
			nearpointEndIndex = k
			distpointEnd = distpointENdNew
		}
	}

	_, path := ALT(graphNodes[:], graphEdges[:], distancesEdges[:], landmarks, nearestpointStartIndex, nearpointEndIndex)

	returndykstrapath := [][2]float64{}
	for k := range path {
		returndykstrapath = append(returndykstrapath, graphNodes[path[k]])
	}

	// Convert returndykstrapath to []types.Point
	shortestPath := make([]types.Point, len(returndykstrapath))
	for i, point := range returndykstrapath {
		shortestPath[i] = types.Point{Lat: point[0], Lng: point[1]}
	}

	return shortestPath
}

func LandmarksDistanceMaximiser() {
	nodes, _, _, _, _ := FileReader()
	longSearch := false
	landmarks := make([]int, numLandmarks)
	maxDistance := 0.0
	if longSearch {
		for i := 0; i < len(nodes); i++ {
			for j := i + 1; j < len(nodes); j++ {
				distance := generator.Haversine(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
				if distance > maxDistance {
					maxDistance = distance
				}
			}
		}
	} else {
		// radius of earth in km
		maxDistance = 6350.0
	}

	slog.Info("Max distance: %f\n", maxDistance)

	for {
		res := chooseLandmarks(nodes, numLandmarks, int(maxDistance))
		if res == nil {
			maxDistance = maxDistance * 0.9
			fmt.Println("Trying with smaller distance: ", maxDistance)
		} else {
			landmarks = res
			break
		}
	}

	landmarksNodes := make([][2]float64, len(landmarks))
	for i, landmark := range landmarks {
		landmarksNodes[i][0] = nodes[landmark][0]
		landmarksNodes[i][1] = nodes[landmark][1]
	}
	generator.WriteToJSONFile("objects/landmarks.json", landmarksNodes)
}

func chooseLandmarks(nodes [][2]float64, numLandmarks int, minDistance int) []int {
	landmarks := make([]int, numLandmarks)
	landmarkCounter := 0
	validityCounter := 0
	for landmarkCounter < numLandmarks && validityCounter < 1000 {
		randomPoint := rand.Intn(len(nodes))
		suitablePoint := true
		if !generator.Contains(landmarks, randomPoint) {
			for _, landmark := range landmarks {
				if generator.Haversine(nodes[randomPoint][0], nodes[randomPoint][1], nodes[landmark][0], nodes[landmark][1]) < float64(minDistance) {
					suitablePoint = false
					validityCounter++
					break
				}
			}
			if suitablePoint {
				landmarks[landmarkCounter] = randomPoint
				landmarkCounter++
				validityCounter = 0
				slog.Info("Landmark", landmarkCounter, ":", randomPoint)
			}

		}
	}

	if validityCounter >= 1000 {
		slog.Info("Could not find suitable landmarks")
		return nil
	}
	return landmarks
}
