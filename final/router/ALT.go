package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
	"math/rand"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

const numLandmarks = 5

func ALT(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, landmarks [][2]float64, src int, dst int) ([]int, int) {
	// Initialize GraphData
	data := types.NewGraphData(len(nodes), src)

	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		// If the node is already visited, skip it
		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		// If we reached the destination, exit the loop
		if currentNode == dst {
			break
		}

		// Iterate over neighbors
		for i, neighbor := range edges[currentNode] {
			if neighbor < 0 || data.Visited[neighbor] {
				continue
			}

			heuristic := 0
			for _, landmark := range landmarks {
				heuristic = int(math.Max(float64(heuristic), generator.Haversine(nodes[neighbor][0], nodes[neighbor][1], landmark[0], landmark[1])-generator.Haversine(nodes[dst][0], nodes[dst][1], landmark[0], landmark[1])))
			}

			newDist := data.Dist[currentNode] + edgeweights[currentNode][i] + heuristic
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				priority := newDist
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: priority})
			}
		}
	}

	// Reconstruct the path
	path := []int{}
	if data.Prev[dst] != -1 || src == dst {
		for at := dst; at != -1; at = data.Prev[at] {
			path = append([]int{at}, path...)
		}
	}

	return path, data.Dist[dst]
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, landmarks [][2]float64, grid [][][]int) ([]types.Point, int) {
	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	distpointStart := math.MaxFloat64
	distpointEnd := math.MaxFloat64

	// Find the nearest start and end nodes
	for k, node := range graphNodes {
		distStart := generator.Haversine(node[0], node[1], nearestnodeStart[0], nearestnodeStart[1])
		distEnd := generator.Haversine(node[0], node[1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distStart < distpointStart {
			nearestpointStartIndex = k
			distpointStart = distStart
		}
		if distEnd < distpointEnd {
			nearpointEndIndex = k
			distpointEnd = distEnd
		}
	}

	path, dist := ALT(graphNodes, graphEdges, distancesEdges, landmarks, nearestpointStartIndex, nearpointEndIndex)

	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}

	return shortestPath, dist
}

func LandmarksDistanceMaximiser() {
	nodes, _, _, _, _, _ := FileReader()
	longSearch := false

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
		// circumference of earth in km
		maxDistance = 6350.0
	}

	slog.Info("Max distance: ", maxDistance)

	landmarks := make([]int, numLandmarks)
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

	generator.WriteToJSONFile("objects/landmarkNodes.json", landmarks)

	for i, landmark := range landmarks {
		landmarksNodes[i][0] = nodes[landmark][0]
		landmarksNodes[i][1] = nodes[landmark][1]
	}
	generator.WriteToJSONFile("objects/landmarks.json", landmarksNodes)

	fidgetor := chin.New()
	go fidgetor.Start()
	landMarksDistanceFinder()
	fidgetor.Stop()
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

func landMarksDistanceFinder() {
	graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes := FileReader()
	completeLandmarksMap := make(map[int][]int)
	for i, landmark := range landmarkNodes {
		slog.Info("Landmark:", i)
		_, dist := Djikstra(graphNodes, graphEdges, distancesEdges, landmark, -1)
		completeLandmarksMap[i] = dist
	}
	generator.WriteToJSONFile("objects/landmarkDistances.json", completeLandmarksMap)
}
