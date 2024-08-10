package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
	// "math/rand"
	"time"
	"github.com/adhocore/chin"
	"github.com/gookit/slog"
	// "runtime"		
)

// Nodes, Edges, Distances, Landmarks, LandmarkDistances, src, dst
// Single landmark heuristic
func ALT(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, sortedLandmarks map[int][]int, landmarkPairDistances [][]int, src int, dst int) ([]int, int) {

	// Initialize GraphData
	data := types.NewGraphData(len(nodes), src)
	heuristicTime := time.Duration(0)
	// fmt.Println("Distance array", landmarkDistances[landmarks[0]][src])
		
	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
			// fmt.Println("Found")
			break
		}

		startindex := startindicesmap[currentNode]
		endindex := startindicesmap[currentNode+1]
		for i := startindex; i < endindex; i++ {
			neighbor := edges[i][1]
			if neighbor < 0 {
				continue
			}

			closestLandmarkSrc := sortedLandmarks[currentNode][0]
			closestLandmarkDst := sortedLandmarks[dst][0]

			indexSrc := generator.FindIndex(landmarks, closestLandmarkSrc)
			indexDst := generator.FindIndex(landmarks, closestLandmarkDst)

			farthestLandmarkIndex := landmarks[landmarkPairDistances[indexSrc][indexDst]]

			heuristic := generator.Abs(landmarkDistances[farthestLandmarkIndex][dst] - landmarkDistances[farthestLandmarkIndex][currentNode])
			// farthestLandmark := landmarkPairDistances[[2]int{closestLandmarkSrc, closestLandmarkDst}]  + landmarkPairDistances[[2]int{closestLandmarkDst, closestLandmarkSrc}]

			newDist := data.Dist[currentNode] + edgeweights[i] + heuristic
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
			}
		}
	}

	fmt.Println("Time taken for ALT: ", heuristicTime)

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		for at := dst; at != -1; at = data.Prev[at] {
			path = append([]int{at}, path...)
		}
	}

	return path, data.Dist[dst]
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int,
	landmarks []int, landmarkDistances map[int][]int, sortedLandmarks map[int][]int, landmarkPairDistances [][]int) ([]types.Point, int) {
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

	path, dist := ALT(graphNodes, graphEdges, distancesEdges, startIndices, landmarks, landmarkDistances, sortedLandmarks, landmarkPairDistances, nearestpointStartIndex, nearpointEndIndex)
	// path, dist := ALTv2(graphNodes, graphEdges, distancesEdges, landmarks, landmarkDistances, nearestpointStartIndex, nearpointEndIndex)
	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}

	return shortestPath, dist
}

func LandmarksDistanceMaximiser() {
	// nodes, edges, distances, _, _, _, _ := FileReader()
	nodes, _, _, _, sorted_edges, sorted_distances, start_indices, _, _, _, _, _ := FileReader()

	
	landmarks := closestIndices(nodes)
	fmt.Println("Closest indices", landmarks)
	// landmarks := make([]int, len(res))
		// res := chooseLandmarksV2(nodes, edges, distances, numLandmarks, int(maxDistance))
	landmarksNodes := make([][2]float64, len(landmarks))

	landmarkPairDistances := make([][]int, len(landmarks))
	for i := range landmarkPairDistances {
		landmarkPairDistances[i] = make([]int, len(landmarks))
	}

	for i, landmark := range landmarks {
		landmarksNodes[i][0] = nodes[landmark][0]
		landmarksNodes[i][1] = nodes[landmark][1]
		slog.Info("Landmark", i, ":", landmarksNodes[i])
	}
	

	fidgetor := chin.New()
	go fidgetor.Start()

	completeLandmarksMap := make(map[int][]int)
	maxMinLandmarksDistanceMap := make(map[int][]int, len(nodes))
	for i, landmark := range landmarks {

		slog.Info("Landmark:", i, landmark)
		// _, dist := Djikstra(nodes, graphEdges, distancesEdges, landmark, -1)
		_, dist := Djikstra(nodes, sorted_edges, sorted_distances, start_indices, landmark, -1)
		completeLandmarksMap[landmark] = dist
		// fmt.Println("Distance from landmark", landmark, ":", dist)
	}

	for i := 0; i < len(nodes); i++ {
		minDistance := math.MaxInt64
		maxDistance := 0
		minIndex := 0
		maxIndex := 0
		for landmark, dist := range completeLandmarksMap {
			if dist[i] < minDistance {
				minDistance = dist[i]
				minIndex = landmark
			}
			if dist[i] > maxDistance {
				maxDistance = dist[i]
				maxIndex = landmark
			}
		}
		maxMinLandmarksDistanceMap[i] = []int{minIndex, maxIndex}
	}

	for i := 0; i < len(landmarks); i++ {
		for j := i+1; j < len(landmarks); j++ {
			farthestIndex := 0
			farthestDistance := 0
			for k, selectedLandmark := range landmarks {
				if selectedLandmark == landmarks[i] || selectedLandmark == landmarks[j] {
					continue
				}
				
				distance := completeLandmarksMap[selectedLandmark][landmarks[i]] + completeLandmarksMap[selectedLandmark][landmarks[j]]
				if distance > farthestDistance {
					farthestDistance = distance
					farthestIndex = k
				}
			}
			slog.Info("Landmark", landmarks[i], ":", landmarks[j], ":", farthestIndex)

			// slog.Info("Landmark Node", landmarks[i], ":", nodes[landmarks[i]], "Landmark Node", landmarks[j], ":", nodes[landmarks[j], "Farthest Node", farthestIndex, "Distance", farthestDistance)
			landmarkPairDistances[i][j] = farthestIndex
			landmarkPairDistances[j][i] = farthestIndex
		}
	}


	// fmt.Println("Landmark Pair Distances", landmarkPairDistances)
	fmt.Println(landmarkPairDistances[2][0], landmarkPairDistances[0][2])
	fmt.Println(landmarks[2], landmarks[0], landmarks[landmarkPairDistances[2][0]])
	// fmt.Println("Pair 2, 0", landmarkPairDistances[[2]int{3302094, 4044}])
	// fmt.Println("Pair 0, 2", landmarkPairDistances[[2]int{4044, 3302094}])

	generator.WriteToJSONFile("landmarkPairDistances.json", landmarkPairDistances)
	generator.WriteToJSONFile("landmarks.json", landmarksNodes)
	generator.WriteToJSONFile("landmarkNodes.json", landmarks)
	generator.WriteToJSONFile("landmarkDistances.json", completeLandmarksMap)
	generator.WriteToJSONFile("sortedLandmarks.json", maxMinLandmarksDistanceMap)
	fidgetor.Stop()
}

// func computePairwiseHeuristics(nodes [][2]float64, start int, end int, landmarks []int, completeLandmarksMap map[int][]int, pairwiseHeuristicsMap [][]int) {
// 	defer wg.Done()

// 	for i := start; i < end; i++ {
// 		slog.Debug("Computing pairwise heuristics for node", i)
// 		if i%100 == 0 {
// 			fmt.Println("Computing pairwise heuristics for node", i)
// 		}
// 			for j := i + 1; j < end; j++ {
// 			maxHeuristic := int(generator.Haversine(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1]))
// 			for _, landmark := range landmarks {
// 				heuristic := generator.Abs(completeLandmarksMap[landmark][i] - completeLandmarksMap[landmark][j])
// 				if heuristic > maxHeuristic {
// 					maxHeuristic = heuristic
// 				}
// 			}
// 			mu.Lock()
// 			pairwiseHeuristicsMap[[2]int{i, j}] = maxHeuristic
// 			mu.Unlock()
// 		}
// 	}
// }

func closestIndices(nodes [][2]float64) (indices []int){
	pointsArray := [][2]float64{
		{-60.0, -170.0},
		{10.0, -170.0},
		{80.0, -170.0},
		{80.0, 0.0},
		{80.0, 170.0},
		{10.0, 170.0},
		{-60.0, 170.0},
		{-60.0, 0.0},
	}
	

	set_indices := make([]int, len(pointsArray))

	
	for i, point := range pointsArray{
		nearestnode := [2]float64{point[0], point[1]}
		nearestpointIndex := -1
		distpoint := math.MaxFloat64

		for k, node := range nodes {
			dist := generator.Haversine(node[0], node[1], nearestnode[0], nearestnode[1])
			if dist < distpoint {
				nearestpointIndex = k
				distpoint = dist
			}
		}

		set_indices[i] = nearestpointIndex
	}

	return set_indices

}