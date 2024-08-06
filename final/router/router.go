package router

import (
	"encoding/json"
	"fmt"
	"math/rand"
	"os"
	"runtime"
	"time"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

// Nodes, Edges, Distances, Grid, Landmarks, LandmarkNodes, LandmarkDistances
func FileReader() (nodes [][2]float64, edges [][4]int, distances [][4]int, grid [][][][3]float64,
	sorted_edges [][2]int, sorted_distances []int, start_indices []int,
	landmarkCoords [][2]float64, landmarks []int, landmarkMap map[int][]int) {

	slog.Info("Reading the files")
	graphNodesJSON, err := os.ReadFile("objects/graphNodes.json")
	if err != nil {
		slog.Info("Error reading graphNodes from file:", err)
	}
	var graphNodes [][2]float64
	err = json.Unmarshal(graphNodesJSON, &graphNodes)
	if err != nil {
		slog.Info("Error unmarshalling graphNodes:", err)
	}

	slog.Debug("Graph Nodes:", len(graphNodes))

	graphEdgesJSON, err := os.ReadFile("objects/graphEdges.json")
	if err != nil {
		slog.Info("Error reading graphEdges from file:", err)
	}
	var graphEdges [][4]int
	err = json.Unmarshal(graphEdgesJSON, &graphEdges)
	if err != nil {
		slog.Info("Error unmarshalling graphEdges:", err)
	}

	slog.Debug("Graph Edges:", len(graphEdges))

	distancesEdgesJSON, err := os.ReadFile("objects/distancesEdges.json")
	if err != nil {
		slog.Info("Error reading distancesEdges from file:", err)
	}
	var distancesEdges [][4]int
	err = json.Unmarshal(distancesEdgesJSON, &distancesEdges)
	if err != nil {
		slog.Info("Error unmarshalling distancesEdges:", err)
	}

	slog.Debug("Distances Edges:", len(distancesEdges))

	gridJSON, err := os.ReadFile("objects/grid.json")
	if err != nil {
		slog.Info("Error reading grid from file:", err)
	}
	var gridNodes [][][][3]float64
	err = json.Unmarshal(gridJSON, &gridNodes)
	if err != nil {
		slog.Info("Error unmarshalling grid:", err)
	}

	slog.Debug("Grid:", len(gridNodes))

	var landmarksCoords [][2]float64
	landmarksJSON, err := os.ReadFile("objects/landmarks.json")
	if err != nil {
		slog.Info("Error reading landmarks from file:", err)
	}
	err = json.Unmarshal(landmarksJSON, &landmarksCoords)
	if err != nil {
		slog.Info("Error unmarshalling landmarks:", err)
	}

	slog.Debug("Landmarks:", len(landmarksCoords))

	landmarkNodes := make([]int, numLandmarksConst)
	landmarkNodesJSON, err := os.ReadFile("objects/landmarkNodes.json")
	if err != nil {
		slog.Info("Error reading landmarkNodes from file:", err)
	}
	err = json.Unmarshal(landmarkNodesJSON, &landmarkNodes)
	if err != nil {
		slog.Info("Error unmarshalling landmarkNodes:", err)
	}

	landmarkDistances := make(map[int][]int)
	landmarkDistancesJSON, err := os.ReadFile("objects/landmarkDistances.json")
	if err != nil {
		slog.Info("Error reading landmarkDistances from file:", err)
	}
	err = json.Unmarshal(landmarkDistancesJSON, &landmarkDistances)
	if err != nil {
		slog.Info("Error unmarshalling landmarkDistances:", err)
	}

	slog.Debug("Landmark Nodes:", len(landmarkNodes))
	slog.Debug("Landmark Distances:", len(landmarkDistances))

	var sEdges [][2]int
	sortedEdgesJSON, err := os.ReadFile("objects/sortedEdges.json")
	if err != nil {
		slog.Info("Error reading sortedEdges from file:", err)
	}
	err = json.Unmarshal(sortedEdgesJSON, &sEdges)
	if err != nil {
		slog.Info("Error unmarshalling sortedEdges:", err)
	}

	slog.Debug("Sorted Edges:", len(sEdges))

	var sDistances []int
	sortedDistancesJSON, err := os.ReadFile("objects/sortedDistances.json")
	if err != nil {
		slog.Info("Error reading sortedDistances from file:", err)
	}
	err = json.Unmarshal(sortedDistancesJSON, &sDistances)
	if err != nil {
		slog.Info("Error unmarshalling sortedDistances:", err)
	}

	slog.Debug("Sorted Distances:", len(sDistances))

	var startIndices []int
	startIndicesJSON, err := os.ReadFile("objects/startIndices.json")
	if err != nil {
		slog.Info("Error reading startIndices from file:", err)
	}
	err = json.Unmarshal(startIndicesJSON, &startIndices)
	if err != nil {
		slog.Info("Error unmarshalling startIndices:", err)
	}

	slog.Debug("Start Indices:", len(startIndices))

	return graphNodes, graphEdges, distancesEdges, gridNodes, sEdges, sDistances, startIndices, landmarksCoords, landmarkNodes, landmarkDistances
}

func MultiRouter(iterations int) {

	fidgeter := chin.New()
	go fidgeter.Start()
	runtime.GOMAXPROCS(runtime.NumCPU())

	// graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()
	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()
	slog.Info("Multi Router started")
	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	runtime.GC()
	var startDijkstra = time.Now()
	for i := 0; i < iterations; i++ {
		path, dist := Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
		if dist[randomIndices[i][1]] <= 0 && len(path) == 0 {
			panic("Djikstra failed")
		}
	}

	avgDijkstra := time.Since(startDijkstra) / time.Duration(iterations)
	fmt.Println("Average Dijsktra time: ", avgDijkstra)

	runtime.GC()
	var startAStar = time.Now()
	for i := 0; i < iterations; i++ {
		path, dist := AStar(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
		if dist <= 0 && len(path) == 0 {
			panic("A* failed")
		}
	}

	avgAstar := time.Since(startAStar) / time.Duration(iterations)
	fmt.Println("Average AStar time: ", avgAstar)

	runtime.GC()
	var startALTv1 = time.Now()
	for i := 0; i < iterations; i++ {
		path, dist := ALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, randomIndices[i][0], randomIndices[i][1])
		if dist <= 0 && len(path) == 0 {
			panic("ALT failed")
		}
	}

	avgALTv1 := time.Since(startALTv1) / time.Duration(iterations)
	fmt.Println("Average ALT time: ", avgALTv1)

	// var startALTv2 = time.Now()
	// for i := 0; i < iterations; i++ {
	// 	ALTv2(graphNodes, graphEdges, distancesEdges, landmarkNodes, LandmarkDistances, randomIndices[i][0], randomIndices[i][1])
	// }

	// avgALTv2 := time.Since(startALTv2) / time.Duration(iterations)
	// fmt.Println("Average ALTv2 time: ", avgALTv2)

	fmt.Println("A* speedup percent: ", float64(avgDijkstra-avgAstar)/float64(avgDijkstra)*100)
	fmt.Println("ALTv1 speedup percent: ", float64(avgDijkstra-avgALTv1)/float64(avgDijkstra)*100)
	// fmt.Println("ALTv2 speedup percent: ", float64(avgDijkstra-avgALTv2)/float64(avgDijkstra)*100)

	fidgeter.Stop()

}

func SingleRouter(router string, iterations int) {
	slog.Info("Single Router started")
	runtime.GOMAXPROCS(runtime.NumCPU())
	fidgeter := chin.New()
	go fidgeter.Start()

	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()

	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	switch router {
	case "djikstra":
		slog.Info("Running Djikstra")
		var startDijkstra = time.Now()
		for i := 0; i < iterations; i++ {
			midStart := time.Now()
			path, dist := Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
			fmt.Println("Djikstra time Iteration: ", i, time.Since(midStart), "Distance: ", dist[randomIndices[i][1]])
			fmt.Println("Path: ", path[0], path[len(path)-1])
			fmt.Println("Path: ", path)
			fmt.Println("src: ", randomIndices[i][0], "dst: ", randomIndices[i][1])
		}

		fmt.Println("Average Dijsktra time: ", time.Since(startDijkstra)/time.Duration(iterations))

	case "astar":
		var startAStar = time.Now()
		for i := 0; i < iterations; i++ {
			// midStart := time.Now()
			AStar(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
			// fmt.Println("A* time Iteration: ", i, time.Since(midStart))
			// fmt.Println("Path: ", path[0], path[len(path)-1])
			// fmt.Println("Path: ", path)
			// fmt.Println("src: ", randomIndices[i][0], "dst: ", randomIndices[i][1])
		}

		fmt.Println("Average AStar time: ", time.Since(startAStar)/time.Duration(iterations))

	case "alt":
		var startALT = time.Now()
		for i := 0; i < iterations; i++ {
			midStart := time.Now()
			path, dist := ALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, randomIndices[i][0], randomIndices[i][1])
			fmt.Println("ALT time Iteration: ", i, time.Since(midStart), "Distance: ", dist)
			fmt.Println("Path: ", path[0], path[len(path)-1])
		}

		fmt.Println("Average ALT time: ", time.Since(startALT)/time.Duration(iterations))
	}
	fidgeter.Stop()
}

// results := make(chan types.Result, 3)
// 	var wg sync.WaitGroup

// 	// Run Dijkstra in a separate goroutine
// 	wg.Add(1)
// 	go func() {
// 		defer wg.Done()
// 		var totalDijkstraTime time.Duration
// 		for i := 0; i < iterations; i++ {
// 			startTime := time.Now()
// 			_, _ = Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
// 			totalDijkstraTime += time.Since(startTime)
// 		}
// 		avgDijkstra := totalDijkstraTime.Milliseconds() / int64(iterations)
// 		results <- types.Result{Algorithm: "Dijkstra", ShortestPath: []types.Point{}, TimeTaken: avgDijkstra}
// 	}()

// 	wg.Add(1)
// 	go func() {
// 		defer wg.Done()
// 		var totalAStarTime time.Duration
// 		for i := 0; i < iterations; i++ {
// 			startTime := time.Now()
// 			_, _ = AStar(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
// 			totalAStarTime += time.Since(startTime)
// 		}
// 		avgAStar := totalAStarTime.Milliseconds() / int64(iterations)
// 		results <- types.Result{Algorithm: "AStar", ShortestPath: []types.Point{}, TimeTaken: avgAStar}
// 	}()

// 	wg.Add(1)
// 	go func() {
// 		defer wg.Done()
// 		var totalALTTime time.Duration
// 		for i := 0; i < iterations; i++ {
// 			startTime := time.Now()
// 			_, _ = ALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, randomIndices[i][0], randomIndices[i][1])
// 			totalALTTime += time.Since(startTime)
// 		}
// 		avgALT := totalALTTime.Milliseconds() / int64(iterations)
// 		results <- types.Result{Algorithm: "ALT", ShortestPath: []types.Point{}, TimeTaken: avgALT}
// 	}()

// 	// Close the results channel when all goroutines are done
// 	go func() {
// 		wg.Wait()
// 		close(results)
// 	}()

// 	var avgDijkstra, avgAstar, avgALTv1 time.Duration

// 	// Collect results
// 	for result := range results {
// 		switch result.Algorithm {
// 		case "Dijkstra":
// 			avgDijkstra = time.Duration(result.TimeTaken)
// 		case "AStar":
// 			avgAstar = time.Duration(result.TimeTaken)
// 		case "ALT":
// 			avgALTv1 = time.Duration(result.TimeTaken)
// 		}
// 	}
