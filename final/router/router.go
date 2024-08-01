package router

import (
	"encoding/json"
	"fmt"
	"math/rand"
	"os"
	"time"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

// Nodes, Edges, Distances, Grid, Landmarks, LandmarkNodes, LandmarkDistances
func FileReader() ([][2]float64, [][4]int, [][4]int, [][][]int, [][2]float64, []int, map[int][]int) {
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
	var grid [][][]int
	err = json.Unmarshal(gridJSON, &grid)
	if err != nil {
		slog.Info("Error unmarshalling grid:", err)
	}

	slog.Debug("Grid:", len(grid))

	var landmarks [][2]float64
	landmarksJSON, err := os.ReadFile("objects/landmarks.json")
	if err != nil {
		slog.Info("Error reading landmarks from file:", err)
	}
	err = json.Unmarshal(landmarksJSON, &landmarks)
	if err != nil {
		slog.Info("Error unmarshalling landmarks:", err)
	}

	slog.Debug("Landmarks:", len(landmarks))

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

	return graphNodes, graphEdges, distancesEdges, grid, landmarks, landmarkNodes, landmarkDistances
}

func MultiRouter(iterations int) {
	slog.Info("Multi Router started")
	fidgeter := chin.New()
	go fidgeter.Start()

	graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes, LandmarkDistances := FileReader()

	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	var startDijkstra = time.Now()
	for i := 0; i < iterations; i++ {
		Djikstra(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
	}

	avgDijkstra := time.Since(startDijkstra) / time.Duration(iterations)
	fmt.Println("Average Dijsktra time: ", avgDijkstra)

	var startAStar = time.Now()
	for i := 0; i < iterations; i++ {
		AStar(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
	}

	avgAstar := time.Since(startAStar) / time.Duration(iterations)
	fmt.Println("Average AStar time: ", avgAstar)

	var startALTv1 = time.Now()
	for i := 0; i < iterations; i++ {
		ALT(graphNodes, graphEdges, distancesEdges, landmarkNodes, LandmarkDistances, randomIndices[i][0], randomIndices[i][1])
	}

	avgALTv1 := time.Since(startALTv1) / time.Duration(iterations)
	fmt.Println("Average ALT time: ", avgALTv1)

	var startALTv2 = time.Now()
	for i := 0; i < iterations; i++ {
		ALTv2(graphNodes, graphEdges, distancesEdges, landmarkNodes, LandmarkDistances, randomIndices[i][0], randomIndices[i][1])
	}

	avgALTv2 := time.Since(startALTv2) / time.Duration(iterations)
	fmt.Println("Average ALTv2 time: ", avgALTv2)

	fmt.Println("A* speedup percent: ", float64(avgDijkstra-avgAstar)/float64(avgDijkstra)*100)
	fmt.Println("ALTv1 speedup percent: ", float64(avgDijkstra-avgALTv1)/float64(avgDijkstra)*100)
	fmt.Println("ALTv2 speedup percent: ", float64(avgDijkstra-avgALTv2)/float64(avgDijkstra)*100)

	fidgeter.Stop()

}

func SingleRouter(router string, iterations int) {
	slog.Info("Single Router started")
	fidgeter := chin.New()
	go fidgeter.Start()

	graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes, landmarkDistances := FileReader()

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
			_, dist := Djikstra(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
			fmt.Println("Djikstra time Iteration: ", i, time.Since(midStart), "Distance: ", dist[randomIndices[i][1]])
		}

		fmt.Println("Average Dijsktra time: ", time.Since(startDijkstra)/time.Duration(iterations))

	case "astar":
		var startAStar = time.Now()
		for i := 0; i < iterations; i++ {
			midStart := time.Now()
			_, dist := AStar(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
			fmt.Println("A* time Iteration: ", i, time.Since(midStart), "Distance: ", dist)
		}

		fmt.Println("Average AStar time: ", time.Since(startAStar)/time.Duration(iterations))

	case "alt":
		var startALT = time.Now()
		for i := 0; i < iterations; i++ {
			midStart := time.Now()
			_, dist := ALT(graphNodes, graphEdges, distancesEdges, landmarkNodes, landmarkDistances, randomIndices[i][0], randomIndices[i][1])
			fmt.Println("ALT time Iteration: ", i, time.Since(midStart), "Distance: ", dist)
		}

		fmt.Println("Average ALT time: ", time.Since(startALT)/time.Duration(iterations))
	}
	fidgeter.Stop()
}
