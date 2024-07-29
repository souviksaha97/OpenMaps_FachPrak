package router

import (
	"encoding/json"
	"math/rand"
	"os"
	"time"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

func FileReader() ([][2]float64, [][4]int, [][4]int, [][][]int, [][2]float64) {
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

	slog.Info("Graph Nodes:", len(graphNodes))

	graphEdgesJSON, err := os.ReadFile("objects/graphEdges.json")
	if err != nil {
		slog.Info("Error reading graphEdges from file:", err)
	}
	var graphEdges [][4]int
	err = json.Unmarshal(graphEdgesJSON, &graphEdges)
	if err != nil {
		slog.Info("Error unmarshalling graphEdges:", err)
	}

	slog.Info("Graph Edges:", len(graphEdges))

	distancesEdgesJSON, err := os.ReadFile("objects/distancesEdges.json")
	if err != nil {
		slog.Info("Error reading distancesEdges from file:", err)
	}
	var distancesEdges [][4]int
	err = json.Unmarshal(distancesEdgesJSON, &distancesEdges)
	if err != nil {
		slog.Info("Error unmarshalling distancesEdges:", err)
	}

	slog.Info("Distances Edges:", len(distancesEdges))

	gridJSON, err := os.ReadFile("objects/grid.json")
	if err != nil {
		slog.Info("Error reading grid from file:", err)
	}
	var grid [][][]int
	err = json.Unmarshal(gridJSON, &grid)
	if err != nil {
		slog.Info("Error unmarshalling grid:", err)
	}

	slog.Info("Grid:", len(grid))

	var landmarks [][2]float64
	landmarksJSON, err := os.ReadFile("objects/landmarks.json")
	if err != nil {
		slog.Info("Error reading landmarks from file:", err)
	}
	err = json.Unmarshal(landmarksJSON, &landmarks)
	if err != nil {
		slog.Info("Error unmarshalling landmarks:", err)
	}

	slog.Info("Landmarks:", len(landmarks))

	return graphNodes, graphEdges, distancesEdges, grid, landmarks
}

func MultiRouter(iterations int) {
	slog.Info("Router started")
	fidgeter := chin.New()
	go fidgeter.Start()

	graphNodes, graphEdges, distancesEdges, _, landmarks := FileReader()

	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	var startDijkstra = time.Now()
	for i := 0; i < iterations; i++ {
		Djikstra(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
	}
	slog.Info("Average Dijsktra time: ", time.Since(startDijkstra)/time.Duration(iterations))

	var startAStar = time.Now()
	for i := 0; i < iterations; i++ {
		AStar(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
	}

	slog.Info("Average AStar time: ", time.Since(startAStar)/time.Duration(iterations))

	var startALT = time.Now()
	for i := 0; i < iterations; i++ {
		ALT(graphNodes, graphEdges, distancesEdges, landmarks, randomIndices[i][0], randomIndices[i][1])
	}

	slog.Info("Average ALT time: ", time.Since(startALT)/time.Duration(iterations))

	fidgeter.Stop()

}
