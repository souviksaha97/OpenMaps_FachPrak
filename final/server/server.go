package server

import (
	"fmt"
	"log/slog"
	"net/http"
	"runtime"
	"strconv"
	"sync"
	"time"

	"final/router"
	"final/types"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
)

func Server() {

	serv := gin.Default()

	// Set up CORS middleware
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true // This allows all origins, you can specify the allowed origins if needed
	config.AllowMethods = []string{"GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"}
	config.AllowHeaders = []string{"Origin", "Content-Type", "Accept"}

	serv.Use(cors.New(config))

	// Read the files
	// graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, landmarkCoords, landmarkNodes, landmarkDistances, sortedLandmarks,  landmarkPairDistances:= router.FileReader()
	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, landmarkCoords, _, _, _, _ := router.FileReader()

	serv.POST("/submit_points", func(c *gin.Context) {
		var requestData map[string]types.Point
		if err := c.BindJSON(&requestData); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid points"})
			return
		}

		start, startOk := requestData["start"]
		end, endOk := requestData["end"]
		if !startOk || !endOk {
			c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid points"})
			return
		}

		fmt.Printf("Received start point: %+v\n", start)
		fmt.Printf("Received end point: %+v\n", end)

		runtime.GC()
		results := make(chan types.Result, 3)
		var wg sync.WaitGroup

		// Run Dijkstra in a separate goroutine
		wg.Add(1)
		go func() {
			defer wg.Done()
			startTime := time.Now()
			shortestPath, dist := router.AlgoDijkstra(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
			timeTaken := time.Since(startTime).Milliseconds()
			// results <- types.Result{Algorithm="Dijkstra", shoshortestPath, timeTaken}
			results <- types.Result{
				Algorithm:    "Dijkstra",
				ShortestPath: shortestPath,
				TimeTaken:    timeTaken,
			}
			slog.Info("Dijkstra distance: " + strconv.Itoa(dist))
		}()

		// Run A* in a separate goroutine
		wg.Add(1)
		go func() {
			defer wg.Done()
			startTime := time.Now()
			shortestPath, dist := router.AlgoAStar(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
			timeTaken := time.Since(startTime).Milliseconds()
			results <- types.Result{
				Algorithm:    "AStar",
				ShortestPath: shortestPath,
				TimeTaken:    timeTaken,
			}
			slog.Info("A* distance: " + strconv.Itoa(dist))
		}()

		// Run ALT in a separate goroutine
		// wg.Add(1)
		// go func() {
		// 	defer wg.Done()
		// 	startTime := time.Now()
		// 	shortestPath, _ := router.AlgoALT(start, end, graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, sortedLandmarks,landmarkPairDistances)
		// 	timeTaken := time.Since(startTime).Milliseconds()
		// 	results <- types.Result{
		// 		Algorithm:    "ALT",
		// 		ShortestPath: shortestPath,
		// 		TimeTaken:    timeTaken,
		// 	}
		// }()

		// Close the results channel when all goroutines are done
		go func() {
			wg.Wait()
			close(results)
			runtime.GC()
		}()

		dijkstraResult := types.Result{}
		astarResult := types.Result{}
		// altResult := types.Result{}

		// Collect results
		for result := range results {
			switch result.Algorithm {
			case "Dijkstra":
				dijkstraResult = result
			case "AStar":
				astarResult = result
				// case "ALT":
				// 	altResult = result
			}
		}

		// Respond with the results
		c.JSON(http.StatusOK, gin.H{
			"astar_time":    astarResult.TimeTaken,
			"dijkstra_time": dijkstraResult.TimeTaken,
			// "alt_time":            altResult.TimeTaken,
			"shortest_path_astar": astarResult.ShortestPath,
			"shortest_path_djik":  dijkstraResult.ShortestPath,
			// "shortest_path_alt":   altResult.ShortestPath,
		})
	})

	serv.GET("/landmarks", func(c *gin.Context) {
		// convert landmarks to json
		landmark_json := make([]types.Point, len(landmarkCoords))
		for i := range landmarkCoords {
			landmark_json[i] = types.Point{Lat: landmarkCoords[i][0], Lng: landmarkCoords[i][1]}
		}
		c.JSON(http.StatusOK, gin.H{"landmarks": landmark_json})
	})

	serv.Run(":5000")
}
