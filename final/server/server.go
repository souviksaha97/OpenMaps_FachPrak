// Runs server to execute queries from the webpage

package server

import (
	"fmt"
	"net/http"
	"runtime"
	"strconv"
	"sync"
	"time"

	"github.com/gookit/slog"

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
	graphNodes, _, _, gridNodes, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := router.FileReader()

	// Update this if you want to use a different number of landmarks
	usedLandmarks := make([]int, 5)
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
			shortestPath, dist, popCounter := router.AlgoDijkstra(start, end, graphNodes, gridNodes, sortedEdges, sortedDistances, startIndices)
			timeTaken := time.Since(startTime).Milliseconds()
			// results <- types.Result{Algorithm="Dijkstra", shoshortestPath, timeTaken}
			results <- types.Result{
				Algorithm:    "Dijkstra",
				ShortestPath: shortestPath,
				TimeTaken:    timeTaken,
				PopCounter:   popCounter,
				Distance:     dist,
			}

			slog.Info("Dijkstra distance: " + strconv.Itoa(dist))
			slog.Info("Shortest Path Length: " + strconv.Itoa(len(shortestPath)))
		}()

		// Run A* in a separate goroutine
		wg.Add(1)
		go func() {
			defer wg.Done()
			startTime := time.Now()
			shortestPath, dist, popCounter := router.AlgoAStar(start, end, graphNodes, gridNodes, sortedEdges, sortedDistances, startIndices)
			timeTaken := time.Since(startTime).Milliseconds()
			results <- types.Result{
				Algorithm:    "AStar",
				ShortestPath: shortestPath,
				TimeTaken:    timeTaken,
				PopCounter:   popCounter,
				Distance:     dist,
			}
			slog.Info("A* distance: " + strconv.Itoa(dist))
			slog.Info("Shortest Path Length: " + strconv.Itoa(len(shortestPath)))
		}()

		// Run ALT in a separate goroutine
		wg.Add(1)
		go func() {
			defer wg.Done()
			startTime := time.Now()
			shortestPath, dist, popCounter, usedLandmarksTemp := router.AlgoALT(start, end, graphNodes, gridNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances)
			timeTaken := time.Since(startTime).Milliseconds()
			results <- types.Result{
				Algorithm:    "ALT",
				ShortestPath: shortestPath,
				TimeTaken:    timeTaken,
				PopCounter:   popCounter,
				Distance:     dist,
			}
			usedLandmarks = usedLandmarksTemp
			slog.Info("ALT distance: " + strconv.Itoa(dist))
			slog.Info("Shortest Path Length: " + strconv.Itoa(len(shortestPath)))
			slog.Info("Pop Counter:", popCounter)
			slog.Info("Used Landmarks:", usedLandmarks)
		}()

		// Close the results channel when all goroutines are done
		go func() {
			wg.Wait()
			close(results)
			runtime.GC()
		}()

		dijkstraResult := types.Result{}
		astarResult := types.Result{}
		altResult := types.Result{}

		// Collect results
		for result := range results {
			switch result.Algorithm {
			case "Dijkstra":
				dijkstraResult = result
			case "AStar":
				astarResult = result
			case "ALT":
				altResult = result
			}
		}

		// Respond with the results
		c.JSON(http.StatusOK, gin.H{
			"astar_time":          astarResult.TimeTaken,
			"dijkstra_time":       dijkstraResult.TimeTaken,
			"alt_time":            altResult.TimeTaken,
			"shortest_path_astar": astarResult.ShortestPath,
			"shortest_path_djik":  dijkstraResult.ShortestPath,
			"shortest_path_alt":   altResult.ShortestPath,
			"dijkstra_pops":       dijkstraResult.PopCounter,
			"astar_pops":          astarResult.PopCounter,
			"alt_pops":            altResult.PopCounter,
			"dijkstra_dist":       dijkstraResult.Distance,
			"astar_dist":          astarResult.Distance,
			"alt_dist":            altResult.Distance,
		})
	})

	// Get landmarks
	serv.GET("/landmarks", func(c *gin.Context) {
		// convert landmarks to json
		landmark_json := make([]types.Point, len(usedLandmarks))
		for i, landmark := range usedLandmarks {
			landmark_json[i] = types.Point{Lat: graphNodes[landmark][0], Lng: graphNodes[landmark][1]}
		}
		c.JSON(http.StatusOK, gin.H{"landmarks": landmark_json})
	})

	serv.Run(":5000")
}
