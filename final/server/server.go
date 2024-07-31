package server

import (
	"net/http"
	"time"

	"final/router"
	"final/types"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/gookit/slog"
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
	graphNodes, graphEdges, distancesEdges, grid, landmarks, landmarkNodes, landmarkDistances := router.FileReader()

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

		// fmt.Printf("Received start point: %+v\n", start)
		// fmt.Printf("Received end point: %+v\n", end)

		// Call the Algo function
		// copier.CopyWithOption(&tempDist, &dist, copier.Option{DeepCopy: true})
		// copier.Copy(&tempDist, &dist)
		var startTimeAStar = time.Now()
		shortestPathAStar, _ := router.AlgoAStar(start, end, graphNodes, graphEdges, distancesEdges, grid)
		var timeTakenAStar = time.Since(startTimeAStar).Milliseconds()
		// copier.CopyWithOption(&tempDist, &dist, copier.Option{DeepCopy: true})
		// copier.Copy(&tempDist, &dist)
		var startTimeDijkstra = time.Now()
		shortestPathDjikstra, _ := router.AlgoDijkstra(start, end, graphNodes, graphEdges, distancesEdges, grid)
		var timeTakenDijkstra = time.Since(startTimeDijkstra).Milliseconds()

		// copier.Copy(&tempDist, &dist)
		var startTimeALT = time.Now()
		shortestPathALT, _ := router.AlgoALT(start, end, graphNodes, graphEdges, distancesEdges, landmarkNodes, landmarkDistances, grid)
		var timeTakenALT = time.Since(startTimeALT).Milliseconds()

		slog.Info("AStar Time:", timeTakenAStar)
		slog.Info("Dijkstra Time:", timeTakenDijkstra)
		slog.Info("ALT Time:", timeTakenALT)
		// slog.Info("Shortest Path:", shortestPath)

		c.JSON(http.StatusOK, gin.H{
			"astar_time":          timeTakenAStar,
			"dijkstra_time":       timeTakenDijkstra,
			"alt_time":            timeTakenALT,
			"shortest_path_astar": shortestPathAStar,
			"shortest_path_djik":  shortestPathDjikstra,
			"shortest_path_alt":   shortestPathALT,
		})
	})

	serv.GET("/landmarks", func(c *gin.Context) {
		// convert landmarks to json
		landmark_json := make([]types.Point, len(landmarks))
		for i := range landmarks {
			landmark_json[i] = types.Point{Lat: landmarks[i][0], Lng: landmarks[i][1]}
		}
		c.JSON(http.StatusOK, gin.H{"landmarks": landmark_json})
	})

	serv.Run(":5000")
}
