package server

import (
	"fmt"
	"net/http"
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
	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, landmarkCoords, landmarkNodes, landmarkDistances := router.FileReader()

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

		var startTimeAStar = time.Now()
		shortestPathAStar, _ := router.AlgoAStar(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
		var timeTakenAStar = time.Since(startTimeAStar).Milliseconds()

		var startTimeDijkstra = time.Now()
		shortestPathDjikstra, _ := router.AlgoDijkstra(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
		var timeTakenDijkstra = time.Since(startTimeDijkstra).Milliseconds()

		// copier.Copy(&tempDist, &dist)
		var startTimeALT = time.Now()
		shortestPathALT, _ := router.AlgoALT(start, end, graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances)
		var timeTakenALT = time.Since(startTimeALT).Milliseconds()

		// fmt.Println(shortestPathALT)

		fmt.Println("AStar Time:", timeTakenAStar)
		fmt.Println("Dijkstra Time:", timeTakenDijkstra)
		fmt.Println("ALT Time:", timeTakenALT)

		c.JSON(http.StatusOK, gin.H{
			"astar_time":          timeTakenAStar,
			"dijkstra_time":       timeTakenDijkstra,
			"alt_time":            timeTakenALT,
			"shortest_path_astar": shortestPathAStar,
			"shortest_path_djik":  shortestPathDjikstra,
			"shortest_path_alt":   shortestPathALT,
		})
	})

	serv.POST("/djikstra", func(c *gin.Context) {
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

		var startTimeDijkstra = time.Now()
		shortestPathDjikstra, _ := router.AlgoDijkstra(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
		var timeTakenDijkstra = time.Since(startTimeDijkstra).Milliseconds()

		c.JSON(http.StatusOK, gin.H{
			"dijkstra_time":      timeTakenDijkstra,
			"shortest_path_djik": shortestPathDjikstra,
		})
	})

	serv.POST("/alt", func(c *gin.Context) {
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

		var startTimeALT = time.Now()
		shortestPathALT, _ := router.AlgoALT(start, end, graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances)
		var timeTakenALT = time.Since(startTimeALT).Milliseconds()

		c.JSON(http.StatusOK, gin.H{
			"alt_time":          timeTakenALT,
			"shortest_path_alt": shortestPathALT,
		})

	})

	serv.POST("/astar", func(c *gin.Context) {
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

		var startTimeAStar = time.Now()
		shortestPathAStar, _ := router.AlgoAStar(start, end, graphNodes, sortedEdges, sortedDistances, startIndices)
		var timeTakenAStar = time.Since(startTimeAStar).Milliseconds()

		c.JSON(http.StatusOK, gin.H{
			"astar_time":          timeTakenAStar,
			"shortest_path_astar": shortestPathAStar,
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
