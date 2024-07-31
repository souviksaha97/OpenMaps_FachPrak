package tester

import (
	"final/router"
	"fmt"
	"math"
	"time"

	"github.com/gookit/slog"

	"math/rand"

	"github.com/adhocore/chin"
)

func Tester(iterations int) {
	// run multiple iterations of alt, with increasing number of landmarks and print the optimal number of landmarks

	// read the files
	fidgeter := chin.New()
	go fidgeter.Start()
	nodes, _, _, _, _, _, _ := router.FileReader()
	timingMap := make(map[int]time.Duration)

	// generate iteration random pairs of src and dst
	srcDstPairs := [][2]int{}
	for i := 0; i < iterations; i++ {
		src := rand.Intn(len(nodes))
		dst := rand.Intn(len(nodes))
		srcDstPairs = append(srcDstPairs, [2]int{src, dst})
	}
	slog.Info("Generated random pairs of src and dst")
	// run alt with increasing number of landmarks from 0.001% to 0.1% of total nodes
	// and calculate the average time taken for each number of landmarks
	// print the optimal number of landmarks
	optimalLandmarks := 0
	optimalTime := math.MaxInt64
	for i := 1; i <= 400; i *= 4 {
		numLandmarks := i
		slog.Info("___________________________________________________________________________")
		slog.Info("Number of landmarks: ", numLandmarks)
		router.LandmarksDistanceMaximiser(numLandmarks)
		slog.Info("Landmarks distance maximiser finished")
		nodes, edges, edgeweights, _, _, landmarkNodes, landmarkDistances := router.FileReader()
		var totalTime time.Duration
		slog.Info("Running ALT")
		for _, pair := range srcDstPairs {
			iterTime := time.Now()
			router.ALT(nodes, edges, edgeweights, landmarkNodes, landmarkDistances, pair[0], pair[1])
			totalTime += time.Since(iterTime)
		}
		averageTime := totalTime / time.Duration(len(srcDstPairs))
		//convert average time to float64
		averageTimeInt := time.Duration.Milliseconds(averageTime)

		timingMap[numLandmarks] = averageTime

		if averageTimeInt < int64(optimalTime) {
			optimalTime = int(averageTimeInt)
			optimalLandmarks = numLandmarks
		}
		fmt.Println("Number of landmarks: ", numLandmarks, "Average time: ", averageTime)
		slog.Info("___________________________________________________________________________")
	}
	slog.Info("Optimal number of landmarks: %d\n", optimalLandmarks)
	fmt.Println("Timing Map: ", timingMap)
	fidgeter.Stop()
}
