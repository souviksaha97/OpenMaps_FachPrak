package main

import (
	"bufio"
	"final/generator"
	"final/router"
	"final/server"
	"fmt"
	"os"
	"strconv"
	"strings"

	"time"

	"github.com/gookit/slog"
)

func main() {
	slog.Info("Starting the program")
	if !init_main() {
		generateTime := time.Now()
		reader := bufio.NewReader(os.Stdin)
		fmt.Print("Do you want to run the generator? (y/N): ")
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input) // Trim any leading/trailing whitespace

		if strings.EqualFold(input, "y") {
			slog.Info("Running the generator")
			slog.Info("Good night! 💤")
			generator.Generator()
		} else {
			fmt.Println("Exiting program.")
			os.Exit(0)
		}
		slog.Info("Generator finished")
		slog.Info("Time taken: ", time.Since(generateTime))
		slog.Info("Good morning! ☀️")
		os.Exit(0)
	} else {
		slog.Debug("All files exist")
	}

	if os.Args[1] == "server" {
		slog.Info("Running the server")
		server.Server()
	} else if os.Args[1] == "multi" {
		slog.Info("Running the multi")
		iterations, _ := strconv.Atoi(os.Args[2])
		router.MultiRouter(iterations)
	} else {
		slog.Info("Invalid argument")
	}

}

func init_main() bool {
	slog.Configure(func(logger *slog.SugaredLogger) {
		logger.Level = slog.InfoLevel
		f := logger.Formatter.(*slog.TextFormatter)
		f.EnableColor = true
	})
	slog.Info("Initializing the program")
	basePath := "objects/"
	filesList := []string{"graphEdges.json", "graphNodes.json", "grid.json", "distancesEdges.json", "landmarks.json"}
	for _, file := range filesList {
		if _, err := os.Stat(basePath + file); os.IsNotExist(err) {
			slog.Info("File: " + file + " does not exist")
			return false
		} else {
			slog.Debug("File: " + file + " exists")
		}
	}
	return true
}
