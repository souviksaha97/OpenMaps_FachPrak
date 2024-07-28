package main

import (
	"final/generator"
	"final/router"
	"final/server"
	"os"

	"time"

	"github.com/gookit/slog"
)

func main() {
	slog.Info("Starting the program")
	if !init_main() {
		generateTime := time.Now()
		slog.Info("Running the generator")
		slog.Info("Good night! 💤")
		generator.Generator()
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
		router.Router()
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
			slog.Info("Running the generator")
			return false
		} else {
			slog.Debug("File: " + file + " exists")
		}
	}
	return true
}
