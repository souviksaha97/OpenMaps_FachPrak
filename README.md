# Open Street Maps Fachpraktikum

## Table of Contents

- [Open Street Maps Fachpraktikum](#open-street-maps-fachpraktikum)
  - [Table of Contents](#table-of-contents)
  - [About ](#about-)
  - [Getting Started ](#getting-started-)
    - [Prerequisites](#prerequisites)
      - [Go](#go)
      - [Osmium](#osmium)
    - [Installing](#installing)
  - [Usage ](#usage-)

## About <a name = "about"></a>

Submission for OSM Fachpraktikum submission for SS2024.

## Getting Started <a name = "getting_started"></a>

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See [deployment](#deployment) for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them.

#### Go

```bash
yay -S go
```

#### Osmium

```bash
yay -S osmium-tool
osmium export planet-coastlines.osm.pbf -o coast.geojson
```

### Installing

A step by step series of examples that tell you how to get a development env running.

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo.

## Usage <a name = "usage"></a>

Add notes about how to use the system.
