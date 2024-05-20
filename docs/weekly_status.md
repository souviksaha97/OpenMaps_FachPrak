# Weekly Status Updates

## 22-04-2024

### Status

- looked into the library "github.com/paulmach/osm/osmpbf" to parse the pbf file
- was able to read number of nodes, relations and ways
- figured out how to convert to a geojson using osmium tool

```bash
osmium export planet-coastlines.osm.pbf -o coast.geojson
```

### To Do

- understand how to reduce the size of the geojson generated so it can be viewed on the geojson.io website
- extract coastline information from the pbf file
- checkout geojson styling guide

## Template

Copy and paste this template below every week to keep track of status throughout the OpenMaps project.

> ## <Enter Date>
> 
> ### Status
> 
> - Enter points here
> - .....
>   
> ### To Do
> 
> - Enter to-dos here
>- .....