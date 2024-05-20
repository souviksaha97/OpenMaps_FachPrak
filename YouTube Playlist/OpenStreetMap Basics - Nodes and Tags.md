Map
 |- Nodes - points on a map which can represent anything (tree, cafe, etc) (set of coordinates, latitude and longitude)
 |- Tags - keywords (no spaces, tags are lowercase)
 |	 |- Key, Value
 |	 |- natural = tree
 |	 |- amenity = cafe
 |	 |- highway = street_lamp
 |- Ways - lines (list of nodes, ordered). Used to represent  rivers, roads. Also has a tag. Traffic/river flows in the ascending order direction of the ordered nodes. For ways to be routable,       |              they need to share a common node. Highway tags are bi-directional, unless tag oneway = yes
 |    |- waterway = river
 |    |- highway = footway (walking path/hiking trail)
 |    |- highway = road
 |    |- highway = primary (main road), name = 'Main Street'
 |- Polygon - closed way (each way has their own identification number) (500->501->502->504->500)
 |- Layers - way of representing elements one on top of the other. Layers can be -ve too.
