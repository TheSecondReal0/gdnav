extends RefCounted
class_name GDAStar2D

# This is an astar implementation meant to be as easy
# 	to use as Godot's built-in solution while being much more
# 	customizeable
# This class matches the built-in AStar API so one should be
# 	able to simply drop this class into any existing project
# 	without any problems

# GDAstar2D and GDAStar3D are equivalent except GDAStar3D uses Vector3s
# 	while GDAStar2D uses Vector2s

var points: Dictionary = {}
var point_id_to_pos: Dictionary = {}
var point_pos_to_id: Dictionary = {}

var last_free_id: int = 0

# Finds the best path using the AStar algorithm
# The behavior input is completely optional and is used when
# 	you don't want to create multiple AStar objects to get
# 	unique pathfinding behavior
# Ignoring the behavior input will make this implementation
# 	behave just like the built-in AStar classes
func astar(from: int, to: int, behavior: GDNavBehavior2D = null) -> PackedInt64Array:
	# Using a dictionary as a hash set, null is placeholder value
	var open_set: Dictionary = {from: null}
	# Stores the preceding node on the cheapest path to each node
	# For node n, came_from[n] is the preceding node on the cheapest path to node n
	var came_from: Dictionary = {}
	# Stores the cost of the lowest path to each node
	var g_scores: Dictionary = {}
	g_scores[from] = 0
	# Stores a guess at how expensive the path from a given point to finish will be
	var f_scores: Dictionary = {}
	f_scores[from] = estimate_cost(from, to, behavior)
	
	var old_current: int = -1
	var current: int = from
	while not open_set.is_empty():
		old_current = current
		current = get_point_id_with_lowest_f_score(open_set, f_scores)
		if current == to:
			# Use current values in came_from to figure out the best path
			var path: PackedInt64Array = [current]
			while current != from:
				current = came_from[current]
				path.append(current)
			# Reverse path because right now it starts at the target and ends at the starting point
			path.reverse()
			return path
		
		if current == old_current and old_current != from:
			assert(false, "We just checked the same point twice in a row, should not happen")
		
		# remove current point from the open set so we don't traverse it again immediately
		open_set.erase(current)
		var current_point: GDNavPoint = points[current]
		var connections: PackedInt64Array = current_point.connections
		var connected_point: GDNavPoint
		var tentative_g_score: float
		var connected_g_score: float
		
		for connected in connections:
			connected_point = points[connected]
			# If for whatever reason we don't want the algorithm to consider this point
			if connected_point.disabled or not should_traverse_point(connected, behavior) or not should_traverse_point_from(current, connected, behavior):
				continue
			tentative_g_score = g_scores[current] + (compute_cost(current, connected, behavior) * connected_point.weight)
			connected_g_score = g_scores.get(connected, INF)
			# If we just found the new cheapest path to a point, record it
			if tentative_g_score < connected_g_score:
				came_from[connected] = current
				g_scores[connected] = tentative_g_score
				f_scores[connected] = tentative_g_score + estimate_cost(connected, to, behavior)
				
				if not connected in open_set:
					open_set[connected] = null
	
	# If this point is reached, the algorithm failed to find a path
	# Return empty list to signify failure
	return PackedInt64Array()

# Function to abstract deciding whether or not to use the GDNavBehavior
# 	instead of the virtual functions in this class
func compute_cost(from_id: int, to_id: int, behavior: GDNavBehavior2D = null) -> float:
	if behavior != null:
		return behavior.compute_cost(from_id, to_id, self)
	return _compute_cost(from_id, to_id)

# Function to abstract deciding whether or not to use the GDNavBehavior
# 	instead of the virtual functions in this class
func estimate_cost(from_id: int, to_id: int, behavior: GDNavBehavior2D = null) -> float:
	if behavior != null:
		return behavior.estimate_cost(from_id, to_id, self)
	return _estimate_cost(from_id, to_id)

# Function to abstract deciding whether or not to use the GDNavBehavior
# 	instead of the virtual functions in this class
func should_traverse_point(id: int, behavior: GDNavBehavior2D = null) -> bool:
	if is_point_disabled(id):
		return false
	if behavior != null:
		return behavior.should_traverse_point(id, self)
	return _should_traverse_point(id)

# For the user to override to decide if a point should be traversed or not on the fly/without editing points
func _should_traverse_point(id: int) -> bool:
	return true

func should_traverse_point_from(from_id: int, to_id: int, behavior: GDNavBehavior2D = null) -> bool:
	if is_point_disabled(from_id) or is_point_disabled(to_id):
		return false
	if behavior != null:
		return behavior.should_traverse_point_from(from_id, to_id, self)
	return _should_traverse_point_from(from_id, to_id)

# For the user to override to decide if a point should be traversed or not on the fly/without editing points
func _should_traverse_point_from(from_id: int, to_id: int) -> bool:
	return true

# Returns the point id of the point at this position
# If no point exists at this position, return -1
func get_point_id(pos: Vector2) -> int:
	return point_pos_to_id.get(pos, -1)

# Check whether or not a point exists at this coordinate
func does_point_exist_at(pos: Vector2) -> bool:
	return pos in point_pos_to_id

func get_point_id_with_lowest_f_score(open_set: Dictionary, f_scores: Dictionary) -> int:
	var min_id: int = open_set.keys()[0]
	var min_f: float = f_scores[min_id]
	
	var curr_f: float
	for id in open_set:
		curr_f = f_scores[id]
		if curr_f < min_f:
			min_id = id
			min_f = curr_f
	
	return  min_id

# API functions to match built-in AStar implementation
# ----------
# Virtual function to be overridden by user to change how
# 	navigation cost is calculated
func _compute_cost(from_id: int, to_id: int) -> float:
	if from_id == to_id:
		return 0.0
	var from_pos: Vector2 = point_id_to_pos.get(from_id, null)
	var to_pos: Vector2 = point_id_to_pos.get(to_id, null)
	return from_pos.distance_to(to_pos)

# Virtual function to be overridden by user when estimating
# 	navigation cost to a certain point
func _estimate_cost(from_id: int, to_id: int) -> float:
	if from_id == to_id:
		return 0.0
	var from_pos: Vector2 = point_id_to_pos.get(from_id, null)
	var to_pos: Vector2 = point_id_to_pos.get(to_id, null)
	return from_pos.distance_to(to_pos)

func add_point(id: int, position: Vector2, weight_scale: float = 1.0) -> void:
	var point: GDNavPoint = GDNavPoint.new()
	point.id = id
	point.position = position
	point.weight = weight_scale
	points[id] = point
	point_id_to_pos[id] = position
	point_pos_to_id[position] = id

func are_points_connected(from_id: int, to_id: int, bidirectional: bool = true) -> bool:
	var from_connections: PackedInt64Array = points[from_id].connections
	var to_connections: PackedInt64Array = points[to_id].connections
	if not to_id in from_connections:
		return false
	return not bidirectional or from_id in to_connections

# Erases all points and segments from graph
func clear() -> void:
	points.clear()
	point_id_to_pos.clear()
	point_pos_to_id.clear()

# Connects two points together
func connect_points(from_id: int, to_id: int, bidirectional: bool = true) -> void:
	if from_id == to_id:
		return
	var from_point: GDNavPoint = points[from_id]
	var to_point: GDNavPoint = points[to_id]
	if not to_id in from_point.connections:
		from_point.connections.append(to_id)
	if bidirectional and not from_id in to_point.connections:
		to_point.connections.append(from_id)

# Disconnects two points
func disconnect_points(from_id: int, to_id: int, bidirectional: bool = true) -> void:
	var from_point: GDNavPoint = points[from_id]
	var to_point: GDNavPoint = points[to_id]
	var index: int = from_point.connections.find(to_id)
	if index != -1:
		from_point.connections.remove_at(index)
	if not bidirectional:
		return
	index = to_point.connections.find(from_id)
	if index != -1:
		to_point.connections.remove_at(index)

# Get an available point id
func get_available_point_id() -> int:
	last_free_id += 1
	return last_free_id

# Returns the closest point ID to the given position
func get_closest_point(to_position: Vector2, include_disabled: bool = false, behavior: GDNavBehavior2D = null) -> int:
	if to_position in point_pos_to_id:
		var id: int = point_pos_to_id[to_position]
		var point: GDNavPoint = points[id]
		if (not point.disabled or include_disabled) and should_traverse_point(id, behavior):
			return point_pos_to_id[to_position]
	
	var closest_id: int = -1
	var closest_dist: float = INF
	var curr_point: GDNavPoint
	var curr_dist: float
	for id in points:
		curr_point = points[id]
		if not ((not curr_point.disabled or include_disabled) and should_traverse_point(id, behavior)):
			continue
		curr_dist = curr_point.position.distance_squared_to(to_position)
		if curr_dist < closest_dist:
			closest_id = id
			closest_dist = curr_dist
	return closest_id

# Returns the closest position that is part of a segment in the graph
func get_closest_position_in_segment(to_position: Vector2) -> Vector2:
	return Vector2()

# Returns an array of point IDs that make up the path between two points
func get_id_path(from_id: int, to_id: int, behavior: GDNavBehavior2D = null) -> PackedInt64Array:
	return astar(from_id, to_id, behavior)

# Returns a list of the point IDs connected to the given point ID
func get_point_connections(id: int) -> PackedInt64Array:
	return points[id].connections

# Returns the number of points on the graph
func get_point_count() -> int:
	return points.size()

# returns a list of all point IDs on the graph
func get_point_ids() -> Array:
	return points.keys()

# Returns the path between two points in terms of vector coordinates instead of IDs
func get_point_path(from_id: int, to_id: int, behavior: GDNavBehavior2D = null) -> PackedVector2Array:
	var path: PackedInt64Array = astar(from_id, to_id, behavior)
	var point_path: PackedVector2Array = PackedVector2Array()
	for id in path:
		point_path.append(point_id_to_pos[id])
	return point_path

# Returns the position of a point with a given point ID
func get_point_position(id: int) -> Vector2:
	return points[id].position

# Returns the weight scale of the given point
func get_point_weight_scale(id: int) -> float:
	return points[id].weight

# Returns whether or not there is already a point with this ID in the graph
func has_point(id: int) -> bool:
	return id in points

# Returns whether or not the point with the given ID is disabled
func is_point_disabled(id: int) -> bool:
	return points[id].disabled

# Removes the point with the given ID from the graph
func remove_point(id: int) -> void:
	point_pos_to_id.erase(point_id_to_pos[id])
	point_id_to_pos.erase(id)
	points.erase(id)

# Disables/enables the point with the given point ID for bathfinding
func set_point_disabled(id: int, disabled: bool = true) -> void:
	points[id].disabled = disabled

# Updates the position of the point with the given point ID
func set_point_position(id: int, position: Vector2) -> void:
	points[id].position = position
	point_id_to_pos[id] = position

# Updates the weight scale of the point with the given point ID
func set_point_weight_scale(id: int, weight_scale: float) -> void:
	points[id].weight = weight_scale
