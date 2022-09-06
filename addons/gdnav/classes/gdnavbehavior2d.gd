extends Resource
class_name GDNavBehavior2D

# This class is meant to allow users to achieve different
# navigation behaviors with the same AStar object. Without
# using a GDNavBehavior object one would need to create and
# instance many AStar objects with differing virtual function
# overrides. This unnecessarily increases memory usage and
# is more work for the developer.

# GDNavBehavior objects are best used when one would want many
# different pathfinding behaviors using the same points, or when
# you would have the same points but connected differently.

# Wrapper function to be called by other objects, because
# 	behavior._some_func() is not pretty
func compute_cost(from_id: int, to_id: int, astar: GDAStar2D) -> float:
	return _compute_cost(from_id, to_id, astar)
# Virtual function to be overridden by user to change how
# 	navigation cost is calculated
func _compute_cost(from_id: int, to_id: int, astar: GDAStar2D) -> float:
	if from_id == to_id:
		return 0.0
	var from_pos: Vector2 = astar.get_point_position(from_id)
	var to_pos: Vector2 = astar.get_point_position(to_id)
	return from_pos.distance_to(to_pos)

# Wrapper function to be called by other objects, because
# 	behavior._some_func() is not pretty
func estimate_cost(from_id: int, to_id: int, astar: GDAStar2D) -> float:
	return _compute_cost(from_id, to_id, astar)
# Virtual function to be overridden by user when estimating
# 	navigation cost to a certain point
func _estimate_cost(from_id: int, to_id: int, astar: GDAStar2D) -> float:
	if from_id == to_id:
		return 0.0
	var from_pos: Vector2 = astar.get_point_position(from_id)
	var to_pos: Vector2 = astar.get_point_position(to_id)
	return from_pos.distance_to(to_pos)

# Wrapper function to be called by other objects, because
# 	behavior._some_func() is not pretty
func should_traverse_point(id: int, astar: GDAStar2D) -> bool:
	return _should_traverse_point(id, astar)
# For the user to override to decide if a point should be traversed or not on the fly/without editing points
func _should_traverse_point(id: int, astar: GDAStar2D) -> bool:
	return true

# Wrapper function to be called by other objects, because
# 	behavior._some_func() is not pretty
func should_traverse_point_from(from_id: int, to_id: int, astar: GDAStar2D) -> bool:
	return _should_traverse_point_from(from_id, to_id, astar)
# For the user to override to decide if a point should be traversed or not from a specific point 
# 	on the fly/without editing points
func _should_traverse_point_from(from_id: int, to_id: int, astar: GDAStar2D) -> bool:
	return true
