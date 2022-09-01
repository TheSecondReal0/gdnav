extends RefCounted
class_name GDNavPoint

# this class stores data about individual points
# 	connections, weight, position, etc.

# general info about point
var id: int = 0
var position: Vector2 = Vector2()
var weight: float = 1.0
var disabled: bool = false
var connections: PackedInt64Array = []
