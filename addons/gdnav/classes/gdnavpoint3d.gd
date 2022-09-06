extends RefCounted
class_name GDNavPoint3D

# this class stores data about individual points
# 	connections, weight, position, etc.

# general info about point
var id: int = 0
var position: Vector3 = Vector3()
var weight: float = 1.0
var disabled: bool = false
var connections: PackedInt64Array = []
