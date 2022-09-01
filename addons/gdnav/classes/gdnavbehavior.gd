extends Resource
class_name GDNavBehavior

# This class is meant to allow users to achieve different
# navigation behaviors with the same AStar object. Without
# using a GDNavBehavior object one would need to create and
# instance many AStar objects with differing virtual function
# overrides. This unnecessarily increases memory usage and
# is more work for the developer.

# GDNavBehavior objects are best used when one would want many
# different pathfinding behaviors using the same points, or when
# you would have the same points but connected differently.
