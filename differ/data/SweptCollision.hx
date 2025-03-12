package differ.data;

import differ.math.Vector;

@:publicFields
class SweptCollision {
	var time:Float;
	var normal:Vector;
	var overlap:Float;

	@:noCompletion
	inline function new() {}

	/**
	 * Whether the movement vector is going in the direction of the shape.
	 * Useful for when `time == 0` and overlap is small, to allow ignoring movements along the edge of a shape.
	 * Basically a dot product of movement and collision normal.
	 */
	inline function pushingIn(movement:Vector):Bool {
		return movement.x * normal.x + movement.y * normal.y < 0;
	}

	inline function isSliding(movement:Vector):Bool {
		return time == 0 && overlap < 1e-6 && !pushingIn(movement);
	}
}
