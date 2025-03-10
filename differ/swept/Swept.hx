package differ.swept;

import differ.data.SweptCollision;
import differ.math.Vector;
import differ.shapes.Circle;
import differ.shapes.Polygon;

function circleToPolygon(circle:Circle, polygon:Polygon, movement:Vector):SweptCollision {
	final verts = polygon.transformedVertices;
	final radius = circle.transformedRadius;
	final radiusSq = radius * radius;
	// Check initial collision.
	for (i => p in verts) {
		final dist = distanceToEdgeSquared(circle.position, p, verts[(i + 1) % verts.length]);
		if (dist < radiusSq) {
			final res = new SweptCollision();
			res.time = 0;
			return res;
		}
	}

	var earliestT:Float = Math.POSITIVE_INFINITY;

	// Check edges.
	for (i in 0...verts.length) {
		var p1 = verts[i];
		var p2 = verts[(i + 1) % verts.length];
		var edgeVec = p2.clone().subtract(p1);

		// Calculate perpendicular normal (clockwise outward).
		var normal = new Vector(edgeVec.y, -edgeVec.x);
		var length = normal.length;
		if (length == 0)
			continue;
		normal.x /= length;
		normal.y /= length;

		var denominator = movement.x * normal.x + movement.y * normal.y;
		if (denominator == 0)
			continue;

		var c0ToP1 = circle.position.clone().subtract(p1);

		var numerator = radius - (c0ToP1.x * normal.x + c0ToP1.y * normal.y);
		var t = numerator / denominator;

		if (t < 0 || t > 1)
			continue;

		// Calculate collision point.
		var collisionX = circle.position.x + movement.x * t;
		var collisionY = circle.position.y + movement.y * t;

		// Project onto edge segment.
		var edgeLenSq = edgeVec.x * edgeVec.x + edgeVec.y * edgeVec.y;
		var proj = ((collisionX - p1.x) * edgeVec.x + (collisionY - p1.y) * edgeVec.y) / edgeLenSq;

		if (proj >= 0 && proj <= 1) {
			earliestT = Math.min(earliestT, t);
		}
	}

	// Check vertices.
	var a = movement.x * movement.x + movement.y * movement.y;
	if (a != 0) {
		for (vertex in verts) {
			final dx = circle.position.x - vertex.x;
			final dy = circle.position.y - vertex.y;
			var b = 2 * (movement.x * dx + movement.y * dy);
			var c = dx * dx + dy * dy - radiusSq;

			var discriminant = b * b - 4 * a * c;
			if (discriminant < 0)
				continue;

			var sqrtD = Math.sqrt(discriminant);
			var t1 = (-b - sqrtD) / (2 * a);
			var t2 = (-b + sqrtD) / (2 * a);

			if (t1 >= 0 && t1 <= 1 && t1 < earliestT)
				earliestT = t1;
			if (t2 >= 0 && t2 <= 1 && t2 < earliestT)
				earliestT = t2;
		}
	}

	if (earliestT > 1)
		return null;
	else {
		final collision = new SweptCollision();
		collision.time = earliestT;
		return collision;
	}
}

private function distanceToEdgeSquared(point:Vector, p1:Vector, p2:Vector):Float {
	final dx = p2.x - p1.x;
	final dy = p2.y - p1.y;
	final l2 = dx * dx + dy * dy;

	final pp1dx = point.x - p1.x;
	final pp1dy = point.y - p1.y;

	if (l2 == 0) {
		return pp1dx * pp1dx + pp1dy * pp1dy;
	}

	var t = (pp1dx * dx + pp1dy * dy) / l2;
	t = Math.max(0, Math.min(1, t));

	final projX = p1.x + t * dx;
	final projY = p1.y + t * dy;

	final pprodx = point.x - projX;
	final pprody = point.y - projY;

	return pprodx * pprodx + pprody * pprody;
}

function polygonToPolygon(polygonA:Polygon, polygonB:Polygon, movement:Vector):SweptCollision {
	var globalEntry:Float = 0;
	var globalExit:Float = 1;

	final vertsA = polygonA.transformedVertices;
	final vertsB = polygonB.transformedVertices;

	// Build a list of potential separating axes (normals to each edge) from both polygons.
	var axes:Array<Vector> = [];

	// Axes from polygonA.
	for (i in 0...vertsA.length) {
		var p1 = vertsA[i];
		var p2 = vertsA[(i + 1) % vertsA.length];
		var edge = p2.clone().subtract(p1);
		// Compute the perpendicular (clockwise) and normalize.
		var normal = new Vector(edge.y, -edge.x);
		var length = normal.length;
		if (length == 0)
			continue;
		normal.x /= length;
		normal.y /= length;
		axes.push(normal);
	}

	// Axes from polygonB.
	for (i in 0...vertsB.length) {
		var p1 = vertsB[i];
		var p2 = vertsB[(i + 1) % vertsB.length];
		var edge = p2.clone().subtract(p1);
		var normal = new Vector(edge.y, -edge.x);
		var length = normal.length;
		if (length == 0)
			continue;
		normal.x /= length;
		normal.y /= length;
		axes.push(normal);
	}

	// For each axis, compute the projection intervals for both polygons and determine the time interval of overlap.
	for (axis in axes) {
		// Project polygonA onto the axis.
		var aMin:Float = Math.POSITIVE_INFINITY;
		var aMax:Float = Math.NEGATIVE_INFINITY;
		for (vertex in vertsA) {
			var proj = vertex.x * axis.x + vertex.y * axis.y;
			if (proj < aMin)
				aMin = proj;
			if (proj > aMax)
				aMax = proj;
		}

		// Project polygonB onto the axis.
		var bMin:Float = Math.POSITIVE_INFINITY;
		var bMax:Float = Math.NEGATIVE_INFINITY;
		for (vertex in vertsB) {
			var proj = vertex.x * axis.x + vertex.y * axis.y;
			if (proj < bMin)
				bMin = proj;
			if (proj > bMax)
				bMax = proj;
		}

		// Project the movement onto the axis.
		var d = movement.x * axis.x + movement.y * axis.y;

		// If there is no movement along this axis then the intervals must overlap initially.
		if (d == 0) {
			if (aMax < bMin || aMin > bMax)
				return null; // They are separated on this axis.
			else
				continue; // They are overlapping on this axis at all times.
		}

		// Calculate the entry and exit times on this axis.
		var t0 = (bMin - aMax) / d;
		var t1 = (bMax - aMin) / d;

		// Ensure t0 is the entry time and t1 the exit time.
		if (t0 > t1) {
			var tmp = t0;
			t0 = t1;
			t1 = tmp;
		}

		// Update the global collision interval.
		if (t0 > globalEntry)
			globalEntry = t0;
		if (t1 < globalExit)
			globalExit = t1;

		// If the intervals do not overlap, no collision occurs.
		if (globalEntry > globalExit)
			return null;
	}

	// If the collision has already occurred, clamp time to 0.
	if (globalEntry < 0)
		globalEntry = 0;

	// If the earliest collision is after the movement is complete, there is no collision.
	if (globalEntry > 1)
		return null;

	var collision = new SweptCollision();
	collision.time = globalEntry;
	return collision;
}
