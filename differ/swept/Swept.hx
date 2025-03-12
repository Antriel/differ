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
	final vertsA = polygonA.transformedVertices;
	final vertsB = polygonB.transformedVertices;

	// Build candidate axes from both polygons (using the outward normals of each edge).
	var axes:Array<Vector> = [];
	for (i in 0...vertsA.length) {
		var p1 = vertsA[i];
		var p2 = vertsA[(i + 1) % vertsA.length];
		var edge = p2.clone().subtract(p1);
		if (edge.length == 0)
			continue;
		var normal = new Vector(edge.y, -edge.x);
		var len = normal.length;
		if (len == 0)
			continue;
		normal.x /= len;
		normal.y /= len;
		axes.push(normal);
	}
	for (i in 0...vertsB.length) {
		var p1 = vertsB[i];
		var p2 = vertsB[(i + 1) % vertsB.length];
		var edge = p2.clone().subtract(p1);
		if (edge.length == 0)
			continue;
		var normal = new Vector(edge.y, -edge.x);
		var len = normal.length;
		if (len == 0)
			continue;
		normal.x /= len;
		normal.y /= len;
		axes.push(normal);
	}

	// Swept collision phase.
	// Initialize globalEntry to a very small value so that if any t0 > this value, we update.
	var globalEntry:Float = -1e-6;
	var globalExit:Float = 1.0;
	var sweptNormal:Vector = null;

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

		// If there is no movement along this axis, the intervals must be overlapping.
		if (d == 0) {
			if (aMax < bMin || aMin > bMax)
				return null; // They are separated on this axis.
			else
				continue;
		}

		// Compute entry and exit times along this axis.
		var t0 = (bMin - aMax) / d;
		var t1 = (bMax - aMin) / d;

		// Ensure t0 is the entry time and t1 the exit time.
		if (t0 > t1) {
			var tmp = t0;
			t0 = t1;
			t1 = tmp;
		}

		if (t0 > globalEntry) {
			globalEntry = t0;
			// Choose the collision normal based on movement direction along the axis.
			// (Ensure it points from polygonB toward polygonA.)
			sweptNormal = (d < 0) ? axis : new Vector(-axis.x, -axis.y);
		}
		if (t1 < globalExit)
			globalExit = t1;

		// If the intervals do not overlap, no collision occurs.
		if (globalEntry > globalExit)
			return null;
	}

	// If the collision occurs after the movement completes, there's no collision.
	if (globalEntry > 1)
		return null;

	// If the collision happens in the future, use the swept collision result.
	if (globalEntry > 0) {
		var collision = new SweptCollision();
		collision.time = globalEntry;
		collision.normal = sweptNormal;
		collision.overlap = 0;
		return collision;
	}

	// Else, globalEntry is 0 (or negative, meaning initial overlap or just touching).
	// Now perform a static SAT test to compute the Minimum Translation Vector (MTV).
	var minOverlap:Float = Math.POSITIVE_INFINITY;
	var mtvAxis:Vector = null;
	for (axis in axes) {
		// Project polygonA on axis.
		var aMin:Float = Math.POSITIVE_INFINITY;
		var aMax:Float = Math.NEGATIVE_INFINITY;
		for (vertex in vertsA) {
			var proj = vertex.x * axis.x + vertex.y * axis.y;
			if (proj < aMin)
				aMin = proj;
			if (proj > aMax)
				aMax = proj;
		}
		// Project polygonB on axis.
		var bMin:Float = Math.POSITIVE_INFINITY;
		var bMax:Float = Math.NEGATIVE_INFINITY;
		for (vertex in vertsB) {
			var proj = vertex.x * axis.x + vertex.y * axis.y;
			if (proj < bMin)
				bMin = proj;
			if (proj > bMax)
				bMax = proj;
		}
		// Compute overlap along this axis.
		var overlap = Math.min(aMax, bMax) - Math.max(aMin, bMin);
		if (overlap < minOverlap) {
			minOverlap = overlap;
			mtvAxis = axis;
		}
	}

	if (mtvAxis == null)
		return null;

	var centerA = new Vector(0, 0); // TODO this could be lazy-cached on the polygon.
	for (vertex in vertsA) {
		centerA.x += vertex.x;
		centerA.y += vertex.y;
	}
	centerA.x /= vertsA.length;
	centerA.y /= vertsA.length;

	var centerB = new Vector(0, 0);
	for (vertex in vertsB) {
		centerB.x += vertex.x;
		centerB.y += vertex.y;
	}
	centerB.x /= vertsB.length;
	centerB.y /= vertsB.length;

	// Vector from centerB to centerA.
	var fromBToA = new Vector(centerA.x - centerB.x, centerA.y - centerB.y);

	// Ensure mtvAxis is pointing from B to A.
	var dotProduct = mtvAxis.x * fromBToA.x + mtvAxis.y * fromBToA.y;
	if (dotProduct < 0) {
		// Flip the direction if it's pointing the wrong way.
		mtvAxis.x = -mtvAxis.x;
		mtvAxis.y = -mtvAxis.y;
	}

	var collision = new SweptCollision();
	collision.time = 0;
	collision.normal = mtvAxis;
	collision.overlap = minOverlap;
	return collision;
}
