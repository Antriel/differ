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
	var initialMinDistSq:Float = Math.POSITIVE_INFINITY;
	for (i => p in verts) {
		final dist = distanceToEdgeSquared(circle.position, p, verts[(i + 1) % verts.length]);
		if (dist < initialMinDistSq)
			initialMinDistSq = dist;
	}

	if (initialMinDistSq <= radiusSq) {
		final res = new SweptCollision();
		res.time = 0;
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
	for (vertex in verts) {
		var a = movement.x * movement.x + movement.y * movement.y;
		if (a == 0)
			continue;

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
