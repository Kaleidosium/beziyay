/**
 * Math for Making Bézier Curves Out of Lines.
 *
 * Implements a modified version of the algorithm defined in the paper:
 * S. F. Frisken, “Efficient Curve Fitting,” Journal of Graphics Tools, vol. 13, no. 2, pp. 37–54, Jan. 2008.
 * doi:10.1080/2151237x.2008.10129260
 */

// --- Interfaces and Object Literals ---

/**
 * Represents a 2D coordinate.
 * @property {number} x - The x coordinate.
 * @property {number} y - The y coordinate.
 */
export interface Coord {
  x: number;
  y: number;
}

/**
 * Represents a single cubic Bézier curve segment.
 * @property {Coord} c0 - Start point of the segment.
 * @property {Coord} c1 - First control point.
 * @property {Coord} c2 - Second control point.
 * @property {Coord} c3 - End point of the segment.
 * @property {Coord} [constrainTo] - Optional: direction constraint for smooth joins.
 * @property {number} [error] - Optional: fitting error.
 * @property {UpdateStatus} [updateStatus] - Optional: status of the last update.
 * @property {number} [steps] - Optional: number of fitting iterations.
 */
export interface CurveSegment {
  c0: Coord; // Start point of the segment
  c1: Coord; // First control point
  c2: Coord; // Second control point
  c3: Coord; // End point of the segment
  constrainTo?: Coord; // Optional: direction constraint for smooth joins
  error?: number; // Optional: fitting error
  updateStatus?: UpdateStatus; // Optional: status of the last update
  steps?: number; // Optional: number of fitting iterations
}

interface CurveData {
  segments: CurveSegment[]; // All curve segments
  vdmap: (Coord | undefined)[][]; // Vector distance map for fast distance queries
}

/**
 * Status codes indicating the result of adding a point to the curve.
 */
const UpdateStatus = {
  SUCCESS: 1, // Fitting succeeded
  FAIL_CORNER: 2, // Fitting failed due to a sharp corner
  FAIL_MAXED: 3, // Fitting failed due to too many iterations
} as const;
type UpdateStatus = typeof UpdateStatus[keyof typeof UpdateStatus];

// --- Curve Class ---

/**
 * Represents a sequence of connected cubic Bézier curve segments
 * fitted to a polyline.
 */
export class Curve {
  private static readonly MAX_ERROR = 2; // Max allowed fitting error
  private static readonly FIELD_RADIUS = 9; // Distance field radius
  private static readonly NUM_SAMPLE_POINTS = 9; // Number of samples along curve
  private static readonly MAX_ITERATIONS = 50; // Max fitting iterations
  private static readonly MAX_CORNER_ANGLE = 80; // Max angle for smooth join
  private static readonly RADIAL_SIMPLIFICATION = 1; // Min distance between points
  private static readonly SHOULD_USE_NEW_CORNER_FINDER = true; // Use improved corner detection
  private static readonly SHOULD_USE_MY_FORCE_VECTORS = true; // Use improved force vectors

  private curveData: CurveData;

  /**
   * Initialize a new curve with a starting coordinate.
   * @param {Coord} startCoord - The starting coordinate of the curve.
   */
  constructor(startCoord: Coord) {
    this.curveData = {
      segments: [this.createInitialSegment(this.roundVec(startCoord))],
      vdmap: [],
    };
  }

  /**
   * Add a new point to the curve, fitting a new segment if needed.
   * Returns the updated segment.
   * @param {Coord} coord - The coordinate to add to the curve.
   * @returns {CurveSegment} The updated curve segment.
   */
  public addToCurve(coord: Coord): CurveSegment {
    const roundedCoord = this.roundVec(coord);
    const segment = this.addToCurveImpl(roundedCoord);

    // If the update failed, try again (e.g., after starting a new segment)
    if (segment.updateStatus === UpdateStatus.SUCCESS) {
      return segment;
    }

    return this.addToCurveImpl(roundedCoord);
  }

  /**
   * Get the current segments of the curve.
   * Returns an array of CurveSegment objects.
   * @returns {CurveSegment[]} The array of curve segments.
   */
  public get segments(): CurveSegment[] {
    return [...this.curveData.segments];
  }

  /**
   * Create a new degenerate (zero-length) segment at a given point.
   */
  private createInitialSegment({ x, y }: Coord): CurveSegment {
    return {
      c0: { x, y },
      c1: { x, y },
      c2: { x, y },
      c3: { x, y },
    };
  }

  /**
   * Get the last segment in the curve.
   */
  private getLastSegment(): CurveSegment {
    return this.curveData.segments[this.curveData.segments.length - 1];
  }

  /**
   * Get the last point (c3) of a segment.
   */
  private getLastPoint(segment: CurveSegment): Coord {
    return segment.c3;
  }

  /**
   * Deep copy a segment (for rollback on fitting failure).
   */
  private copySegment(segment: CurveSegment): CurveSegment {
    return {
      c0: { ...segment.c0 },
      c1: { ...segment.c1 },
      c2: { ...segment.c2 },
      c3: { ...segment.c3 },
      constrainTo: segment.constrainTo ? { ...segment.constrainTo } : undefined,
      error: segment.error ?? 0,
      updateStatus: segment.updateStatus,
      steps: segment.steps,
    };
  }

  /**
   * Main logic for adding a point to the curve.
   * Handles segment splitting and fitting.
   */
  private addToCurveImpl(newPoint: Coord): CurveSegment {
    const lastSegment = this.getLastSegment();
    const lastPoint = this.getLastPoint(lastSegment);

    // Skip if points are too close together (radial simplification)
    if (this.getMagnitude(newPoint, lastPoint) < Curve.RADIAL_SIMPLIFICATION) {
      return lastSegment;
    }

    // If the last segment failed, start a new segment
    if (
      lastSegment.updateStatus !== UpdateStatus.SUCCESS &&
      lastSegment.updateStatus !== undefined
    ) {
      let newSegment: CurveSegment;

      if (lastSegment.updateStatus === UpdateStatus.FAIL_CORNER) {
        // Start a new, unconstrained segment for corners
        newSegment = this.createInitialSegment(lastPoint);
      } else if (lastSegment.updateStatus === UpdateStatus.FAIL_MAXED) {
        // Continue the curve with a new constrained segment
        newSegment = this.createInitialSegment(lastPoint);
        newSegment.constrainTo = this.getUnitVector(
          this.getCurveEndTangent(lastSegment),
        );
      } else {
        newSegment = this.createInitialSegment(lastPoint);
      }

      // Reset the vector distance map for new curve fitting
      this.curveData.vdmap = [];
      this.curveData.segments.push(newSegment);
    }

    // Fit the curve to the new point
    return this.updateDistanceField(newPoint);
  }

  /**
   * Fit the current segment to the new point using iterative force-based optimization.
   * Builds a distance field for fast error computation.
   */
  private updateDistanceField(currentPoint: Coord): CurveSegment {
    const segment = this.getLastSegment();
    const lastPoint = this.getLastPoint(segment);
    const originalControls = this.copySegment(segment);

    // Check for sharp corners (angle too small)
    if (this.checkCorner(segment, currentPoint)) {
      segment.updateStatus = UpdateStatus.FAIL_CORNER;
      return segment;
    }

    // Update segment endpoints
    segment.c3 = { ...currentPoint };
    segment.c2 = {
      x: segment.c2.x + currentPoint.x - lastPoint.x,
      y: segment.c2.y + currentPoint.y - lastPoint.y,
    };

    // Build a vector distance field around the line for fast distance queries
    this.buildDistanceField(lastPoint, currentPoint);

    // Render a square cap at the endpoint for completeness
    this.renderEndCap(currentPoint);

    // Iteratively fit the curve by adjusting control points
    let steps = 0;
    while (true) {
      let f1 = { x: 0, y: 0 };
      let f2 = { x: 0, y: 0 };

      // --- Create force vectors by sampling the curve and measuring error ---
      for (let i = 0; i < Curve.NUM_SAMPLE_POINTS; i++) {
        const t = i / Curve.NUM_SAMPLE_POINTS;
        const point = this.roundVec(this.getPointAlongCurve(segment, t));
        const dist = this.getDistanceFromPolyline(point);
        const d = this.magnitudeVec(dist);
        const { x: dx, y: dy } = dist;

        // Heavily weight the ends to avoid "hooking"
        let modifier = 1;
        if (Curve.SHOULD_USE_MY_FORCE_VECTORS && (t < 0.1 || t > 0.9)) {
          modifier = 10;
        }

        // Accumulate force vectors for control points
        f1.x += t * Math.pow(1 - t, 2) * d * dx * modifier;
        f1.y += t * Math.pow(1 - t, 2) * d * dy * modifier;
        f2.x += Math.pow(t, 2) * (1 - t) * d * dx * modifier;
        f2.y += Math.pow(t, 2) * (1 - t) * d * dy * modifier;
      }

      // --- Mitigate "hooking" by nudging force vectors toward the midpoint ---
      if (Curve.SHOULD_USE_MY_FORCE_VECTORS) {
        const midpoint = this.getSegmentMidpoint(segment);
        const fromEnd = this.subVec(midpoint, segment.c2);
        const fromBeginning = this.subVec(midpoint, segment.c1);
        f1 = this.subVec(f1, this.multVec(fromBeginning, 0.03));
        f2 = this.subVec(f2, this.multVec(fromEnd, 0.03));
      }

      // --- Constrain first control point for smooth joins ---
      if (segment.constrainTo) {
        f1 = this.multVec(
          segment.constrainTo,
          this.dotProduct(segment.constrainTo, f1),
        );
      }

      // --- Apply force vectors to control points ---
      const alpha = 1;
      const scaleFactor = (alpha * 6) / Curve.NUM_SAMPLE_POINTS;
      segment.c1.x -= f1.x * scaleFactor;
      segment.c1.y -= f1.y * scaleFactor;
      segment.c2.x -= f2.x * scaleFactor;
      segment.c2.y -= f2.y * scaleFactor;

      // --- Compute fitting error ---
      let error = 0;
      for (let i = 0; i < Curve.NUM_SAMPLE_POINTS; i++) {
        const t = i / Curve.NUM_SAMPLE_POINTS;
        const point = this.getPointAlongCurve(segment, t);
        const dist = this.getDistanceFromPolyline(this.roundVec(point));
        error += dist.x ** 2 + dist.y ** 2;
      }
      error /= Curve.NUM_SAMPLE_POINTS;
      steps++;

      segment.error = error;
      segment.steps = steps;

      // --- Stop if error is low enough or we've iterated too much ---
      if (error < Curve.MAX_ERROR || steps > Curve.MAX_ITERATIONS) {
        break;
      }
    }

    // --- If fitting failed, roll back to original controls ---
    if (steps > Curve.MAX_ITERATIONS) {
      Object.assign(segment, originalControls);
      segment.updateStatus = UpdateStatus.FAIL_MAXED;
      return segment;
    }

    segment.updateStatus = UpdateStatus.SUCCESS;
    return segment;
  }

  /**
   * Build a vector distance field (vdmap) around the line segment.
   * This allows fast distance queries for curve fitting.
   */
  private buildDistanceField(lastPoint: Coord, currentPoint: Coord): void {
    // Get a unit vector perpendicular to the line
    const perpVec = this.getPerpendicularUnitVector(lastPoint, currentPoint);
    // Scale it to the field radius
    const dist = this.getScaledVectorDifference(perpVec);
    // Compute the two corners of the rectangle around the line
    const a1 = this.addVec(lastPoint, dist);
    const b1 = this.addVec(currentPoint, dist);

    // Compute the vertical and horizontal steps for DDA rasterization
    const vertDiff = this.negateVec(this.multVec(dist, 2));
    const vertSteps = this.getDDASteps(vertDiff);
    const vertIncrement = this.divVec(vertDiff, vertSteps);

    const horizDiff = this.subVec(b1, a1);
    const horizSteps = this.getDDASteps(horizDiff);
    const horizIncrement = this.divVec(horizDiff, horizSteps);

    // Rasterize the rectangle using DDA (Digital Differential Analyzer)
    let currentHorizLocation = a1;
    for (let i = 0; i < horizSteps; i++) {
      let currentLocation = currentHorizLocation;
      let currentVal = dist;

      for (let j = 0; j < vertSteps; j++) {
        const { x, y } = this.roundVec(currentLocation);
        this.updateVDMap(x, y, this.roundVec(currentVal));

        currentVal = this.addVec(currentVal, vertIncrement);
        currentLocation = this.addVec(currentLocation, vertIncrement);
      }

      currentHorizLocation = this.addVec(currentHorizLocation, horizIncrement);
    }
  }

  /**
   * Render a square "cap" at the endpoint to complete the distance field.
   */
  private renderEndCap(currentPoint: Coord): void {
    const upperLeft = this.subVec(currentPoint, {
      x: Curve.FIELD_RADIUS,
      y: Curve.FIELD_RADIUS,
    });
    const bottomRight = this.addVec(currentPoint, {
      x: Curve.FIELD_RADIUS,
      y: Curve.FIELD_RADIUS,
    });

    for (let x = upperLeft.x; x < bottomRight.x; x++) {
      for (let y = upperLeft.y; y < bottomRight.y; y++) {
        const val = this.subVec({ x, y }, currentPoint);
        this.updateVDMap(x, y, val);
      }
    }
  }

  /**
   * Update the vector distance map at (x, y) with a new value if it's closer.
   */
  private updateVDMap(x: number, y: number, val: Coord): void {
    if (!this.curveData.vdmap[x]) {
      this.curveData.vdmap[x] = [];
    }

    const existing = this.curveData.vdmap[x][y];
    if (existing === undefined) {
      this.curveData.vdmap[x][y] = val;
    } else {
      this.curveData.vdmap[x][y] =
        this.sumOfSquaresVec(existing) < this.sumOfSquaresVec(val)
          ? existing
          : val;
    }
  }

  /**
   * Get the vector distance from a point to the polyline using the distance field.
   */
  private getDistanceFromPolyline({ x, y }: Coord): Coord {
    const column = this.curveData.vdmap[x];
    if (!column) {
      return { x: Curve.FIELD_RADIUS, y: Curve.FIELD_RADIUS };
    }

    const first = column[y];
    const second = column[y + 1];

    let val = first
      ? { ...first }
      : { x: Curve.FIELD_RADIUS, y: Curve.FIELD_RADIUS };

    if (second?.x !== undefined && Math.abs(val.x) > Math.abs(second.x)) {
      val.x = second.x;
    }
    if (second?.y !== undefined && Math.abs(val.y) > Math.abs(second.y)) {
      val.y = second.y;
    }

    return val;
  }

  /**
   * Get the tangent vector at the end of a curve segment.
   */
  private getCurveEndTangent(curve: CurveSegment): Coord {
    const controlPoint = curve.c3;
    const endpoint = curve.c2;
    return {
      x: 3 * (controlPoint.x - endpoint.x),
      y: 3 * (controlPoint.y - endpoint.y),
    };
  }

  /**
   * Check if the angle at the join is too sharp (corner).
   */
  private checkCorner(curveSegment: CurveSegment, point: Coord): boolean {
    if (this.equalVec(curveSegment.c0, curveSegment.c3)) {
      return false;
    }

    let tan: Coord;
    if (Curve.SHOULD_USE_NEW_CORNER_FINDER) {
      // Use the tangent at t=0.95 for better accuracy
      tan = this.getUnitVector(
        this.subVec(
          this.getPointAlongCurve(curveSegment, 0.95),
          curveSegment.c3,
        ),
      );
    } else {
      tan = this.getUnitVector(this.getCurveEndTangent(curveSegment));
    }

    // New segment direction
    const newSegment = this.getUnitVector({
      x: point.x - curveSegment.c3.x,
      y: point.y - curveSegment.c3.y,
    });

    // Compute angle between tangents
    const dot = this.dotProduct(tan, newSegment);
    const angle = this.radiansToDegrees(Math.acos(dot));

    // If angle is too sharp, treat as a corner
    return angle < Curve.MAX_CORNER_ANGLE;
  }

  // --- Vector utility methods ---

  /**
   * Get the midpoint of a segment.
   */
  private getSegmentMidpoint(segment: CurveSegment): Coord {
    return this.addVec(
      this.divVec(this.subVec(segment.c3, segment.c0), 2),
      segment.c0,
    );
  }

  /**
   * Scale a unit vector by the field radius.
   */
  private getScaledVectorDifference(uvec: Coord): Coord {
    return {
      x: uvec.x * Curve.FIELD_RADIUS,
      y: uvec.y * Curve.FIELD_RADIUS,
    };
  }

  /**
   * Negate a vector.
   */
  private negateVec(vec: Coord): Coord {
    return { x: -vec.x, y: -vec.y };
  }

  /**
   * Round a vector's components.
   */
  private roundVec(vec: Coord): Coord {
    return { x: Math.round(vec.x), y: Math.round(vec.y) };
  }

  /**
   * Sum of squares of a vector's components.
   */
  private sumOfSquaresVec(vec: Coord): number {
    return vec.x ** 2 + vec.y ** 2;
  }

  /**
   * Magnitude (length) of a vector.
   */
  private magnitudeVec(vec: Coord): number {
    return Math.sqrt(this.sumOfSquaresVec(vec));
  }

  /**
   * Add two vectors.
   */
  private addVec(a: Coord, b: Coord): Coord {
    return { x: a.x + b.x, y: a.y + b.y };
  }

  /**
   * Subtract vector b from a.
   */
  private subVec(a: Coord, b: Coord): Coord {
    return this.addVec(a, this.negateVec(b));
  }

  /**
   * Multiply a vector by a scalar.
   */
  private multVec(a: Coord, scalar: number): Coord {
    return { x: a.x * scalar, y: a.y * scalar };
  }

  /**
   * Divide a vector by a scalar (safe: avoids division by zero).
   */
  private divVec(a: Coord, scalar: number): Coord {
    if (scalar === 0) {
      // Return a zero vector if dividing by zero
      return { x: 0, y: 0 };
    }
    return this.multVec(a, 1 / scalar);
  }

  /**
   * Check if two vectors are equal.
   */
  private equalVec(a: Coord, b: Coord): boolean {
    return a.x === b.x && a.y === b.y;
  }

  /**
   * Dot product of two vectors.
   */
  private dotProduct(a: Coord, b: Coord): number {
    return a.x * b.x + a.y * b.y;
  }

  /**
   * Get the distance between two points.
   */
  private getMagnitude(vec: Coord, p2: Coord = { x: 0, y: 0 }): number {
    return Math.sqrt((vec.x - p2.x) ** 2 + (vec.y - p2.y) ** 2);
  }

  /**
   * Convert radians to degrees.
   */
  private radiansToDegrees(rad: number): number {
    return (rad * 180) / Math.PI;
  }

  /**
   * Get the number of DDA steps for rasterizing a vector.
   */
  private getDDASteps(diff: Coord): number {
    return Math.max(Math.abs(diff.x), Math.abs(diff.y));
  }

  /**
   * Get a unit vector perpendicular to the line from a to b.
   */
  private getPerpendicularUnitVector(a: Coord, b: Coord): Coord {
    const deltaX = b.x - a.x;
    const deltaY = b.y - a.y;
    return this.getUnitVector({ x: deltaY, y: -deltaX });
  }

  /**
   * Normalize a vector to unit length.
   */
  private getUnitVector({ x, y }: Coord): Coord {
    const magnitude = this.getMagnitude({ x, y });
    return { x: x / magnitude, y: y / magnitude };
  }

  /**
   * Get a point along a cubic Bezier curve at parameter t.
   */
  private getPointAlongCurve(curve: CurveSegment, t: number): Coord {
    const x = this.singleComponentBezier(
      t,
      curve.c0.x,
      curve.c1.x,
      curve.c2.x,
      curve.c3.x,
    );
    const y = this.singleComponentBezier(
      t,
      curve.c0.y,
      curve.c1.y,
      curve.c2.y,
      curve.c3.y,
    );
    return { x, y };
  }

  /**
   * Evaluate a single component of a cubic Bezier curve at t.
   */
  private singleComponentBezier(
    t: number,
    w0: number,
    w1: number,
    w2: number,
    w3: number,
  ): number {
    return (
      w0 * (1 - t) ** 3 +
      w1 * 3 * t * (1 - t) ** 2 +
      w2 * 3 * t ** 2 * (1 - t) +
      w3 * t ** 3
    );
  }
}
