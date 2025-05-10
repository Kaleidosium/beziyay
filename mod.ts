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
 */
export interface Coord {
  x: number;
  y: number;
}

/**
 * Represents a single cubic Bézier curve segment.
 */
export interface CurveSegment {
  /** Start point of the segment. */
  c0: Coord;
  /** First control point. */
  c1: Coord;
  /** Second control point. */
  c2: Coord;
  /** End point of the segment. */
  c3: Coord;
  /** Optional constraint vector for the c1 control point, ensuring smoothness between segments. */
  constrain_to?: Coord;
  /** Calculated error metric for how well the segment fits the polyline data. */
  error?: number;
  /** Status of the last update operation for this segment. */
  update_status?: UpdateStatus;
  /** Number of iterations performed during the last fitting update (for debugging). */
  steps?: number;
}

/**
 * Status codes indicating the result of adding a point to the curve.
 */
const UpdateStatus = {
  /** The point was added successfully, and the curve segment was updated. */
  SUCCESS: 1,
  /** A sharp corner was detected; a new segment was started. */
  FAIL_CORNER: 2,
  /** The fitting algorithm reached the maximum number of iterations without converging; a new segment was started with a constraint. */
  FAIL_MAXED: 3,
} as const; // Use 'as const' for stricter typing
/** Type alias for the possible values of UpdateStatus. */
type UpdateStatus = typeof UpdateStatus[keyof typeof UpdateStatus];

// --- Curve Class ---

/**
 * Represents a sequence of connected cubic Bézier curve segments
 * fitted to a polyline.
 */
export class Curve {
  /** The array of Bézier segments that make up the curve. */
  private segments: CurveSegment[] = [];
  /** A 2D map storing precomputed shortest vectors from grid points to the original polyline. */
  private vdmap: Coord[][] = []; // Vector distance map

  // --- Constants ---
  // TODO: Add configuration settings rather than hardcoding constants
  /** Maximum allowed error for curve fitting. */
  private readonly MAX_ERROR = 2;
  /** Radius around the polyline segment to build the vector distance map. */
  private readonly FIELD_RADIUS = 9;
  /** Number of points along the curve to sample for error calculation and force vector generation. */
  private readonly NUM_SAMPLE_POINTS = 9;
  /** Maximum number of iterations for the curve fitting optimization loop. */
  private readonly MAX_ITERATIONS = 50;
  /** Maximum angle (in degrees) between the curve's end tangent and the next line segment before declaring a corner. */
  private readonly MAX_CORNER_ANGLE = 80;
  /** Minimum distance between points to process; points closer than this are ignored. */
  private readonly RADIAL_SIMPLIFICATION = 1;
  /** Flag to use a modified corner detection method (tangent near end vs. exact end tangent). */
  private readonly SHOULD_USE_NEW_CORNER_FINDER = true;
  /** Flag to use custom force vector modifications (weighting ends, pushing towards midpoint). */
  private readonly SHOULD_USE_MY_FORCE_VECTORS = true;

  /**
   * Creates a new Curve instance, starting with a single point.
   * @param {Coord} startCoord - The initial coordinate to start the curve.
   */
  constructor(startCoord: Coord) {
    this.segments.push(this.initCurveSegment(this.roundVec(startCoord)));
  }

  /**
   * Adds a new point to the polyline and updates the curve fitting.
   * This may modify the last segment or add a new segment if a corner
   * is detected or the fitting fails to converge.
   * @param {Coord} coord - The new coordinate to add to the polyline.
   * @returns {CurveSegment | undefined} The last segment of the curve after the update, or undefined if initialization failed.
   */
  public addToCurve(coord: Coord): CurveSegment {
    const roundedCoord = this.roundVec(coord);
    const segment = this.addToCurveImpl(roundedCoord);

    if (segment.update_status === UpdateStatus.SUCCESS) {
      return segment;
    }

    return this.addToCurveImpl(roundedCoord);
  }

  /**
   * Get all segments of the curve
   */
  public getSegments(): CurveSegment[] {
    return [...this.segments];
  }

  private initCurveSegment(coord: Coord): CurveSegment {
    const { x, y } = coord;
    return {
      c0: { x, y },
      c1: { x, y },
      c2: { x, y },
      c3: { x, y },
    };
  }

  private getLastSegment(): CurveSegment {
    return this.segments[this.segments.length - 1];
  }

  private getLastPoint(segment: CurveSegment): Coord {
    return segment.c3;
  }

  private copySegment(segment: CurveSegment): CurveSegment {
    return {
      c0: { x: segment.c0.x, y: segment.c0.y },
      c1: { x: segment.c1.x, y: segment.c1.y },
      c2: { x: segment.c2.x, y: segment.c2.y },
      c3: { x: segment.c3.x, y: segment.c3.y },
      constrain_to: segment.constrain_to
        ? { x: segment.constrain_to.x, y: segment.constrain_to.y }
        : undefined,
      error: segment.error || 0,
      update_status: segment.update_status,
    };
  }

  private addToCurveImpl(newPoint: Coord): CurveSegment {
    const lastSegment = this.getLastSegment();
    const lastPoint = this.getLastPoint(lastSegment);

    // We do too much work with too little benefit if points are too close together.
    if (this.getMagnitude(newPoint, lastPoint) < this.RADIAL_SIMPLIFICATION) {
      return lastSegment;
    }

    if (
      lastSegment.update_status !== UpdateStatus.SUCCESS &&
      lastSegment.update_status !== undefined
    ) {
      let newSegment: CurveSegment;

      // Start a new, unconstrained segment for corners.
      if (lastSegment.update_status === UpdateStatus.FAIL_CORNER) {
        newSegment = this.initCurveSegment({ x: lastPoint.x, y: lastPoint.y });
      } // We had to give up, so continue the curve, but with a new segment.
      else if (lastSegment.update_status === UpdateStatus.FAIL_MAXED) {
        newSegment = this.initCurveSegment({ x: lastPoint.x, y: lastPoint.y });
        newSegment.constrain_to = this.getUnitVector(
          this.getCurveEndTangent(lastSegment),
        );
      } else {
        // Default case, should not happen but TypeScript requires it
        newSegment = this.initCurveSegment({ x: lastPoint.x, y: lastPoint.y });
      }

      // Reset the vector distance map because we're fitting a new curve!
      this.vdmap = [];
      this.segments.push(newSegment);
    }

    return this.updateDistanceField(newPoint);
  }

  private updateDistanceField(currentPoint: Coord): CurveSegment {
    const segment = this.getLastSegment();
    const { x, y } = currentPoint;

    const lastPoint = this.getLastPoint(segment);
    const ogControls = this.copySegment(segment);

    if (this.checkCorner(segment, currentPoint)) {
      segment.update_status = UpdateStatus.FAIL_CORNER;
      return segment;
    }

    segment.c3 = { x, y };
    segment.c2 = {
      x: segment.c2.x + x - lastPoint.x,
      y: segment.c2.y + y - lastPoint.y,
    };

    //
    // Prep the rectangle around our line in which we'll store distance-to-the-line
    // for each pixel in the box.
    const perpVec = this.getPerpindicularUnitVector(lastPoint, currentPoint);
    const dist = this.getScaledVectorDifference(perpVec);
    const a1 = this.addVec(lastPoint, dist);
    const b1 = this.addVec(currentPoint, dist);

    const vertDiff = this.negateVec(this.multVec(dist, 2));
    const vertSteps = this.getDDASteps(vertDiff);
    const vertIncrement = this.divVec(vertDiff, vertSteps);

    const horizDiff = this.subVec(b1, a1);
    const horizSteps = this.getDDASteps(horizDiff);
    const horizIncrement = this.divVec(horizDiff, horizSteps);

    //
    // "Render" the rectangle around the line by interpolating `current_val`
    // up the box (perpindicularly to the line) for each step across the box
    // (parallel to the line). This is a very basic rasterization concept with a
    // scary name: Digital Differential Analyzer (DDA).
    //
    // a1------------------b1  -,
    // |                    |   |-> FIELD_RADIUS
    // |------the line------|  -'
    // |               •----|-------> Sample distance "pixel": { x: 0, y: -1 }
    // a2------------------b2        (Example: a2 = a1 + vertDiff)
    //
    // Rasterize distance vectors along the rectangle using DDA (ensure vdmap access is safe)
    let currentHorizLocation = a1;
    for (let i = 0; i < horizSteps; i++) {
      let currentLocation = currentHorizLocation;
      let currentVal = dist; // Start with the vector pointing from line to edge
      for (let j = 0; j < vertSteps; j++) {
        const { x, y } = this.roundVec(currentLocation); // Use rounded coords for map keys
        if (!this.vdmap[x]) this.vdmap[x] = []; // Initialize row if needed

        // Store the vector if it's the first one for this grid point or if it's shorter
        if (this.vdmap[x][y] === undefined) {
          this.vdmap[x][y] = this.roundVec(currentVal);
        } else {
          this.vdmap[x][y] =
            this.sumOfSquaresVec(this.vdmap[x][y]) <
                this.sumOfSquaresVec(this.roundVec(currentVal))
              ? this.vdmap[x][y]
              : this.roundVec(currentVal);
        }
        // Move to the next point along the perpendicular line
        currentVal = this.addVec(currentVal, vertIncrement);
        currentLocation = this.addVec(currentLocation, vertIncrement);
      }
      // Move to the next starting point along the parallel line
      currentHorizLocation = this.addVec(currentHorizLocation, horizIncrement);
    }

    //
    // "Render" a square cap at the endpoint.
    const upperLeftPoint = this.subVec(currentPoint, {
      x: this.FIELD_RADIUS,
      y: this.FIELD_RADIUS,
    });
    const bottomRightPoint = this.addVec(currentPoint, {
      x: this.FIELD_RADIUS,
      y: this.FIELD_RADIUS,
    });
    for (let x = upperLeftPoint.x; x < bottomRightPoint.x; x++) {
      for (let y = upperLeftPoint.y; y < bottomRightPoint.y; y++) {
        if (!this.vdmap[x]) this.vdmap[x] = []; // Initialize row if needed
        const val = this.subVec({ x, y }, currentPoint); // Vector from currentPoint to grid point (x, y)

        // Store the vector if it's the first one or shorter than the existing one
        if (this.vdmap[x][y] === undefined) {
          this.vdmap[x][y] = val;
        } else {
          this.vdmap[x][y] =
            this.sumOfSquaresVec(this.vdmap[x][y]) < this.sumOfSquaresVec(val)
              ? this.vdmap[x][y]
              : val;
        }
      }
    }

    //
    // Trial-and-error over and over to get a curve that fits nicely.
    let steps = 0;
    while (true) {
      // Force vectors to be applied to control points c1 and c2
      let f1: Coord = { x: 0, y: 0 }; // Force for c1
      let f2: Coord = { x: 0, y: 0 }; // Force for c2

      //
      // Create force vectors by checking the distance (with the vector
      // distance field we've built up above so it's fast) of our
      // iteratively-fitting curve and pushing the control points
      // in the direction that helps most.
      // Sample points along the current guess of the Bézier curve.
      for (let i = 0; i < this.NUM_SAMPLE_POINTS; i++) {
        const t = i / this.NUM_SAMPLE_POINTS;
        const point = this.roundVec(this.getPointAlongCurve(segment, t));
        const dist = this.getDistanceFromPolyline(point);
        const d = this.magnitudeVec(dist);
        const dx = dist.x;
        const dy = dist.y;
        let modifier = 1;
        if (this.SHOULD_USE_MY_FORCE_VECTORS) {
          if (t < 0.1 || t > 0.9) modifier = 10;
        }
        f1.x += t * Math.pow(1 - t, 2) * d * dx * modifier;
        f1.y += t * Math.pow(1 - t, 2) * d * dy * modifier;
        f2.x += Math.pow(t, 2) * (1 - t) * d * dx * modifier;
        f2.y += Math.pow(t, 2) * (1 - t) * d * dy * modifier;
      }

      //
      // Push the force vectors slightly toward the middle to mitigate
      // hooking at the end of the curve.
      if (this.SHOULD_USE_MY_FORCE_VECTORS) {
        const fromEnd = this.subVec(
          this.getSegmentMidpoint(segment),
          segment.c2,
        );
        const fromBeginning = this.subVec(
          this.getSegmentMidpoint(segment),
          segment.c1,
        );
        f1 = this.subVec(f1, this.multVec(fromBeginning, 0.03));
        f2 = this.subVec(f2, this.multVec(fromEnd, 0.03));
      }

      //
      // Constrain the first control point to adjust itself along the same
      // line as the previous segments second control point so the curve
      // looks continuous.
      if (segment.constrain_to) {
        f1 = this.multVec(
          segment.constrain_to,
          this.dotProduct(segment.constrain_to, f1),
        );
      }

      //
      // Apply the force vectors to the control points.
      const alpha = 1;
      segment.c1.x -= (alpha * f1.x * 6) / this.NUM_SAMPLE_POINTS;
      segment.c1.y -= (alpha * f1.y * 6) / this.NUM_SAMPLE_POINTS;
      segment.c2.x -= (alpha * f2.x * 6) / this.NUM_SAMPLE_POINTS;
      segment.c2.y -= (alpha * f2.y * 6) / this.NUM_SAMPLE_POINTS;

      //
      // Add up the error of the curve (again with our fast distance field).
      let error = 0;
      for (let i = 0; i < this.NUM_SAMPLE_POINTS; i++) {
        const t = i / this.NUM_SAMPLE_POINTS;
        const point = this.getPointAlongCurve(segment, t);
        const dist = this.getDistanceFromPolyline(this.roundVec(point));
        error += Math.pow(dist.x, 2) + Math.pow(dist.y, 2);
      }
      error = error / this.NUM_SAMPLE_POINTS;
      steps++;

      segment.error = error;
      segment.steps = steps;

      //
      // Do it all again unless the curve is good enough or we've been at it for a bit.
      if (error < this.MAX_ERROR || steps > this.MAX_ITERATIONS) break;
    }

    //
    // If we failed, reset the segment back to the way it was.
    if (steps > this.MAX_ITERATIONS) {
      segment.c0 = ogControls.c0;
      segment.c1 = ogControls.c1;
      segment.c2 = ogControls.c2;
      segment.c3 = ogControls.c3;
      segment.error = ogControls.error;
      segment.constrain_to = ogControls.constrain_to;
      segment.update_status = UpdateStatus.FAIL_MAXED;
      return segment;
    }

    segment.update_status = UpdateStatus.SUCCESS;
    return segment;
  }

  private getDistanceFromPolyline(point: Coord): Coord {
    const { x, y } = point;

    if (this.vdmap[x]) {
      const first = this.vdmap[x][y];
      const second = this.vdmap[x][y + 1];
      let val = first !== undefined
        ? { x: first.x, y: first.y }
        : { x: this.FIELD_RADIUS, y: this.FIELD_RADIUS };

      if (
        second && second.x !== undefined && Math.abs(val.x) > Math.abs(second.x)
      ) {
        val.x = second.x;
      }
      if (
        second && second.y !== undefined && Math.abs(val.y) > Math.abs(second.y)
      ) {
        val.y = second.y;
      }
      return val;
    } else {
      return { x: this.FIELD_RADIUS, y: this.FIELD_RADIUS };
    }
  }

  private getCurveEndTangent(curve: CurveSegment): Coord {
    const controlPoint = curve.c3;
    const endpoint = curve.c2;
    return {
      x: 3 * (controlPoint.x - endpoint.x),
      y: 3 * (controlPoint.y - endpoint.y),
    };
  }

  private checkCorner(curveSegment: CurveSegment, point: Coord): boolean {
    if (this.equalVec(curveSegment.c0, curveSegment.c3)) return false;

    let tan: Coord;
    if (this.SHOULD_USE_NEW_CORNER_FINDER) {
      tan = this.getUnitVector(
        this.subVec(
          this.getPointAlongCurve(curveSegment, 0.95),
          curveSegment.c3,
        ),
      );
    } else {
      tan = this.getUnitVector(this.getCurveEndTangent(curveSegment));
    }

    const newSegment = this.getUnitVector({
      x: point.x - curveSegment.c3.x,
      y: point.y - curveSegment.c3.y,
    });

    // Dot product == cos(angle) since the magnitude of unit vectors is 1
    const dot = this.dotProduct(tan, newSegment);
    const angle = this.radiansToDegrees(Math.acos(dot));

    return angle < this.MAX_CORNER_ANGLE;
  }

  // Vector utility methods
  private getSegmentMidpoint(segment: CurveSegment): Coord {
    return this.addVec(
      this.divVec(this.subVec(segment.c3, segment.c0), 2),
      segment.c0,
    );
  }

  private getScaledVectorDifference(uvec: Coord): Coord {
    return {
      x: uvec.x * this.FIELD_RADIUS,
      y: uvec.y * this.FIELD_RADIUS,
    };
  }

  private negateVec(vec: Coord): Coord {
    return { x: -vec.x, y: -vec.y };
  }

  private roundVec(vec: Coord): Coord {
    return { x: Math.round(vec.x), y: Math.round(vec.y) };
  }

  private sumOfSquaresVec(vec: Coord): number {
    return Math.pow(vec.x, 2) + Math.pow(vec.y, 2);
  }

  private magnitudeVec(vec: Coord): number {
    return Math.sqrt(this.sumOfSquaresVec(vec));
  }

  private addVec(a: Coord, b: Coord): Coord {
    return { x: a.x + b.x, y: a.y + b.y };
  }

  private subVec(a: Coord, b: Coord): Coord {
    return this.addVec(a, this.negateVec(b));
  }

  private multVec(a: Coord, scalarB: number): Coord {
    return { x: a.x * scalarB, y: a.y * scalarB };
  }

  private divVec(a: Coord, scalarB: number): Coord {
    return this.multVec(a, 1 / scalarB);
  }

  private equalVec(a: Coord, b: Coord): boolean {
    return a.x === b.x && a.y === b.y;
  }

  private dotProduct(a: Coord, b: Coord): number {
    return a.x * b.x + a.y * b.y;
  }

  private getMagnitude(point: Coord, p2: Coord = { x: 0, y: 0 }): number {
    return Math.sqrt(Math.pow(point.x - p2.x, 2) + Math.pow(point.y - p2.y, 2));
  }

  private radiansToDegrees(rad: number): number {
    return (rad * 180) / Math.PI;
  }

  private getDDASteps(diff: Coord): number {
    return Math.max(Math.abs(diff.x), Math.abs(diff.y));
  }

  private getPerpindicularUnitVector(a: Coord, b: Coord): Coord {
    const deltaX = b.x - a.x;
    const deltaY = b.y - a.y;
    const x = deltaY;
    const y = -deltaX;
    return this.getUnitVector({ x, y });
  }

  private getUnitVector(coord: Coord): Coord {
    const magnitude = this.getMagnitude(coord);
    return { x: coord.x / magnitude, y: coord.y / magnitude };
  }

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

  private singleComponentBezier(
    t: number,
    w0: number,
    w1: number,
    w2: number,
    w3: number,
  ): number {
    return (
      w0 * Math.pow(1 - t, 3) +
      w1 * 3 * t * Math.pow(1 - t, 2) +
      w2 * 3 * Math.pow(t, 2) * (1 - t) +
      w3 * Math.pow(t, 3)
    );
  }
}
