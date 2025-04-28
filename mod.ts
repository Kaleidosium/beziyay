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
  error: number;
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

// --- Constants ---
// TODO: Add configuration settings rather than hardcoding constants

/** Maximum allowed error for curve fitting. */
const MAX_ERROR: number = 2;
/** Radius around the polyline segment to build the vector distance map. */
const FIELD_RADIUS: number = 9;
/** Number of points along the curve to sample for error calculation and force vector generation. */
const NUM_SAMPLE_POINTS: number = 9;
/** Maximum number of iterations for the curve fitting optimization loop. */
const MAX_ITERATIONS: number = 50;
/** Maximum angle (in degrees) between the curve's end tangent and the next line segment before declaring a corner. */
const MAX_CORNER_ANGLE: number = 80;
/** Minimum distance between points to process; points closer than this are ignored. */
const RADIAL_SIMPLIFICATION: number = 1;
/** Flag to use a modified corner detection method (tangent near end vs. exact end tangent). */
const SHOULD_USE_NEW_CORNER_FINDER: boolean = true;
/** Flag to use custom force vector modifications (weighting ends, pushing towards midpoint). */
const SHOULD_USE_MY_FORCE_VECTORS: boolean = true;

// --- Curve Class ---

/**
 * Represents a sequence of connected cubic Bézier curve segments
 * fitted to a polyline.
 */
export class Curve {
  /** The array of Bézier segments that make up the curve. */
  #segments: CurveSegment[] = [];
  /** A 2D map storing precomputed shortest vectors from grid points to the original polyline. */
  #vdmap: Coord[][] = []; // Vector distance map

  /**
   * Creates a new Curve instance, starting with a single point.
   * @param {Coord} startCoord - The initial coordinate to start the curve.
   */
  constructor(startCoord: Coord) {
    this.#initializeCurve(roundVec(startCoord));
  }

  // --- Private Initialization and Segment Access ---

  /**
   * Initializes the curve state with the first segment.
   * @param {Coord} coord - The starting coordinate (already rounded).
   * @private
   */
  #initializeCurve({ x, y }: Coord): void {
    this.#segments = [];
    this.#vdmap = [];
    this.#segments.push(this.#initCurveSegment(x, y));
  }

  /**
   * Creates an initial curve segment where all control points are the same.
   * @param {number} x - The x-coordinate.
   * @param {number} y - The y-coordinate.
   * @returns {CurveSegment} The initialized curve segment.
   * @private
   */
  #initCurveSegment(x: number, y: number): CurveSegment {
    const point = { x, y };
    // Start with a degenerate segment (all points the same)
    return {
      c0: point,
      c1: point,
      c2: point,
      c3: point,
      error: 0, // Initial error is zero
    };
  }

  /**
   * Gets the last segment in the curve sequence.
   * @returns {CurveSegment | undefined} The last segment, or undefined if the curve is empty.
   * @private
   */
  #getLastSegment(): CurveSegment | undefined {
    return this.#segments[this.#segments.length - 1];
  }

  /**
   * Gets the end point (c3) of a given segment.
   * @param {CurveSegment} segment - The segment to get the last point from.
   * @returns {Coord} The coordinates of the last point (c3).
   * @private
   */
  #getLastPoint(segment: CurveSegment): Coord {
    return segment.c3;
  }

  /**
   * Creates a deep copy of a curve segment.
   * @param {CurveSegment} segment - The segment to copy.
   * @returns {CurveSegment} A new segment object with the same values.
   * @private
   */
  #copySegment(segment: CurveSegment): CurveSegment {
    return {
      c0: { ...segment.c0 },
      c1: { ...segment.c1 },
      c2: { ...segment.c2 },
      c3: { ...segment.c3 },
      constrain_to: segment.constrain_to
        ? { ...segment.constrain_to }
        : undefined,
      error: segment.error || 0,
      update_status: segment.update_status,
      steps: segment.steps,
    };
  }

  // --- Public Method to Add Point ---

  /**
   * Adds a new point to the polyline and updates the curve fitting.
   * This may modify the last segment or add a new segment if a corner
   * is detected or the fitting fails to converge.
   * @param {Coord} coord - The new coordinate to add to the polyline.
   * @returns {CurveSegment | undefined} The last segment of the curve after the update, or undefined if initialization failed.
   */
  public addToCurve(coord: Coord): CurveSegment | undefined {
    const roundedCoord = roundVec(coord); // Work with integer coordinates for VDMap
    let segmentToUpdate = this.#getLastSegment();

    if (!segmentToUpdate) {
      console.error("Curve not initialized properly.");
      return undefined;
    }

    // Optimization: Ignore points too close together to avoid unnecessary work.
    if (
      getMagnitude(roundedCoord, this.#getLastPoint(segmentToUpdate)) <
        RADIAL_SIMPLIFICATION
    ) {
      // If the new point is too close, just return the current last segment without changes.
      return segmentToUpdate;
    }

    // --- Step 1: Handle state from *previous* point addition ---
    // Check if the *last* operation resulted in a failure status.
    // If so, we need to start a new segment before fitting the *current* point.
    if (
      segmentToUpdate.update_status &&
      segmentToUpdate.update_status !== UpdateStatus.SUCCESS
    ) {
      let newSegment: CurveSegment | undefined;
      const lastCurvePoint = this.#getLastPoint(segmentToUpdate);

      // Start a new, unconstrained segment for corners.
      if (segmentToUpdate.update_status === UpdateStatus.FAIL_CORNER) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
        // If the previous fit failed due to max iterations, start a new segment
        // constrained by the tangent of the previous segment's end.
      } else if (segmentToUpdate.update_status === UpdateStatus.FAIL_MAXED) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
        const prevTangent = getCurveEndTangent(segmentToUpdate);
        // Only apply constraint if the tangent is valid (non-zero magnitude)
        if (magnitudeVec(prevTangent) > 0) {
          newSegment.constrain_to = getUnitVector(prevTangent);
        }
      }

      if (newSegment) {
        // Reset the vector distance map because we're fitting a new curve!
        this.#vdmap = [];
        this.#segments.push(newSegment);
        segmentToUpdate = newSegment; // Target the newly added segment for the upcoming update
      } else {
        // This case should ideally not be reached if UpdateStatus is handled correctly.
        console.warn(
          "Unhandled update status before adding point:",
          segmentToUpdate.update_status,
        );
        // Proceed with the current last segment if status wasn't handled
      }
    }

    // --- Step 2: First attempt to update the target segment ---
    // 'segmentToUpdate' is now either the original last segment or a newly created one.
    // Call the core fitting logic.
    let updatedSegment = this.#updateDistanceField(
      segmentToUpdate,
      roundedCoord,
    );

    // --- Step 3: Retry Logic (if first attempt wasn't SUCCESS) ---
    // This replicates the original code's behavior of unconditionally retrying
    // if the first call to the implementation function didn't return SUCCESS.
    // Directly call #updateDistanceField again, operating on the *result* of the first attempt.
    if (updatedSegment.update_status !== UpdateStatus.SUCCESS) {
      // Retry fitting on the same segment state left by the first call.
      updatedSegment = this.#updateDistanceField(updatedSegment, roundedCoord);
    }

    // Return the final state of the last segment after attempts/retries.
    return updatedSegment;
  }

  // --- Private Distance Field Update ---

  /**
   * Updates the Vector Distance Map (VDMap) and iteratively fits the Bézier segment
   * to the polyline data represented by the VDMap.
   * @param {CurveSegment} segment - The specific segment to update.
   * @param {Coord} currentPoint - The new point added to the polyline (rounded).
   * @returns {CurveSegment} The updated segment with its final state and status.
   * @private
   */
  #updateDistanceField(
    segment: CurveSegment,
    currentPoint: Coord,
  ): CurveSegment {
    // **Important:** This method operates on the *segment* passed to it.

    const lastPoint = this.#getLastPoint(segment); // The end point of the segment *before* adding currentPoint
    const ogControls = this.#copySegment(segment); // Backup state in case fitting fails

    // First, check if adding this point creates a sharp corner.
    if (this.#checkCorner(segment, currentPoint)) {
      segment.update_status = UpdateStatus.FAIL_CORNER;
      return segment; // Return early if it's a corner
    }

    // --- Initial Guess for Control Points ---
    // Update the end point (c3) to the new point.
    segment.c3 = { ...currentPoint };
    // Make an initial guess for the second control point (c2) by shifting it
    // parallel to the line connecting the old end point and the new point.
    // The first control point (c1) remains unchanged initially.
    segment.c2 = {
      x: segment.c2.x + currentPoint.x - lastPoint.x,
      y: segment.c2.y + currentPoint.y - lastPoint.y,
    };

    // --- Build Vector Distance Map (VDMap) ---
    // The VDMap stores the shortest vector from grid points near the line segment
    // (lastPoint -> currentPoint) back to that line segment. This allows for
    // fast distance lookups during the fitting process.
    //
    // Prep the rectangle around our line in which we'll store distance-to-the-line
    // for each pixel in the box.
    const perpVec = getPerpindicularUnitVector(lastPoint, currentPoint);
    // Check if perpVec is valid (magnitude > 0)
    if (isNaN(perpVec.x) || isNaN(perpVec.y)) {
      // Handle degenerate case where lastPoint and currentPoint are the same
      console.warn(
        "Degenerate segment in updateDistanceField, points are identical.",
      );
      segment.update_status = UpdateStatus.SUCCESS; // Treat as success? Or a specific error?
      return segment;
    }

    const dist = getScaledVectorDifference(perpVec); // Vector scaled by FIELD_RADIUS
    const a1 = addVec(lastPoint, dist); // Corner of the bounding box
    const b1 = addVec(currentPoint, dist); // Opposite corner
    const vertDiff = negateVec(multVec(dist, 2)); // Vector across the short side of the box
    const vertSteps = getDDASteps(vertDiff); // Number of steps for DDA
    const vertIncrement = vertSteps > 0
      ? divVec(vertDiff, vertSteps)
      : { x: 0, y: 0 }; // DDA increment vector
    const horizDiff = subVec(b1, a1); // Vector across the long side of the box
    const horizSteps = getDDASteps(horizDiff); // Number of steps for DDA
    const horizIncrement = horizSteps > 0
      ? divVec(horizDiff, horizSteps)
      : { x: 0, y: 0 }; // DDA increment vector

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
        const { x: rx, y: ry } = roundVec(currentLocation); // Use rounded coords for map keys
        if (!this.#vdmap[rx]) this.#vdmap[rx] = []; // Initialize row if needed
        const roundedVal = roundVec(currentVal);
        const existingVal = this.#vdmap[rx][ry];
        // Store the vector if it's the first one for this grid point or if it's shorter
        if (
          existingVal === undefined ||
          sumOfSquaresVec(roundedVal) < sumOfSquaresVec(existingVal)
        ) {
          this.#vdmap[rx][ry] = roundedVal;
        }
        // Move to the next point along the perpendicular line
        currentVal = addVec(currentVal, vertIncrement);
        currentLocation = addVec(currentLocation, vertIncrement);
      }
      // Move to the next starting point along the parallel line
      currentHorizLocation = addVec(currentHorizLocation, horizIncrement);
    }

    //
    // "Render" a square cap at the endpoint (currentPoint) to handle distances near the end.
    // Ensure vdmap access is safe during cap rasterization.
    const upperLeftPoint = subVec(currentPoint, {
      x: FIELD_RADIUS,
      y: FIELD_RADIUS,
    });
    const bottomRightPoint = addVec(currentPoint, {
      x: FIELD_RADIUS,
      y: FIELD_RADIUS,
    });
    const startX = Math.round(upperLeftPoint.x);
    const endX = Math.round(bottomRightPoint.x);
    const startY = Math.round(upperLeftPoint.y);
    const endY = Math.round(bottomRightPoint.y);

    for (let x = startX; x < endX; x++) {
      if (!this.#vdmap[x]) this.#vdmap[x] = []; // Initialize row if needed
      for (let y = startY; y < endY; y++) {
        const val = subVec({ x, y }, currentPoint); // Vector from currentPoint to grid point (x, y)
        const existingVal = this.#vdmap[x][y];
        // Store the vector if it's the first one or shorter than the existing one
        if (
          existingVal === undefined ||
          sumOfSquaresVec(val) < sumOfSquaresVec(existingVal)
        ) {
          this.#vdmap[x][y] = val;
        }
      }
    }
    // --- End VDMap Build ---

    // --- Curve Fitting Iteration ---
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
      for (let i = 0; i < NUM_SAMPLE_POINTS; i++) {
        const t = i / NUM_SAMPLE_POINTS; // Parameter along the curve (0 to 1)
        const pointOnCurve = roundVec(getPointAlongCurve(segment, t)); // Point on the current curve guess
        const distVec = this.#getDistanceFromPolyline(pointOnCurve); // Get shortest vector from VDMap

        // Check if distVec is valid before using its properties
        if (
          distVec.x === undefined || distVec.y === undefined ||
          isNaN(distVec.x) || isNaN(distVec.y)
        ) {
          console.warn(
            `Invalid distance vector at t=${t}, point=`,
            pointOnCurve,
          );
          continue; // Skip this sample point if distance is invalid
        }

        const d = magnitudeVec(distVec); // Magnitude of the distance vector
        const dx = distVec.x; // x-component of distance vector
        const dy = distVec.y; // y-component of distance vector

        // Apply a modifier, potentially weighting points near the ends more heavily.
        let modifier = 1;
        if (SHOULD_USE_MY_FORCE_VECTORS && (t < 0.1 || t > 0.9)) {
          modifier = 10; // Increase force near endpoints
        }

        // Accumulate forces based on Bernstein polynomial basis functions and distance vector.
        // These formulas distribute the "pull" of the distance vector to the control points.
        if (!isNaN(dx) && !isNaN(dy) && !isNaN(d)) {
          // Force contribution to c1
          f1.x += t * Math.pow(1 - t, 2) * d * dx * modifier;
          f1.y += t * Math.pow(1 - t, 2) * d * dy * modifier;
          // Force contribution to c2
          f2.x += Math.pow(t, 2) * (1 - t) * d * dx * modifier;
          f2.y += Math.pow(t, 2) * (1 - t) * d * dy * modifier;
        }
      }

      //
      // Push the force vectors slightly toward the middle to mitigate
      // hooking at the end of the curve (optional modification).
      if (SHOULD_USE_MY_FORCE_VECTORS) {
        const midpoint = getSegmentMidpoint(segment);
        const fromEnd = subVec(midpoint, segment.c2); // Vector from c2 to midpoint
        const fromBeginning = subVec(midpoint, segment.c1); // Vector from c1 to midpoint
        // Apply a small counter-force pushing control points towards the segment midpoint
        f1 = subVec(f1, multVec(fromBeginning, 0.03));
        f2 = subVec(f2, multVec(fromEnd, 0.03));
      }

      //
      // Constrain the first control point (c1) to move along the tangent
      // defined by the previous segment's end, ensuring C1 continuity (smoothness).
      if (segment.constrain_to) {
        const constraintVec = segment.constrain_to; // Unit vector of the constraint direction
        // Project the calculated force f1 onto the constraint vector.
        if (magnitudeVec(f1) > 0) { // Avoid modifying if force is zero
          f1 = multVec(constraintVec, dotProduct(constraintVec, f1));
        }
      }

      //
      // Apply the calculated force vectors to the control points.
      // Check for NaN forces before applying.
      if (isNaN(f1.x) || isNaN(f1.y) || isNaN(f2.x) || isNaN(f2.y)) {
        console.warn(
          "NaN force vector detected, stopping iteration for this segment.",
        );
        // Restore original state as forces are invalid
        segment.c0 = ogControls.c0;
        segment.c1 = ogControls.c1;
        segment.c2 = ogControls.c2;
        segment.c3 = ogControls.c3;
        segment.error = ogControls.error;
        segment.constrain_to = ogControls.constrain_to;
        segment.update_status = UpdateStatus.FAIL_MAXED; // Indicate failure
        return segment;
      }

      const alpha = 1; // Step size factor (could be adjusted for convergence speed)
      // Update control points by subtracting the scaled forces.
      // The scaling factor 6 / NUM_SAMPLE_POINTS comes from the paper's formulation.
      segment.c1.x -= (alpha * f1.x * 6) / NUM_SAMPLE_POINTS;
      segment.c1.y -= (alpha * f1.y * 6) / NUM_SAMPLE_POINTS;
      segment.c2.x -= (alpha * f2.x * 6) / NUM_SAMPLE_POINTS;
      segment.c2.y -= (alpha * f2.y * 6) / NUM_SAMPLE_POINTS;

      //
      // Add up the error of the curve (again with our fast distance field).
      // Calculate the sum of squared distances from sample points on the *new* curve
      // guess to the polyline (via VDMap).
      let error = 0;
      for (let i = 0; i < NUM_SAMPLE_POINTS; i++) {
        const t = i / NUM_SAMPLE_POINTS;
        const point = getPointAlongCurve(segment, t); // Point on the updated curve
        const distVec = this.#getDistanceFromPolyline(roundVec(point)); // Distance from VDMap
        // Accumulate squared error safely
        if (
          distVec.x !== undefined && distVec.y !== undefined &&
          !isNaN(distVec.x) && !isNaN(distVec.y)
        ) {
          error += sumOfSquaresVec(distVec);
        }
      }
      // Check if error calculation is valid
      if (isNaN(error)) {
        console.warn("NaN error detected during fitting.");
        error = Infinity; // Treat NaN error as maximum error
      } else {
        // Average the squared error
        error = error / NUM_SAMPLE_POINTS;
      }

      steps++; // Increment iteration counter
      segment.error = error; // Store current error
      segment.steps = steps; // Store number of steps taken

      //
      // Check termination conditions:
      // Do it all again unless the curve is good enough (error below threshold)
      // or we've been at it for too long (max iterations reached).
      if (error < MAX_ERROR || steps >= MAX_ITERATIONS) { // Use >= for MAX_ITERATIONS check
        break; // Exit the fitting loop
      }
    }
    // --- End Curve Fitting ---

    //
    // Handle iteration failure (maxed out iterations AND error still too high).
    // If we failed, reset the segment back to the way it was before this update attempt.
    if (steps >= MAX_ITERATIONS && segment.error >= MAX_ERROR) { // Check error condition too
      // Restore segment state from the backup made at the start of this method.
      segment.c0 = ogControls.c0;
      segment.c1 = ogControls.c1;
      segment.c2 = ogControls.c2;
      segment.c3 = ogControls.c3;
      segment.error = ogControls.error;
      segment.constrain_to = ogControls.constrain_to;
      // Setting FAIL_MAXED seems correct as this specific update failed.
      segment.update_status = UpdateStatus.FAIL_MAXED;
      return segment;
    }

    // Success for this update! The segment fitting converged or finished within limits.
    segment.update_status = UpdateStatus.SUCCESS;
    return segment;
  }

  // --- Private Helper Methods (VDMap lookup, Corner check) ---

  /**
   * Gets the precomputed shortest distance vector from the polyline
   * to a given grid point using the Vector Distance Map (VDMap).
   * Includes a heuristic check with a neighbor point.
   * @param {Coord} point - The grid point (rounded coordinates) to query.
   * @returns {Coord} The shortest distance vector, or a default vector if outside the map.
   * @private
   */
  #getDistanceFromPolyline({ x, y }: Coord): Coord {
    const defaultValue = { x: FIELD_RADIUS, y: FIELD_RADIUS }; // Default if outside map

    // Basic bounds and initialization checks for the VDMap access
    if (!this.#vdmap || x < 0 || x >= this.#vdmap.length || !this.#vdmap[x]) {
      return defaultValue;
    }
    const column = this.#vdmap[x];
    if (y < 0 || y >= column.length) {
      return defaultValue;
    }

    const first = column[y]; // Get the vector stored at the exact grid point
    const val = first !== undefined ? { ...first } : defaultValue; // Use stored value or default

    // Heuristic: Check the next point vertically (y+1) in the VDMap.
    // Sometimes, the closest point on the polyline might be better represented
    // by the vector stored at a neighboring grid cell.
    if (y + 1 < column.length) { // Ensure y+1 is within bounds
      const second = column[y + 1];
      if (second !== undefined) {
        // If the x-component of the neighbor's vector is smaller (closer), use it.
        if (
          second.x !== undefined && !isNaN(second.x) &&
          Math.abs(val.x) > Math.abs(second.x)
        ) {
          val.x = second.x;
        }
        // If the y-component of the neighbor's vector is smaller (closer), use it.
        if (
          second.y !== undefined && !isNaN(second.y) &&
          Math.abs(val.y) > Math.abs(second.y)
        ) {
          val.y = second.y;
        }
      }
    }

    // Ensure returned value components are valid numbers.
    if (val.x === undefined || isNaN(val.x)) val.x = FIELD_RADIUS;
    if (val.y === undefined || isNaN(val.y)) val.y = FIELD_RADIUS;

    return val;
  }

  /**
   * Checks if adding a new point creates a sharp corner based on the angle
   * between the curve's end tangent and the vector to the new point.
   * @param {CurveSegment} curveSegment - The segment being extended.
   * @param {Coord} point - The new point being added.
   * @returns {boolean} True if a corner is detected, false otherwise.
   * @private
   */
  #checkCorner(curveSegment: CurveSegment, point: Coord): boolean {
    // Cannot form a corner if the segment start and end points are the same.
    if (equalVec(curveSegment.c0, curveSegment.c3)) return false;

    let tangent: Coord;
    // Calculate the tangent at or near the end of the current segment.
    if (SHOULD_USE_NEW_CORNER_FINDER) {
      // Use tangent based on a point slightly before the end (more robust for near-degenerate ends).
      const pointNearEnd = getPointAlongCurve(curveSegment, 0.95);
      // If point near end is same as end, cannot calculate tangent this way.
      if (equalVec(pointNearEnd, curveSegment.c3)) return false;
      tangent = getUnitVector(subVec(pointNearEnd, curveSegment.c3));
    } else {
      // Use the mathematical tangent exactly at the endpoint c3.
      tangent = getUnitVector(getCurveEndTangent(curveSegment));
    }

    // Check if tangent calculation was valid.
    if (isNaN(tangent.x) || isNaN(tangent.y)) return false;

    // Calculate the vector representing the new line segment to be added.
    const newSegmentVec = getUnitVector(subVec(point, curveSegment.c3));

    // Check if the new segment vector is valid (points are different).
    if (isNaN(newSegmentVec.x) || isNaN(newSegmentVec.y)) {
      return false; // Cannot determine angle if new point is same as last
    }

    // Calculate the angle using the dot product of the unit vectors.
    // Dot product == cos(angle) since the magnitude of unit vectors is 1.
    const dot = dotProduct(tangent, newSegmentVec);
    // Clamp dot product to [-1, 1] to avoid Math.acos errors due to floating point inaccuracies.
    const clampedDot = Math.max(-1, Math.min(1, dot));
    const angle = radiansToDegrees(Math.acos(clampedDot));

    // If the angle is less than the threshold, it's considered a corner.
    return angle < MAX_CORNER_ANGLE;
  }

  // --- Public Getter ---

  /**
   * Gets the array of curve segments.
   * @returns {ReadonlyArray<CurveSegment>} A read-only array of the segments.
   */
  get segments(): ReadonlyArray<CurveSegment> {
    return this.#segments;
  }
}

// --- Utility Functions ---

/**
 * Calculates the tangent vector at the end (t=1) of a cubic Bézier curve segment.
 * The tangent points from c3 towards c2.
 * @param {CurveSegment} curve - The curve segment.
 * @returns {Coord} The tangent vector.
 * @see {@link https://pomax.github.io/bezierinfo/#derivatives}
 */
function getCurveEndTangent(curve: CurveSegment): Coord {
  const endpoint = curve.c3;
  const controlPoint = curve.c2;
  // Formula for derivative at t=1: 3 * (c3 - c2) - we use c2-c3 and negate later if needed.
  // Actually, the tangent points from c3 towards c2, so it's 3*(c2-c3) or 3*(c3-c2) depending on definition.
  // Let's use 3 * (c2 - c3) as per standard derivative formula, pointing "out" of the curve end.
  // Wait, the formula is 3 * (P3 - P2). Let's stick to that.
  // Original code used 3 * (control_point.x - endpoint.x), which is 3 * (c2.x - c3.x).
  // Let's verify... Derivative B'(t) = 3(1-t)^2(P1-P0) + 6t(1-t)(P2-P1) + 3t^2(P3-P2)
  // At t=1, B'(1) = 3(1)^2(P3-P2) = 3(c3-c2).
  // The original code seems to have used 3*(c2-c3). Let's keep the original's behavior.
  return {
    x: 3 * (controlPoint.x - endpoint.x),
    y: 3 * (controlPoint.y - endpoint.y),
  };
}

/**
 * Calculates the midpoint of the line segment connecting the start (c0) and end (c3) points.
 * @param {CurveSegment} segment - The curve segment.
 * @returns {Coord} The midpoint coordinates.
 */
function getSegmentMidpoint(segment: CurveSegment): Coord {
  return addVec(divVec(subVec(segment.c3, segment.c0), 2), segment.c0);
}

/**
 * Scales a unit vector by the FIELD_RADIUS constant.
 * @param {Coord} uvec - The unit vector.
 * @returns {Coord} The scaled vector.
 */
function getScaledVectorDifference(uvec: Coord): Coord {
  return {
    x: uvec.x * FIELD_RADIUS,
    y: uvec.y * FIELD_RADIUS,
  };
}

/**
 * Calculates the coordinates of a point along a cubic Bézier curve at parameter t.
 * @param {CurveSegment} curve - The curve segment definition.
 * @param {number} t - The parameter value (usually between 0 and 1).
 * @returns {Coord} The coordinates of the point on the curve.
 * @see {@link https://pomax.github.io/bezierinfo/#explanation}
 */
function getPointAlongCurve(curve: CurveSegment, t: number): Coord {
  const x = singleComponentBezier(
    t,
    curve.c0.x,
    curve.c1.x,
    curve.c2.x,
    curve.c3.x,
  );
  const y = singleComponentBezier(
    t,
    curve.c0.y,
    curve.c1.y,
    curve.c2.y,
    curve.c3.y,
  );
  return { x, y };
}

/**
 * Calculates a single coordinate (x or y) of a point on a cubic Bézier curve using the Bernstein polynomial form.
 * B(t) = P0*(1-t)^3 + P1*3t(1-t)^2 + P2*3t^2(1-t) + P3*t^3
 * @param {number} t - The parameter value (usually between 0 and 1).
 * @param {number} w0 - Coordinate value of the start point (P0).
 * @param {number} w1 - Coordinate value of the first control point (P1).
 * @param {number} w2 - Coordinate value of the second control point (P2).
 * @param {number} w3 - Coordinate value of the end point (P3).
 * @returns {number} The calculated coordinate value.
 */
function singleComponentBezier(
  t: number,
  w0: number,
  w1: number,
  w2: number,
  w3: number,
): number {
  const t2 = t * t;
  const t3 = t2 * t;
  const mt = 1 - t;
  const mt2 = mt * mt;
  const mt3 = mt2 * mt;
  return (
    w0 * mt3 + // P0*(1-t)^3
    w1 * 3 * mt2 * t + // P1*3t(1-t)^2
    w2 * 3 * mt * t2 + // P2*3t^2(1-t)
    w3 * t3 // P3*t^3
  ); // Using expanded form for potential clarity/performance? Original used Math.pow. Let's keep original's pow form.
  /* Original calculation:
  return (
    w0 * Math.pow(1 - t, 3) +
    w1 * 3 * t * Math.pow(1 - t, 2) +
    w2 * 3 * Math.pow(t, 2) * (1 - t) +
    w3 * Math.pow(t, 3)
  ); // [cite: 365]
  */
}

// --- Vector Math Utilities ---

/** Rounds the components of a vector to the nearest integer. */
function roundVec(vec: Coord): Coord {
  return { x: Math.round(vec.x), y: Math.round(vec.y) };
}
/** Negates the components of a vector. */
function negateVec(vec: Coord): Coord {
  return { x: -vec.x, y: -vec.y };
}
/** Adds two vectors component-wise. */
function addVec(a: Coord, b: Coord): Coord {
  return { x: a.x + b.x, y: a.y + b.y };
}
/** Subtracts vector b from vector a component-wise. */
function subVec(a: Coord, b: Coord): Coord {
  return addVec(a, negateVec(b));
}
/** Multiplies a vector by a scalar value. */
function multVec(a: Coord, scalarB: number): Coord {
  return { x: a.x * scalarB, y: a.y * scalarB };
}
/** Divides a vector by a scalar value. */
function divVec(a: Coord, scalarB: number): Coord {
  // Avoid division by zero, return zero vector or throw error?
  // Original code didn't check. Let's assume scalarB is non-zero where used.
  return multVec(a, 1 / scalarB);
}
/** Calculates the sum of the squares of the vector's components. */
function sumOfSquaresVec(vec: Coord): number {
  return Math.pow(vec.x, 2) + Math.pow(vec.y, 2);
}
/** Calculates the magnitude (length) of a vector. */
function magnitudeVec(vec: Coord): number {
  return Math.sqrt(sumOfSquaresVec(vec));
}
/** Checks if two vectors are equal component-wise. */
function equalVec(a: Coord, b: Coord): boolean {
  return a.x === b.x && a.y === b.y;
}
/** Calculates the dot product of two vectors. */
function dotProduct(a: Coord, b: Coord): number {
  return a.x * b.x + a.y * b.y;
}
/** Calculates the Euclidean distance between two points (or magnitude of a vector if p2 is omitted). */
function getMagnitude(p1: Coord, p2: Coord = { x: 0, y: 0 }): number {
  return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}
/** Converts radians to degrees. */
function radiansToDegrees(rad: number): number {
  return (rad * 180) / Math.PI;
}
/** Calculates the maximum absolute difference in x or y components, used for DDA steps. */
function getDDASteps(diff: Coord): number {
  return Math.max(Math.abs(diff.x), Math.abs(diff.y));
}

/**
 * Calculates the unit vector (a vector with magnitude 1) in the same direction as the input vector.
 * Handles the case of a zero vector.
 * @param {Coord} vec - The input vector.
 * @returns {Coord} The corresponding unit vector, or {0, 0} if the input magnitude is 0.
 */
function getUnitVector({ x, y }: Coord): Coord {
  const magnitude = getMagnitude({ x, y });
  if (magnitude === 0) return { x: 0, y: 0 }; // Avoid division by zero
  return { x: x / magnitude, y: y / magnitude };
}

/**
 * Calculates a unit vector perpendicular to the line segment defined by points a and b.
 * Rotates the vector (b-a) by 90 degrees.
 * @param {Coord} a - The starting point of the line segment.
 * @param {Coord} b - The ending point of the line segment.
 * @returns {Coord} The perpendicular unit vector.
 */
function getPerpindicularUnitVector(a: Coord, b: Coord): Coord {
  const deltaX = b.x - a.x;
  const deltaY = b.y - a.y;
  // Rotate (dx, dy) by 90 degrees -> (-dy, dx) or (dy, -dx)
  // Original code used (dy, -dx)
  const x = deltaY;
  const y = -deltaX;
  return getUnitVector({ x, y });
}
