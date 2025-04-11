/**
 * Math for Making Bézier Curves Out of Lines.
 *
 * Implements a modified version of the algorithm defined in the paper:
 * S. F. Frisken, “Efficient Curve Fitting,” Journal of Graphics Tools, vol. 13, no. 2, pp. 37–54, Jan. 2008.
 * doi:10.1080/2151237x.2008.10129260
 */

// --- Interfaces and Object Literals ---

export interface Coord {
  x: number;
  y: number;
}

export interface CurveSegment {
  c0: Coord;
  c1: Coord;
  c2: Coord;
  c3: Coord;
  constrain_to?: Coord; // Optional constraint vector
  error: number;
  update_status?: UpdateStatus; // Optional status
  steps?: number; // Optional debug info
}

const UpdateStatus = {
  SUCCESS: 1,
  FAIL_CORNER: 2,
  FAIL_MAXED: 3,
} as const;
type UpdateStatus = typeof UpdateStatus[keyof typeof UpdateStatus];

// --- Constants ---

// TODO: Add configuration settings rather than hardcoding constants
const MAX_ERROR: number = 2; // [cite: 6]
const FIELD_RADIUS: number = 9; // [cite: 6]
const NUM_SAMPLE_POINTS: number = 9; // [cite: 6]
const MAX_ITERATIONS: number = 50; // [cite: 6]
const MAX_CORNER_ANGLE: number = 80; // [cite: 6]
const RADIAL_SIMPLIFICATION: number = 1; // [cite: 7]
const SHOULD_USE_NEW_CORNER_FINDER: boolean = true; // [cite: 7]
const SHOULD_USE_MY_FORCE_VECTORS: boolean = true; // [cite: 7]

// --- Curve Class ---

export class Curve {
  #segments: CurveSegment[] = [];
  #vdmap: Coord[][] = [];

  constructor(startCoord: Coord) {
    this.#initializeCurve(roundVec(startCoord));
  }

  // --- Private Initialization and Segment Access ---
  #initializeCurve({ x, y }: Coord): void {
    this.#segments = [];
    this.#vdmap = [];
    this.#segments.push(this.#initCurveSegment(x, y));
  }

  #initCurveSegment(x: number, y: number): CurveSegment {
    const point = { x, y };
    return {
      c0: point,
      c1: point,
      c2: point,
      c3: point,
      error: 0,
    };
  }

  #getLastSegment(): CurveSegment | undefined {
    return this.#segments[this.#segments.length - 1];
  }

  #getLastPoint(segment: CurveSegment): Coord {
    return segment.c3;
  }

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
  public addToCurve(coord: Coord): CurveSegment | undefined {
    const roundedCoord = roundVec(coord);
    let segmentToUpdate = this.#getLastSegment();

    if (!segmentToUpdate) {
      console.error("Curve not initialized properly.");
      return undefined;
    }

    // Optimization: Ignore points too close
    if (
      getMagnitude(roundedCoord, this.#getLastPoint(segmentToUpdate)) <
        RADIAL_SIMPLIFICATION
    ) {
      return segmentToUpdate;
    }

    // --- Step 1: Handle state from *previous* point addition ---
    // Create a new segment if the *last* operation ended in failure.
    // This happens *before* attempting to fit the current 'roundedCoord'.
    if (
      segmentToUpdate.update_status &&
      segmentToUpdate.update_status !== UpdateStatus.SUCCESS
    ) {
      let newSegment: CurveSegment | undefined;
      const lastCurvePoint = this.#getLastPoint(segmentToUpdate);

      if (segmentToUpdate.update_status === UpdateStatus.FAIL_CORNER) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
      } else if (segmentToUpdate.update_status === UpdateStatus.FAIL_MAXED) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
        const prevTangent = getCurveEndTangent(segmentToUpdate);
        if (magnitudeVec(prevTangent) > 0) {
          newSegment.constrain_to = getUnitVector(prevTangent);
        }
      }

      if (newSegment) {
        this.#vdmap = []; // Reset VDMap for the new segment
        this.#segments.push(newSegment);
        segmentToUpdate = newSegment; // Target the newly added segment for the upcoming update
      } else {
        console.warn(
          "Unhandled update status before adding point:",
          segmentToUpdate.update_status,
        );
        // Proceed with the current last segment
      }
    }

    // --- Step 2: First attempt to update the target segment ---
    // 'segmentToUpdate' is now either the original last segment or a newly created one.
    let updatedSegment = this.#updateDistanceField(
      segmentToUpdate,
      roundedCoord,
    );

    // --- Step 3: Retry Logic (if first attempt wasn't SUCCESS) ---
    // Directly call #updateDistanceField again, operating on the *result* of the first attempt.
    if (updatedSegment.update_status !== UpdateStatus.SUCCESS) {
      // Retry fitting on the same segment state left by the first call.
      updatedSegment = this.#updateDistanceField(updatedSegment, roundedCoord);
    }

    return updatedSegment;
  }

  // --- Private Helper for Core Update Logic (Called by addToCurve) ---
  #internalCurveUpdateLogic(roundedCoord: Coord): CurveSegment | undefined {
    let segmentToUpdate = this.#getLastSegment();

    if (!segmentToUpdate) return undefined;

    // Handle cases where the *existing* last segment had a failed status *before* adding the new point
    if (
      segmentToUpdate.update_status &&
      segmentToUpdate.update_status !== UpdateStatus.SUCCESS
    ) {
      let newSegment: CurveSegment | undefined;
      const lastCurvePoint = this.#getLastPoint(segmentToUpdate);

      if (segmentToUpdate.update_status === UpdateStatus.FAIL_CORNER) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
      } else if (segmentToUpdate.update_status === UpdateStatus.FAIL_MAXED) {
        newSegment = this.#initCurveSegment(lastCurvePoint.x, lastCurvePoint.y);
        // Apply constraint from the previous segment's end tangent
        const prevTangent = getCurveEndTangent(segmentToUpdate);
        if (magnitudeVec(prevTangent) > 0) { // Avoid normalizing zero vector
          newSegment.constrain_to = getUnitVector(prevTangent);
        }
      }

      if (newSegment) {
        this.#vdmap = []; // Reset VDMap only when a new segment is truly added due to failure
        this.#segments.push(newSegment);
        segmentToUpdate = newSegment; // Target the newly added segment for the upcoming update
      } else {
        // If status was non-SUCCESS but didn't match FAIL_CORNER or FAIL_MAXED,
        // we might just proceed with the current segment or log an error.
        // For now, proceed with the current last segment.
        console.warn(
          "Unhandled update status before adding point:",
          segmentToUpdate.update_status,
        );
      }
    }

    // Update the distance field and fit the curve for the *targeted* segment (either old last or new)
    // Pass the segment explicitly to updateDistanceField
    return this.#updateDistanceField(segmentToUpdate, roundedCoord);
  }

  // --- Private Distance Field Update (Needs segment passed explicitly) ---
  #updateDistanceField(
    segment: CurveSegment,
    currentPoint: Coord,
  ): CurveSegment {
    // **Important:** This method now operates on the *segment* passed to it,
    // not necessarily this.#getLastSegment().

    const lastPoint = this.#getLastPoint(segment);
    const ogControls = this.#copySegment(segment); // Backup state of the segment being updated

    // Check for corner using the segment being updated
    if (this.#checkCorner(segment, currentPoint)) {
      segment.update_status = UpdateStatus.FAIL_CORNER;
      return segment; // Return early if it's a corner
    }

    // Update segment control points based on the new currentPoint
    segment.c3 = { ...currentPoint };
    segment.c2 = {
      x: segment.c2.x + currentPoint.x - lastPoint.x,
      y: segment.c2.y + currentPoint.y - lastPoint.y,
    };

    // --- Build Vector Distance Map (VDMap) ---
    // This part uses 'this.#vdmap' which is the instance's map.
    // It's rebuilt based on the line between lastPoint and currentPoint of the *current* segment update.
    const perpVec = getPerpindicularUnitVector(lastPoint, currentPoint);
    // Check if perpVec is valid (magnitude > 0)
    if (isNaN(perpVec.x) || isNaN(perpVec.y)) {
      // Handle degenerate case where lastPoint and currentPoint are the same
      console.warn(
        "Degenerate segment in updateDistanceField, points are identical.",
      );
      segment.update_status = UpdateStatus.SUCCESS; // Or some other appropriate status
      return segment;
    }

    const dist = getScaledVectorDifference(perpVec);
    const a1 = addVec(lastPoint, dist);
    const b1 = addVec(currentPoint, dist);
    const vertDiff = negateVec(multVec(dist, 2));
    const vertSteps = getDDASteps(vertDiff);
    const vertIncrement = vertSteps > 0
      ? divVec(vertDiff, vertSteps)
      : { x: 0, y: 0 }; // Avoid division by zero
    const horizDiff = subVec(b1, a1);
    const horizSteps = getDDASteps(horizDiff);
    const horizIncrement = horizSteps > 0
      ? divVec(horizDiff, horizSteps)
      : { x: 0, y: 0 }; // Avoid division by zero

    // Rasterize distance vectors (ensure vdmap access is safe)
    let currentHorizLocation = a1;
    for (let i = 0; i < horizSteps; i++) {
      let currentLocation = currentHorizLocation;
      let currentVal = dist;
      for (let j = 0; j < vertSteps; j++) {
        const { x: rx, y: ry } = roundVec(currentLocation); // Rounded coords for map keys
        if (!this.#vdmap[rx]) this.#vdmap[rx] = [];
        const roundedVal = roundVec(currentVal);
        const existingVal = this.#vdmap[rx][ry];
        if (
          existingVal === undefined ||
          sumOfSquaresVec(roundedVal) < sumOfSquaresVec(existingVal)
        ) {
          this.#vdmap[rx][ry] = roundedVal;
        }
        currentVal = addVec(currentVal, vertIncrement);
        currentLocation = addVec(currentLocation, vertIncrement);
      }
      currentHorizLocation = addVec(currentHorizLocation, horizIncrement);
    }

    // Rasterize end cap (ensure vdmap access is safe)
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
      if (!this.#vdmap[x]) this.#vdmap[x] = [];
      for (let y = startY; y < endY; y++) {
        const val = subVec({ x, y }, currentPoint);
        const existingVal = this.#vdmap[x][y];
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
    let steps = 0;
    while (true) {
      let f1: Coord = { x: 0, y: 0 };
      let f2: Coord = { x: 0, y: 0 };

      // Calculate forces using the VDMap and the *current state* of the 'segment' being fitted
      for (let i = 0; i < NUM_SAMPLE_POINTS; i++) {
        const t = i / NUM_SAMPLE_POINTS;
        const pointOnCurve = roundVec(getPointAlongCurve(segment, t)); // Use passed segment
        const distVec = this.#getDistanceFromPolyline(pointOnCurve); // Use instance vdmap
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
        const d = magnitudeVec(distVec);
        const dx = distVec.x;
        const dy = distVec.y;
        let modifier = 1;
        if (SHOULD_USE_MY_FORCE_VECTORS && (t < 0.1 || t > 0.9)) {
          modifier = 10;
        }
        // Accumulate forces safely
        if (!isNaN(dx) && !isNaN(dy) && !isNaN(d)) {
          f1.x += t * Math.pow(1 - t, 2) * d * dx * modifier;
          f1.y += t * Math.pow(1 - t, 2) * d * dy * modifier;
          f2.x += Math.pow(t, 2) * (1 - t) * d * dx * modifier;
          f2.y += Math.pow(t, 2) * (1 - t) * d * dy * modifier;
        }
      }

      // Mitigate hooking
      if (SHOULD_USE_MY_FORCE_VECTORS) {
        const midpoint = getSegmentMidpoint(segment); // Use passed segment
        const fromEnd = subVec(midpoint, segment.c2);
        const fromBeginning = subVec(midpoint, segment.c1);
        f1 = subVec(f1, multVec(fromBeginning, 0.03));
        f2 = subVec(f2, multVec(fromEnd, 0.03));
      }

      // Apply constraint
      if (segment.constrain_to) {
        const constraintVec = segment.constrain_to;
        if (magnitudeVec(f1) > 0) { // Avoid modifying if force is zero
          f1 = multVec(constraintVec, dotProduct(constraintVec, f1));
        }
      }

      // Apply forces (check for NaN forces)
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
        segment.update_status = UpdateStatus.FAIL_MAXED; // Or a new status like FAIL_NAN
        return segment;
      }

      const alpha = 1; // Step size factor
      segment.c1.x -= (alpha * f1.x * 6) / NUM_SAMPLE_POINTS;
      segment.c1.y -= (alpha * f1.y * 6) / NUM_SAMPLE_POINTS;
      segment.c2.x -= (alpha * f2.x * 6) / NUM_SAMPLE_POINTS;
      segment.c2.y -= (alpha * f2.y * 6) / NUM_SAMPLE_POINTS;

      // Calculate error
      let error = 0;
      for (let i = 0; i < NUM_SAMPLE_POINTS; i++) {
        const t = i / NUM_SAMPLE_POINTS;
        const point = getPointAlongCurve(segment, t); // Use passed segment
        const distVec = this.#getDistanceFromPolyline(roundVec(point)); // Use instance vdmap
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
        error = error / NUM_SAMPLE_POINTS;
      }

      steps++;
      segment.error = error;
      segment.steps = steps;

      // Check termination conditions
      if (error < MAX_ERROR || steps >= MAX_ITERATIONS) { // Use >= for MAX_ITERATIONS
        break;
      }
    }
    // --- End Curve Fitting ---

    // Handle iteration failure (maxed out)
    if (steps >= MAX_ITERATIONS && segment.error >= MAX_ERROR) { // Check error condition too
      // Reset segment to original state before iterations started for *this update*
      segment.c0 = ogControls.c0;
      segment.c1 = ogControls.c1;
      segment.c2 = ogControls.c2;
      segment.c3 = ogControls.c3;
      segment.error = ogControls.error;
      segment.constrain_to = ogControls.constrain_to;
      // Keep potential status from ogControls if it existed? Or set FAIL_MAXED?
      // Setting FAIL_MAXED seems correct as this specific update failed.
      segment.update_status = UpdateStatus.FAIL_MAXED;
      return segment;
    }

    // Success for this update!
    segment.update_status = UpdateStatus.SUCCESS;
    return segment;
  }

  // --- Private Helper Methods (VDMap lookup, Corner check) ---
  #getDistanceFromPolyline({ x, y }: Coord): Coord {
    const defaultValue = { x: FIELD_RADIUS, y: FIELD_RADIUS };
    if (!this.#vdmap || x < 0 || x >= this.#vdmap.length || !this.#vdmap[x]) {
      return defaultValue;
    }
    const column = this.#vdmap[x];
    if (y < 0 || y >= column.length) {
      return defaultValue;
    }

    const first = column[y];
    const val = first !== undefined ? { ...first } : defaultValue;

    // Check neighbour heuristic (ensure y+1 is in bounds)
    if (y + 1 < column.length) {
      const second = column[y + 1];
      if (second !== undefined) {
        if (
          second.x !== undefined && !isNaN(second.x) &&
          Math.abs(val.x) > Math.abs(second.x)
        ) {
          val.x = second.x;
        }
        if (
          second.y !== undefined && !isNaN(second.y) &&
          Math.abs(val.y) > Math.abs(second.y)
        ) {
          val.y = second.y;
        }
      }
    }

    // Ensure returned value is not undefined/NaN
    if (val.x === undefined || isNaN(val.x)) val.x = FIELD_RADIUS;
    if (val.y === undefined || isNaN(val.y)) val.y = FIELD_RADIUS;

    return val;
  }

  #checkCorner(curveSegment: CurveSegment, point: Coord): boolean {
    if (equalVec(curveSegment.c0, curveSegment.c3)) return false;

    let tangent: Coord;
    if (SHOULD_USE_NEW_CORNER_FINDER) {
      // Ensure point 0.95 is different from c3 before calculating tangent
      const pointNearEnd = getPointAlongCurve(curveSegment, 0.95);
      if (equalVec(pointNearEnd, curveSegment.c3)) return false; // Cannot determine tangent
      tangent = getUnitVector(subVec(pointNearEnd, curveSegment.c3));
    } else {
      tangent = getUnitVector(getCurveEndTangent(curveSegment));
    }

    // Check if tangent is valid
    if (isNaN(tangent.x) || isNaN(tangent.y)) return false;

    // Vector for the new segment
    const newSegmentVec = getUnitVector(subVec(point, curveSegment.c3));

    // Check if new segment vector is valid
    if (isNaN(newSegmentVec.x) || isNaN(newSegmentVec.y)) {
      return false; // Cannot determine angle if new point is same as last
    }

    const dot = dotProduct(tangent, newSegmentVec);
    const clampedDot = Math.max(-1, Math.min(1, dot)); // Clamp for safety
    const angle = radiansToDegrees(Math.acos(clampedDot));

    return angle < MAX_CORNER_ANGLE;
  }

  // Public getter for segments
  get segments(): ReadonlyArray<CurveSegment> {
    return this.#segments;
  }
}

// --- Utility Functions ---

function getCurveEndTangent(curve: CurveSegment): Coord { // [cite: 81]
  const endpoint = curve.c3;
  const controlPoint = curve.c2;
  return {
    x: 3 * (controlPoint.x - endpoint.x), // [cite: 82]
    y: 3 * (controlPoint.y - endpoint.y), // [cite: 82]
  };
}

function getSegmentMidpoint(segment: CurveSegment): Coord { // [cite: 88]
  return addVec(divVec(subVec(segment.c3, segment.c0), 2), segment.c0);
}

function getScaledVectorDifference(uvec: Coord): Coord { // [cite: 89]
  return {
    x: uvec.x * FIELD_RADIUS,
    y: uvec.y * FIELD_RADIUS,
  };
}

function getPointAlongCurve(curve: CurveSegment, t: number): Coord { // [cite: 104]
  const x = singleComponentBezier(
    t,
    curve.c0.x,
    curve.c1.x,
    curve.c2.x,
    curve.c3.x,
  ); // [cite: 104]
  const y = singleComponentBezier(
    t,
    curve.c0.y,
    curve.c1.y,
    curve.c2.y,
    curve.c3.y,
  ); // [cite: 105]
  return { x, y }; // [cite: 106]
}

function singleComponentBezier(
  t: number,
  w0: number,
  w1: number,
  w2: number,
  w3: number,
): number { // [cite: 106]
  return (
    w0 * Math.pow(1 - t, 3) +
    w1 * 3 * t * Math.pow(1 - t, 2) +
    w2 * 3 * Math.pow(t, 2) * (1 - t) +
    w3 * Math.pow(t, 3)
  ); // [cite: 107]
}

// --- Vector Math Utilities ---

function roundVec(vec: Coord): Coord {
  return { x: Math.round(vec.x), y: Math.round(vec.y) };
} // [cite: 91]
function negateVec(vec: Coord): Coord {
  return { x: -vec.x, y: -vec.y };
} // [cite: 90]
function addVec(a: Coord, b: Coord): Coord {
  return { x: a.x + b.x, y: a.y + b.y };
} // [cite: 93]
function subVec(a: Coord, b: Coord): Coord {
  return addVec(a, negateVec(b));
} // [cite: 94]
function multVec(a: Coord, scalarB: number): Coord {
  return { x: a.x * scalarB, y: a.y * scalarB };
} // [cite: 94]
function divVec(a: Coord, scalarB: number): Coord {
  return multVec(a, 1 / scalarB);
} // [cite: 95]
function sumOfSquaresVec(vec: Coord): number {
  return Math.pow(vec.x, 2) + Math.pow(vec.y, 2);
} // [cite: 92]
function magnitudeVec(vec: Coord): number {
  return Math.sqrt(sumOfSquaresVec(vec));
} // [cite: 92]
function equalVec(a: Coord, b: Coord): boolean {
  return a.x === b.x && a.y === b.y;
} // [cite: 96]
function dotProduct(a: Coord, b: Coord): number {
  return a.x * b.x + a.y * b.y;
} // [cite: 97]
function getMagnitude(p1: Coord, p2: Coord = { x: 0, y: 0 }): number { // [cite: 98]
  return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}
function radiansToDegrees(rad: number): number {
  return (rad * 180) / Math.PI;
} // [cite: 99]
function getDDASteps(diff: Coord): number {
  return Math.max(Math.abs(diff.x), Math.abs(diff.y));
} // [cite: 99]

function getUnitVector({ x, y }: Coord): Coord { // [cite: 102]
  const magnitude = getMagnitude({ x, y }); // [cite: 102]
  if (magnitude === 0) return { x: 0, y: 0 }; // Avoid division by zero
  return { x: x / magnitude, y: y / magnitude }; // [cite: 103]
}

function getPerpindicularUnitVector(a: Coord, b: Coord): Coord { // [cite: 100]
  const deltaX = b.x - a.x; // [cite: 100]
  const deltaY = b.y - a.y; // [cite: 100]
  const x = deltaY; // [cite: 101]
  const y = -deltaX; // [cite: 101]
  return getUnitVector({ x, y }); // [cite: 101]
}
