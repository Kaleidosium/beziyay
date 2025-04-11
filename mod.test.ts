/// <reference lib="deno.ns" />

import { expect, it } from "vitest";
import { Curve } from "./mod.ts";
import type { CurveSegment, Coord } from "./mod.ts";
import sampleLines from "./test-utils/sample-lines.js";

sampleLines.forEach((line: Coord[]) => {
  it("Fits curves deterministically", () => {
    const [first, ...rest] = line;

    if (!first) {
        throw new Error("Sample line is empty");
    }

    const curve = new Curve(first);

    let lastSegment: CurveSegment | undefined;
    rest.forEach(point => {
        lastSegment = curve.addToCurve(point);
    });

    expect(lastSegment).toMatchSnapshot();
  });
});
