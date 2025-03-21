/// <reference lib="deno.ns" />

import { expect, it } from 'vitest'
import { addToCurve, startCurve } from "./mod.ts";
import sampleLines from "./test-utils/sample-lines.js";

sampleLines.forEach((line) => {
  it("Fits curves deterministically", () => {
    const [first, ...rest] = line;

    startCurve(first);
    expect(rest.reduce((_memo, point) => addToCurve(point))).toMatchSnapshot();
  });
});
