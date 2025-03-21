/// <reference lib="deno.ns" />

import { expect } from "jsr:@std/expect";
import { addToCurve, startCurve } from "./mod.ts";
import sampleLines from "../test-utils/sample-lines.js";

sampleLines.forEach((line) => {
  Deno.test("fits curves deterministically", () => {
    const [first, ...rest] = line;

    startCurve(first);
    expect(rest.reduce((_memo, point) => addToCurve(point)));
  });
});
