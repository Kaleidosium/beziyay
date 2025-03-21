import * as esbuild from "esbuild";
import { denoPlugins } from "esbuild-deno-loader";

await esbuild.build({
  plugins: [...denoPlugins()],
  entryPoints: ["./mod.ts"],
  outfile: "./dist/beziyay.esm.js",
  bundle: true,
  platform: "browser",
  minify: true,
  target: "es2020",
  sourcemap: true,
  treeShaking: true,
  format: "esm",
});

esbuild.stop();
