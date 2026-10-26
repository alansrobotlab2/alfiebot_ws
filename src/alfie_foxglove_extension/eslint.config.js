// @ts-check

const foxglove = require("@foxglove/eslint-plugin");
const globals = require("globals");
const tseslint = require("typescript-eslint");

module.exports = tseslint.config({
  files: ["src/**/*.ts", "src/**/*.tsx"],
  extends: [foxglove.configs.base, foxglove.configs.react, foxglove.configs.typescript],
  languageOptions: {
    globals: {
      ...globals.es2020,
      ...globals.browser,
    },
    parserOptions: {
      project: "tsconfig.json",
      tsconfigRootDir: __dirname,
    },
  },
  rules: {
    "react-hooks/exhaustive-deps": "error",
    // The panel is a port of the node's web UI, whose toggles are plain
    // boolean callbacks (onLoop(on), setPlaying(playing)); keep them.
    "@foxglove/no-boolean-parameters": "off",
  },
});
