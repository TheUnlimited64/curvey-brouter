# brouter-web

The routing engine for the Curvey web app, compiled to WebAssembly (Wasm GC) with TeaVM 0.16.

    ./gradlew :brouter-web:test :brouter-web:generateWasmGC :brouter-web:copyWasmGCRuntime

Output in `build/generated/teavm/wasm-gc/`: `brouter.wasm` and its loader `brouter.wasm-runtime.js`
(defines `globalThis.TeaVM.wasmGC.load`). After loading, call `exports.main([])` once, then
`exports.route(lonlats, profilePath, segmentDir, requestParams, profileParams)` (see `WebRouter`).

## Files
The engine reads files through `JsFileSystem`, which calls the host's `globalThis.brouterFs`
(`size`, `read`, optional `list`). The calls are synchronous, so run the engine in a Web Worker.
The profile directory must contain `lookups.dat` next to the profile.

## Why Wasm GC, not JS
TeaVM's JS backend computes Java `float` as double, so route costs differ from the JVM engine.
Wasm GC keeps float arithmetic: on 45 recorded Curvey roundtrip requests its output was
byte-identical to the JVM engine and as fast (Curvey #259).

## Browser replacements
`src/main/java/btools/util` replaces two brouter-util classes: `StackSampler` (thread dumps) and
`BuildVersion` (jar manifest). brouter-util is compiled from a copy of its sources without them.

## Composite builds
`settings.gradle` includes this module only in a standalone build (`gradle.parent == null`), so
Curvey's Android app, which includes this repository, never resolves TeaVM.
