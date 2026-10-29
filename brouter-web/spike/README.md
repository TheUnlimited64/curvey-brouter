# Spike: the routing engine in the browser (curvey #259)

Not a release module. TeaVM 0.16 compiles the engine (core, mapaccess, expressions, codec, util) to
JavaScript and to WebAssembly (Wasm GC). Files reach the engine through TeaVM's virtual file system
(`JsFileSystem`), backed by `globalThis.brouterFs`; the engine still uses `RandomAccessFile` unchanged.

Browser replacements in `src/main/java/btools/util` shadow two brouter-util classes:
`StackSampler` (thread dumps) and `BuildVersion` (jar manifest). brouter-util is compiled from a copy
of its sources without those two files, so the replacements win.

## Run

    ./gradlew :brouter-web:generateJavaScript :brouter-web:generateWasmGC :brouter-web:copyWasmGCRuntime
    ./gradlew :brouter-web:jvmClasses :brouter-web:printJvmClasspath   # JVM baseline classpath

`bench.mjs` expects a data directory with `segments4/*.rd5`, `profiles2/lookups.dat`,
`profiles2/<case>.brf`, `calls/<case>.txt` (`lon,lat|...`) and the JVM reference output
`out-jvm-<case>.json` (from `btools.web.JvmMain`):

    node spike/bench.mjs wasm build/generated/teavm <dataDir> 3
