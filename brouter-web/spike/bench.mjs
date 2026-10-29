// Spike #259 harness: route recorded requests with the TeaVM builds of the fork engine in Node.
// Usage: node bench.mjs <js|wasm> <buildDir> <dataDir> [runs]
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';

const [, , target, buildDir, dataDirArg, runsArg] = process.argv;
const dataDir = path.resolve(dataDirArg);
const runs = Number(runsArg ?? 3);
const fds = new Map();
const stats = { reads: 0, bytes: 0, ranges: new Map(), all: new Map() };

globalThis.brouterFs = {
  size(p) {
    try {
      const s = fs.statSync(p);
      return s.isDirectory() ? -2 : s.size;
    } catch {
      return -1;
    }
  },
  read(p, offset, length) {
    let fd = fds.get(p);
    if (fd === undefined) { fd = fs.openSync(p, 'r'); fds.set(p, fd); }
    const out = new Int8Array(length);
    const n = fs.readSync(fd, out, 0, length, offset);
    stats.reads += 1;
    stats.bytes += n;
    if (p.endsWith('.rd5')) { stats.ranges.set(`${offset}:${n}`, n); stats.all.set(`${offset}:${n}`, n); }
    return n === length ? out : out.subarray(0, n);
  },
  list(p) { try { return fs.readdirSync(p); } catch { return []; } },
};

let route;
if (target === 'js') {
  const mod = await import(path.join(buildDir, 'js', 'brouter.js'));
  mod.main?.([]);
  route = mod.route;
} else {
  vm.runInThisContext(fs.readFileSync(path.join(buildDir, 'wasm-gc', 'brouter.wasm-runtime.js'), 'utf8'));
  const teavm = await globalThis.TeaVM.wasmGC.load(fs.readFileSync(path.join(buildDir, 'wasm-gc', 'brouter.wasm')));
  teavm.exports.main([]);
  route = teavm.exports.route;
}

const callsDir = path.join(dataDir, 'calls');
const results = [];
for (const file of fs.readdirSync(callsDir).sort()) {
  const name = file.replace(/\.txt$/, '');
  const jvmOut = path.join(dataDir, `out-jvm-${name}.json`);
  if (!fs.existsSync(jvmOut) || !fs.readFileSync(jvmOut, 'utf8').includes('"features"')) continue;
  const lonlats = fs.readFileSync(path.join(callsDir, file), 'utf8');
  const times = [];
  let json = '';
  let firstRunRd5 = 0;
  for (let i = 0; i < runs; i += 1) {
    stats.ranges.clear();
    const start = performance.now();
    json = route(lonlats, path.join(dataDir, 'profiles2', `${name}.brf`), path.join(dataDir, 'segments4'), '');
    times.push(performance.now() - start);
    if (i === 0) firstRunRd5 = [...stats.ranges.values()].reduce((a, b) => a + b, 0);
  }
  const ours = JSON.parse(json);
  const ref = JSON.parse(fs.readFileSync(jvmOut, 'utf8'));
  const a = ours.features?.[0];
  const b = ref.features[0];
  const same = a !== undefined
    && JSON.stringify(a.geometry.coordinates) === JSON.stringify(b.geometry.coordinates)
    && a.properties['track-length'] === b.properties['track-length']
    && a.properties['total-time'] === b.properties['total-time']
    && JSON.stringify(a.properties.messages) === JSON.stringify(b.properties.messages);
  results.push({ name, same, km: Number(b.properties['track-length']) / 1000, first: times[0], warm: Math.min(...times.slice(1)), rd5MB: firstRunRd5 / 1e6 });
  if (!same) console.log('DIFF', name, ours.error ?? '');
}
const mem = process.memoryUsage();
console.log(JSON.stringify({ target, cases: results.length, identical: results.filter((r) => r.same).length,
  firstMsMedian: median(results.map((r) => r.first)), warmMsMedian: median(results.map((r) => r.warm)),
  warmMsMax: Math.max(...results.map((r) => r.warm)), kmMedian: median(results.map((r) => r.km)),
  rd5MBFirstRunMedian: median(results.map((r) => r.rd5MB)), rd5MBFirstRunMax: Math.max(...results.map((r) => r.rd5MB)),
  rd5MBAllCasesUnique: Math.round([...stats.all.values()].reduce((x, y) => x + y, 0) / 1e5) / 10,
  rssMB: Math.round(mem.rss / 1e6), heapMB: Math.round(mem.heapUsed / 1e6) }, null, 1));
function median(v) { const s = [...v].sort((x, y) => x - y); return Math.round(s[Math.floor(s.length / 2)] * 10) / 10; }
