const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync('config/miti_research_panel.js','utf8');
const {browserClockOffset}=vm.runInNewContext(source+';rtabmapResearchFactory({})');
const server=Date.parse('2026-11-22T12:00:00Z');
for(const skew of [-3600000,0,3600000]){
  const started=server+500+skew,received=started+40;
  const offset=browserClockOffset(new Date(server).toUTCString(),started,received);
  const serverNow=received/1000-offset;
  assert.ok(Math.abs(serverNow-(server+520)/1000)<1e-9);
  assert.ok(serverNow-(server+300)/1000<4,'fresh result should stay fresh on Windows');
  assert.ok(serverNow-(server-10000)/1000>4,'stopped producer must remain stale');
}
assert.equal(browserClockOffset(null,100,200),null);
assert.equal(browserClockOffset(new Date(server).toUTCString(),0,4000),null);
console.log('Browser/server clock conversion: 5 cases passed');
