#pragma once

// The bring-up web page: brightness, the beaker's base dye colour, and a firmware upload -- the
// three things that used to need a physical BOOT-button jumper or a re-flash to change. Served
// from PROGMEM rather than a filesystem partition: it is small, and a filesystem is one more thing
// that can go stale relative to the firmware that reads it.
//
// The three controls post to /set (brightness, dye) and /update (OTA); both just turn their input
// into the exact console command line a human would have typed and hand it to
// App::submitCommand -- see main.cpp. No second copy of what a brightness or a dye value means.
static const char kWebUiHtml[] PROGMEM = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>partsim mini</title>
<style>
  body { font-family: system-ui, sans-serif; max-width: 420px; margin: 2em auto; padding: 0 1em;
         background: #111; color: #eee; }
  h1 { font-size: 1.2em; }
  label { display: block; margin-top: 1.2em; font-size: 0.9em; color: #aaa; }
  small { color: #777; }
  input[type=range] { width: 100%; }
  input[type=color] { width: 100%; height: 2.5em; border: none; background: none; }
  input[type=file] { width: 100%; }
  button { margin-top: 0.6em; padding: 0.5em 1em; }
  #status { margin-top: 1em; font-size: 0.9em; color: #8f8; min-height: 1.2em; }
  .row { display: flex; gap: 0.5em; align-items: center; }
  .row output { min-width: 3em; text-align: right; }
</style>
</head>
<body>
<h1>partsim mini</h1>

<label for="b">Brightness</label>
<div class="row">
  <input type="range" id="b" min="0" max="255" value="96" oninput="brightOut.value=b.value">
  <output id="brightOut">96</output>
</div>

<label for="dye">Beaker dye (base colour)</label>
<input type="color" id="dye" value="#8080ff">
<button onclick="setDye()">Set dye</button>
<div><small>A mix, not literal RGB -- pure white will read as a pale, desaturated grey.</small></div>

<label>Firmware update</label>
<form id="ota" method="POST" action="/update" enctype="multipart/form-data">
  <input type="file" name="firmware" accept=".bin">
  <button type="submit">Upload &amp; flash</button>
</form>

<div id="status"></div>

<script>
const status = document.getElementById('status');

document.getElementById('b').addEventListener('change', (e) => {
  fetch('/set?b=' + e.target.value).then(r => r.text()).then(t => status.textContent = t);
});

function setDye() {
  // Dye is a MIX, not independent RGB -- see the `d` console command: R and G are shares of a
  // 255 budget and blue is whatever is left. A colour picker has no such constraint, so its R/G/B
  // are normalised onto that budget (preserving the ratio between them) rather than sent through
  // unchanged, which silently dropped the picker's blue channel entirely and is what produced
  // "R255 B255 turns fully red" -- the console-side sum was 510, not 255.
  const hex = document.getElementById('dye').value;
  const r = parseInt(hex.substr(1, 2), 16);
  const g = parseInt(hex.substr(3, 2), 16);
  const b = parseInt(hex.substr(5, 2), 16);
  const total = r + g + b;
  const dr = total === 0 ? 0 : Math.round(r * 255 / total);
  const dg = total === 0 ? 0 : Math.round(g * 255 / total);
  fetch('/set?dye=' + dr + ',' + dg).then(r => r.text()).then(t => status.textContent = t);
}

document.getElementById('ota').addEventListener('submit', (e) => {
  e.preventDefault();
  status.textContent = 'uploading...';
  const data = new FormData(e.target);
  fetch('/update', { method: 'POST', body: data })
    .then(r => r.text())
    .then(t => { status.textContent = t; })
    .catch(() => { status.textContent = 'upload failed (board rebooting?)'; });
});
</script>
</body>
</html>
)HTML";
