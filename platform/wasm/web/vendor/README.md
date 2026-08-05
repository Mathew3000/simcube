# Vendored dependencies

Committed rather than installed via npm, so the repo stays dependency-free, builds offline, and
the web frontend needs no bundler — just a static file server and a browser import map.

| file | version | source | licence |
|---|---|---|---|
| `three.module.js` | r170 | `https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js` | MIT |
| `OrbitControls.js` | r170 | `https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/controls/OrbitControls.js` | MIT |

three.js is Copyright © 2010-2024 three.js authors, MIT licensed. Full text:
https://github.com/mrdoob/three.js/blob/dev/LICENSE

`OrbitControls.js` does `import ... from 'three'`, which the pages resolve with an import map:

```html
<script type="importmap">
{ "imports": { "three": "./vendor/three.module.js" } }
</script>
```

## Upgrading

Keep both files on the same revision — mixing an example module with a different core version
breaks in confusing ways.

```sh
V=0.171.0
curl -sL -o three.module.js  "https://cdn.jsdelivr.net/npm/three@$V/build/three.module.js"
curl -sL -o OrbitControls.js "https://cdn.jsdelivr.net/npm/three@$V/examples/jsm/controls/OrbitControls.js"
```

Then re-check `orient.html`: a three.js change to `DataTexture` defaults (`flipY`,
`unpackAlignment`) or to `makeBasis` handedness would silently flip or mirror panels.
