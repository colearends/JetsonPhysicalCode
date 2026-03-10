const ui = {
  startLbl: document.getElementById('startLbl'),
  endLbl: document.getElementById('endLbl'),
  distLbl: document.getElementById('distLbl'),
  gpsBtn: document.getElementById('gpsBtn'),
  resetBtn: document.getElementById('resetBtn'),
  status: document.getElementById('status'),

  // New UI elements (concept-style panel)
  distanceText: document.getElementById('distanceText'),
  etaText: document.getElementById('etaText'),
  arrivalText: document.getElementById('arrivalText'),
  destinationSelect: document.getElementById('destinationSelect'),
  goBtn: document.getElementById('goBtn'),
  exitBtn: document.getElementById('exitBtn'),
};

// ---------------- Arduino Serial Communication ----------------

let arduinoPort = null;
let arduinoWriter = null;

async function connectArduino() {
  try {
    arduinoPort = await navigator.serial.requestPort();
    await arduinoPort.open({ baudRate: 115200 });   // match Arduino
    arduinoWriter = arduinoPort.writable.getWriter();
    console.log("Arduino connected!");
  } catch (err) {
    console.error("Arduino connection failed:", err);
  }
}

document.getElementById("connectArduino").addEventListener("click", connectArduino);

async function sendToArduino(obj) {
  if (!arduinoWriter) {
    console.warn("Arduino not connected.");
    return;
  }

  const text = JSON.stringify(obj) + "\n";
  await arduinoWriter.write(new TextEncoder().encode(text));
  console.log("Sent to Arduino:", text);
}


const state = {
  nodes: [],
  edges: [],
  adj: new Map(),
  degree: new Map(),
  nodesById: new Map(),
  config: null,
  map: null,
  routeLine: null,
  destMarker: null,
  gpsMarker: null,
  gpsSnapMarker: null,
  startNodeId: null,
  endNodeId: null,
  watchId: null,

  routeActive: false,

  editableGroup: null,
  drawControl: null,
  editingEnabled: false,

  // Connected-component metadata for the routing graph.
  // This prevents snapping to small disconnected "islands" that have degree>0 but cannot reach most destinations.
  compIdByNode: new Map(),
  compSizeById: new Map(),
  mainCompId: null,
};

function haversine(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = (d) => d * Math.PI / 180;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(lat1)) *
      Math.cos(toRad(lat2)) *
      Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

function buildAdjFromEdges(edges) {
  const adj = new Map();
  for (const e of edges) {
    if (!adj.has(e.from)) adj.set(e.from, []);
    adj.get(e.from).push({ to: e.to, w: e.weight });
  }
  return adj;
}

class MinHeap {
  constructor() {
    this.a = [];
  }
  push(item) {
    this.a.push(item);
    this._up(this.a.length - 1);
  }
  pop() {
    if (this.a.length === 0) return null;
    const top = this.a[0];
    const last = this.a.pop();
    if (this.a.length) {
      this.a[0] = last;
      this._down(0);
    }
    return top;
  }
  _up(i) {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.a[p].k <= this.a[i].k) break;
      [this.a[p], this.a[i]] = [this.a[i], this.a[p]];
      i = p;
    }
  }
  _down(i) {
    const n = this.a.length;
    while (true) {
      let l = i * 2 + 1,
        r = l + 1,
        s = i;
      if (l < n && this.a[l].k < this.a[s].k) s = l;
      if (r < n && this.a[r].k < this.a[s].k) s = r;
      if (s === i) break;
      [this.a[s], this.a[i]] = [this.a[i], this.a[s]];
      i = s;
    }
  }
  get size() {
    return this.a.length;
  }
}

function nodeById(id) {
  return state.nodesById.get(id);
}

function formatMeters(m) {
  if (!Number.isFinite(m)) return '—';
  if (m < 1000) return `${m.toFixed(0)} m`;
  return `${(m / 1000).toFixed(2)} km`;
}

function nearestNode(lat, lon) {
  let best = null,
    bestD = Infinity;
  for (const n of state.nodes) {
    const d = haversine(lat, lon, n.lat, n.lon);
    if (d < bestD) {
      bestD = d;
      best = n;
    }
  }
  return { node: best, distM: bestD };
}

function nearestRoutableNode(lat, lon) {
  // Prefer nodes that are:
  //   1) routable (degree > 0), and
  //   2) in the *largest* connected component.
  //
  // Your current dataset has multiple components; snapping into a smaller component will produce
  // "graph disconnected" even though degree>0.
  let bestMain = null,
    bestMainD = Infinity;
  let bestAny = null,
    bestAnyD = Infinity;
  let fallback = null,
    fallbackD = Infinity;

  for (const n of state.nodes) {
    const d = haversine(lat, lon, n.lat, n.lon);
    if (d < fallbackD) {
      fallbackD = d;
      fallback = n;
    }

    const deg = state.degree.get(n.id) || 0;
    if (deg <= 0) continue;

    if (d < bestAnyD) {
      bestAnyD = d;
      bestAny = n;
    }

    const cid = state.compIdByNode.get(n.id);
    if (state.mainCompId !== null && cid !== state.mainCompId) continue;
    if (d < bestMainD) {
      bestMainD = d;
      bestMain = n;
    }
  }

  const pick = bestMain || bestAny || fallback;
  const usedFallback = !bestMain;
  const distM = bestMain ? bestMainD : bestAny ? bestAnyD : fallbackD;
  const degree = state.degree.get(pick?.id) || 0;
  const compId = state.compIdByNode.get(pick?.id);
  const compSize =
    compId !== undefined ? state.compSizeById.get(compId) || 0 : 0;
  return { node: pick, distM, degree, usedFallback, compId, compSize };
}

function computeComponents() {
  // Undirected reachability works here because your edges.json contains both directions.
  const seen = new Set();
  state.compIdByNode = new Map();
  state.compSizeById = new Map();
  state.mainCompId = null;

  let compCounter = 0;
  for (const n of state.nodes) {
    const start = n.id;
    if (seen.has(start)) continue;
    // BFS
    const q = [start];
    seen.add(start);
    const members = [];
    while (q.length) {
      const u = q.shift();
      members.push(u);
      for (const { to } of state.adj.get(u) || []) {
        if (!seen.has(to)) {
          seen.add(to);
          q.push(to);
        }
      }
    }
    const cid = String(++compCounter);
    for (const id of members) state.compIdByNode.set(id, cid);
    state.compSizeById.set(cid, members.length);
  }

  // Find the largest component id
  let bestId = null;
  let bestSize = -1;
  for (const [cid, size] of state.compSizeById.entries()) {
    if (size > bestSize) {
      bestSize = size;
      bestId = cid;
    }
  }
  state.mainCompId = bestId;
  console.log(
    'Graph components:',
    Object.fromEntries(state.compSizeById),
    'main=',
    state.mainCompId
  );
}

function aStar(startId, goalId) {
  const goal = nodeById(goalId);
  const start = nodeById(startId);
  if (!start || !goal) return { path: null, distance: Infinity };

  const open = new MinHeap();
  const g = new Map();
  const prev = new Map();
  g.set(startId, 0);
  open.push({ k: 0, id: startId });
  const closed = new Set();

  while (open.size) {
    const cur = open.pop();
    const u = cur.id;
    if (closed.has(u)) continue;
    if (u === goalId) break;
    closed.add(u);

    for (const { to, w } of state.adj.get(u) || []) {
      const ng = (g.get(u) ?? Infinity) + w;
      if (ng < (g.get(to) ?? Infinity)) {
        g.set(to, ng);
        prev.set(to, u);
        const vNode = nodeById(to);
        const h = haversine(vNode.lat, vNode.lon, goal.lat, goal.lon);
        open.push({ k: ng + h, id: to });
      }
    }
  }

  const total = g.get(goalId);
  if (!Number.isFinite(total)) return { path: null, distance: Infinity };

  const path = [];
  let cur = goalId;
  while (cur) {
    path.push(cur);
    if (cur === startId) break;
    cur = prev.get(cur);
  }
  path.reverse();
  return { path, distance: total };
}

function clearRoute() {
  if (state.routeLine) {
    state.map.removeLayer(state.routeLine);
    state.routeLine = null;
  }
  ui.distLbl.textContent = '—';

  // Also clear the concept-panel fields when route is cleared
  if (ui.distanceText) ui.distanceText.textContent = '-';
  if (ui.etaText) ui.etaText.textContent = '-';
  if (ui.arrivalText) ui.arrivalText.textContent = '-';
}

function drawRoute(pathIds) {
  clearRoute();
  const latlngs = pathIds.map((id) => {
    const n = nodeById(id);
    return [n.lat, n.lon];
  });
  state.routeLine = L.polyline(latlngs, {
    weight: 8,
    opacity: 0.75,
    color: '#000000',
    dashArray: '10 10',
    lineCap: 'round',
    lineJoin: 'round',
  }).addTo(state.map);
}
function updateLabels() {
  ui.startLbl.textContent = state.startNodeId
    ? `Node ${state.startNodeId}`
    : '—';
  ui.endLbl.textContent = state.endNodeId ? `Node ${state.endNodeId}` : '—';
}

function isReachable(startId, goalId) {
  const q = [startId];
  const seen = new Set([startId]);

  while (q.length) {
    const u = q.shift();
    if (u === goalId) return true;

    for (const { to } of state.adj.get(u) || []) {
      if (!seen.has(to)) {
        seen.add(to);
        q.push(to);
      }
    }
  }
  return false;
}

function recomputeRoute() {
  if (!state.startNodeId || !state.endNodeId) {
    ui.status.textContent = 'Waiting for start and destination...';
    return;
  }

  clearRoute();
  if (!state.startNodeId || !state.endNodeId) {
    ui.status.textContent = 'Waiting for start and destination...';
    return;
  }
  if (!isReachable(state.startNodeId, state.endNodeId)) {
    const sc = state.compIdByNode.get(state.startNodeId);
    const ec = state.compIdByNode.get(state.endNodeId);
    const ss = sc ? state.compSizeById.get(sc) || 0 : 0;
    const es = ec ? state.compSizeById.get(ec) || 0 : 0;
    ui.status.textContent = `No route: network disconnected. startComp=${sc} (${ss}), endComp=${ec} (${es}), mainComp=${state.mainCompId}`;
    return;
  }
  console.log('Routing from', state.startNodeId, 'to', state.endNodeId);

  ui.status.textContent = 'Routing…';
  setTimeout(() => {
    const res = aStar(state.startNodeId, state.endNodeId);
    if (!res.path) {
      ui.status.textContent = `No route (graph disconnected). start=${state.startNodeId} deg=${
        state.degree.get(state.startNodeId) || 0
      }, end=${state.endNodeId} deg=${
        state.degree.get(state.endNodeId) || 0
      }`;
      return;
    }
    drawRoute(res.path);

    // Old hidden label (meters)
    ui.distLbl.textContent = formatMeters(res.distance);

    // New concept-panel fields
    const meters = res.distance;

    // Distance in miles
    if (ui.distanceText) {
      const miles = meters / 1609.34;
      ui.distanceText.textContent = miles.toFixed(2) + ' miles';
    }

    // ETA in minutes (assume ~15 mph => ~6.7 m/s)
    const shuttleSpeed = 0.9; // m/s
    const seconds = meters / shuttleSpeed;
    const minutes = Math.round(seconds / 60);
    if (ui.etaText) {
      ui.etaText.textContent = minutes + ' min';
    }

    // Arrival time (clock)
    if (ui.arrivalText) {
      const now = new Date();
      now.setMinutes(now.getMinutes() + minutes);
      ui.arrivalText.textContent = now.toLocaleTimeString([], {
        hour: '2-digit',
        minute: '2-digit',
      });
    }

    ui.status.textContent = 'Ready';
  }, 10);
}

function setDestination(lat, lon) {
  const { node, distM, degree, usedFallback } = nearestRoutableNode(lat, lon);
  if (usedFallback)
    console.warn('Destination snapped to isolated node; routing may fail.');
  if (!node) return;
  state.endNodeId = node.id;
  updateLabels();

  if (state.destMarker) state.map.removeLayer(state.destMarker);
  state.destMarker = L.circleMarker([node.lat, node.lon], {
    radius: 7,
    weight: 2,
    fillOpacity: 0.9,
    color: '#0033A0',
    fillColor: '#0033A0',
  })
    .addTo(state.map)
    .bindTooltip('Destination', { permanent: false });

  const cid = state.compIdByNode.get(node.id);
  const csz = cid ? state.compSizeById.get(cid) || 0 : 0;
  const main =
    cid && state.mainCompId && cid === state.mainCompId ? 'main' : 'non-main';
  ui.status.textContent = `Destination snapped (${distM.toFixed(
    1
  )} m), deg=${degree}, comp=${cid} (${csz}, ${main})`;
  recomputeRoute();
}

function setStartFromGPS(lat, lon, acc) {
  const { node, distM, degree, usedFallback } = nearestRoutableNode(lat, lon);
  if (usedFallback)
    console.warn('Start snapped to isolated node; routing may fail.');
  if (!node) return;

  console.log('GPS start node:', node.id);

  state.startNodeId = node.id;
  updateLabels();

  // Raw GPS position (yellow)
  if (!state.gpsMarker) {
    state.gpsMarker = L.circleMarker([lat, lon], {
      radius: 7,
      weight: 2,
      fillOpacity: 0.2,
      color: '#FFD100',
      fillColor: '#FFD100',
    })
      .addTo(state.map)
      .bindTooltip('GPS', { permanent: false });
  } else {
    state.gpsMarker.setLatLng([lat, lon]);
  }

  // Snapped start node (orange outline)
  if (!state.gpsSnapMarker) {
    state.gpsSnapMarker = L.circleMarker([node.lat, node.lon], {
      radius: 8,
      weight: 3,
      fillOpacity: 0.9,
      color: '#FFD100',
    })
      .addTo(state.map)
      .bindTooltip('Start (snapped)', { permanent: false });
  } else {
    state.gpsSnapMarker.setLatLng([node.lat, node.lon]);
  }

  const cid = state.compIdByNode.get(node.id);
  const csz = cid ? state.compSizeById.get(cid) || 0 : 0;
  const main =
    cid && state.mainCompId && cid === state.mainCompId ? 'main' : 'non-main';
  ui.status.textContent = `GPS ok (±${acc.toFixed(
    0
  )} m), snapped ${distM.toFixed(1)} m, deg=${degree}, comp=${cid} (${csz}, ${main})`;
  recomputeRoute();
}

function startGPS() {
  if (!navigator.geolocation) {
    ui.status.textContent = 'Geolocation not supported.';
    return;
  }
  if (state.watchId !== null) return;

  ui.status.textContent = 'Requesting GPS permission…';
  state.watchId = navigator.geolocation.watchPosition(
    (pos) => {
      const { latitude, longitude, accuracy } = pos.coords;
      setStartFromGPS(latitude, longitude, accuracy || 0);
    },
    (err) => {
      ui.status.textContent = `GPS error: ${err.message}`;
      state.watchId = null;
    },
    { enableHighAccuracy: true, maximumAge: 1000, timeout: 10000 }
  );
}

function resetAll() {
  state.startNodeId = null;
  state.endNodeId = null;
  updateLabels();
  clearRoute();
  if (state.destMarker) {
    state.map.removeLayer(state.destMarker);
    state.destMarker = null;
  }
  ui.status.textContent = 'Ready';
}

function downloadJson(obj, filename) {
  const blob = new Blob([JSON.stringify(obj, null, 2)], {
    type: 'application/json',
  });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function rebuildGraphFromWays(waysFC) {
  const nodes = [];
  const nodesByKey = new Map();
  const edges = [];

  function getNodeId(lat, lon) {
    const key = `${lat.toFixed(5)},${lon.toFixed(5)}`;
    if (nodesByKey.has(key)) return nodesByKey.get(key);
    const id = String(nodes.length + 1);
    nodesByKey.set(key, id);
    nodes.push({ id, lat, lon });
    return id;
  }

  for (const feat of waysFC.features || []) {
    if (!feat.geometry || feat.geometry.type !== 'LineString') continue;
    const coords = feat.geometry.coordinates;
    if (!coords || coords.length < 2) continue;
    for (let i = 0; i < coords.length - 1; i++) {
      const [lon1, lat1] = coords[i];
      const [lon2, lat2] = coords[i + 1];
      const a = getNodeId(lat1, lon1);
      const b = getNodeId(lat2, lon2);
      const w = haversine(lat1, lon1, lat2, lon2);
      edges.push({ from: a, to: b, weight: w });
      edges.push({ from: b, to: a, weight: w });
    }
  }

  const seen = new Set();
  const dedup = [];
  for (const e of edges) {
    const k = `${e.from}->${e.to}`;
    if (seen.has(k)) continue;
    seen.add(k);
    dedup.push(e);
  }
  return { nodes, edges: dedup };
}

async function loadData() {
  const [config, nodes, edges, ways] = await Promise.all([
    fetch('./data/config.json').then((r) => r.json()),
    fetch('./data/nodes.json').then((r) => r.json()),
    fetch('./data/edges.json').then((r) => r.json()),
    fetch('./data/ways.geojson').then((r) => r.json()),
  ]);
  state.config = config;
  state.nodes = nodes;
  state.edges = edges;
  state.nodesById = new Map();
  for (const n of nodes) state.nodesById.set(n.id, n);
  state.adj = buildAdjFromEdges(edges);
  // Degree (treat as undirected by counting both endpoints)
  state.degree = new Map();
  for (const e of edges) {
    state.degree.set(e.from, (state.degree.get(e.from) || 0) + 1);
    state.degree.set(e.to, (state.degree.get(e.to) || 0) + 1);
  }
  computeComponents();
  return ways;
}

// Populate destination dropdown dynamically from nodes (top 50 by degree)
function populateDestinationDropdown() {
  if (!ui.destinationSelect) return;

  const select = ui.destinationSelect;
  select.innerHTML = '<option value="">Select destination…</option>';

  // ⭐ Only one destination for now
  const named = [
    { id: "298", label: "SDSU Dairy Bar" },
    { id: "118", label: "Union West Entrance" },
    { id: "750", label: "Union East Entrance" },
    { id: "1043", label: "Dana Dykehouse Stadium" },
    { id: "114", label: "Miller Wellness Center" },
    { id: "293", label: "Raven Ag Center" },
    { id: "103", label: "Jackrabbit Village" },
    { id: "662", label: "Sanford Complex" },
    { id: "684", label: "Oscar Larson Performing Arts Center" },
    { id: "394", label: "Larson Dining Hall" },
    { id: "887", label: "Daktronics Engineering Hall" },
    { id: "527", label: "Crothers Engineering Hall" },
    { id: "842", label: "Hilton M. Briggs Library" },
    { id: "418", label: "Frost Arena" }
  ];

  for (const d of named) {
    const opt = document.createElement('option');
    opt.value = d.id;
    opt.textContent = d.label;
    select.appendChild(opt);
  }
}


(async function main() {
  const ways = await loadData();
  const b = state.config.bounds;
  const bounds = L.latLngBounds(
    [b.south, b.west],
    [b.north, b.east]
  );

  state.map = L.map('map', {
    zoomControl: true,
    minZoom: 16.4999999,
    maxZoom: 18,
    maxBounds: bounds,
    maxBoundsViscosity: 0.75,
  });
  window.leafletMap = state.map;
  state.map.fitBounds(bounds, {
  padding: [0, 0],
  animate: false
  });

  const overlay = L.imageOverlay('./assets/sdsu_map.png', bounds, {
  opacity: 0.95,
  interactive: false
  }).addTo(state.map);

  // Lock the map to the image bounds
  state.map.setMaxBounds(bounds);
  // Force Leaflet to recalc size after layout loads
  setTimeout(() => {
  state.map.invalidateSize();
  state.map.panBy([0, 150], { animate: false });
  }, 200);


  state.editableGroup = new L.FeatureGroup();
  state.map.addLayer(state.editableGroup);

  const waysLayer = L.geoJSON(ways, {
    style: {
      weight: 5,
      opacity: 0.25,
      color: '#ffffff',
      lineCap: 'round',
      lineJoin: 'round',
    },
    onEachFeature: (feature) => {
      if (!feature.properties) feature.properties = {};
      if (!feature.properties._sid) {
        if (typeof crypto !== 'undefined' && crypto.randomUUID)
          feature.properties._sid = crypto.randomUUID();
        else feature.properties._sid = String(Math.random()).slice(2);
      }
    },
  });
  waysLayer.eachLayer((l) => state.editableGroup.addLayer(l));

  ui.status.textContent = `Loaded: ${state.config.stats.ways} ways, ${state.config.stats.nodes_used} nodes`;
  populateDestinationDropdown();

if (ui.destinationSelect) {
  ui.destinationSelect.addEventListener('change', (ev) => {
    const id = ev.target.value;
    if (!id) return;

    const node = nodeById(id);
    if (!node) {
      console.warn("No node found for id", id);
      return;
    }

    // Snap directly to the node's coordinates
    setDestination(node.lat, node.lon);
  });
}



  // Map click sets destination
  state.map.on('click', (ev) => {
    if (state.editingEnabled) return;
    setDestination(ev.latlng.lat, ev.latlng.lng);
  });

  // Buttons
if (ui.gpsBtn) {
  ui.gpsBtn.addEventListener('click', startGPS);
}

if (ui.resetBtn) {
  ui.resetBtn.addEventListener('click', resetAll);
}

if (ui.goBtn) {
  ui.goBtn.addEventListener('click', async () => {
    recomputeRoute();
    state.routeActive = true;

    setTimeout(async () => {
      if (state.routeLine) {
        const coords = state.routeLine.getLatLngs();

        const waypointList = coords.map(c => ({
          lat: c.lat,
          lon: c.lng
        }));

        console.log("WAYPOINT LIST:", waypointList);

        await sendToArduino({
          routeActive: true,
          waypoints: waypointList
        });

        downloadJson(waypointList, "route_waypoints.json");
      }
    }, 50);
  });
}




if (ui.exitBtn) {
  ui.exitBtn.addEventListener('click', async () => {
    state.routeActive = false;

    await sendToArduino({
      routeActive: false
    });

    if (state.routeLine) {
      state.map.removeLayer(state.routeLine);
      state.routeLine = null;
    }

    if (state.destMarker) {
      state.map.removeLayer(state.destMarker);
      state.destMarker = null;
    }

    if (ui.destinationSelect) {
      ui.destinationSelect.value = '';
    }

    if (ui.distanceText) ui.distanceText.textContent = '-';
    if (ui.etaText) ui.etaText.textContent = '-';
    if (ui.arrivalText) ui.arrivalText.textContent = '-';

    state.endNodeId = null;
    updateLabels();

    ui.status.textContent = 'Route and destination cleared';
  });
}

document.getElementById("dpad-up").addEventListener("click", () => console.log("UP pressed"));
document.getElementById("dpad-down").addEventListener("click", () => console.log("DOWN pressed"));
document.getElementById("dpad-left").addEventListener("click", () => console.log("LEFT pressed"));
document.getElementById("dpad-right").addEventListener("click", () => console.log("RIGHT pressed"));
document.getElementById("dpad-center").addEventListener("click", () => console.log("CENTER pressed"));



})();
