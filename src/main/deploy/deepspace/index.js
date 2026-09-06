// Deep Space (2019) dashboard logic. Round-trips six bitfield channels with DeepSpaceObserver.java
// over NT4:
//
//   left_rocket_hatch  / left_rocket_cargo   int  6-bit bitfields (3 levels x 2 bays)
//   right_rocket_hatch / right_rocket_cargo  int  6-bit bitfields
//   cargo_ship_hatch   / cargo_ship_cargo    int  8-bit bitfields
//
// Each bay is a donut: the ring toggles bit N of the *_hatch channel, the center dot toggles bit
// N of the matching *_cargo channel. The robot writes every channel to <table>/ToDashboard and
// reads them back from <table>/ToRobot. Keep the prefixes below and the data-hatch / data-cargo
// attributes in index.html identical to CONFIG and the DashboardChannel names in
// DeepSpaceObserver.java.

import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/DeepSpace/ToRobot/";
const toDashboardPrefix = "/DeepSpace/ToDashboard/";

const topics = []; // every bitfield channel name, filled in from the .structure elements on load
const state = {}; // last value the robot reported, keyed by bare topic name

const ntClient = new NT4_Client(
  window.location.hostname,
  "DeepSpaceObserver",
  () => {}, // topic announce
  () => {}, // topic unannounce
  (topic, _, value) => {
    if (!topic.name.startsWith(toDashboardPrefix)) return;
    const name = topic.name.slice(toDashboardPrefix.length);
    if (!(name in state)) return;
    state[name] = value;
    render();
  },
  () => document.body.classList.add("connected"),
  () => document.body.classList.remove("connected")
);

// ***** BAY GENERATION *****
// Each .structure becomes a vertical stack of rows. Rockets carry data-levels (labelled L3..L1,
// top down); the cargo ship carries data-rows (unlabelled). Every row holds data-cols bays, and
// each bay is a hatch ring (.hatch) wrapping a cargo dot (.cargo).

function makeLevelLabel(text) {
  const label = document.createElement("div");
  label.className = "level-label";
  label.innerText = text;
  return label;
}

function makeBay(hatchTopic, cargoTopic, bit) {
  const bay = document.createElement("div");
  bay.className = "bay";

  const ring = document.createElement("div");
  ring.className = "hatch";
  ring.dataset.topic = hatchTopic;
  ring.dataset.bit = bit;
  bind(ring, () =>
    ntClient.addSample(toRobotPrefix + hatchTopic, state[hatchTopic] ^ (1 << bit))
  );

  const dot = document.createElement("div");
  dot.className = "cargo";
  dot.dataset.topic = cargoTopic;
  dot.dataset.bit = bit;
  bind(dot, (event) => {
    event.stopPropagation(); // don't also toggle the hatch ring we're nested in
    ntClient.addSample(toRobotPrefix + cargoTopic, state[cargoTopic] ^ (1 << bit));
  });

  ring.appendChild(dot);
  bay.appendChild(ring);
  return bay;
}

function buildBays() {
  document.querySelectorAll(".structure").forEach((structure) => {
    const hatchTopic = structure.dataset.hatch;
    const cargoTopic = structure.dataset.cargo;
    const cols = Number(structure.dataset.cols);
    const levels = Number(structure.dataset.levels || 0);
    const rows = levels || Number(structure.dataset.rows);
    // e.g. "NEAR,FAR" -- one label per column, left to right. Optional.
    const colNames = structure.dataset.columns
      ? structure.dataset.columns.split(",")
      : null;

    topics.push(hatchTopic, cargoTopic);
    state[hatchTopic] = 0;
    state[cargoTopic] = 0;

    const container = structure.querySelector(".bays");

    if (colNames) {
      const head = document.createElement("div");
      head.className = "col-headers";
      if (levels) head.appendChild(makeLevelLabel("")); // gutter over the L3/L2/L1 column
      colNames.forEach((name) => {
        const h = document.createElement("div");
        h.className = "col-header";
        h.innerText = name;
        head.appendChild(h);
      });
      container.appendChild(head);
    }

    let bit = 0;
    for (let r = 0; r < rows; r++) {
      const row = document.createElement("div");
      row.className = "bay-row";
      if (levels) row.appendChild(makeLevelLabel("L" + (levels - r))); // top row = highest level
      for (let c = 0; c < cols; c++) {
        row.appendChild(makeBay(hatchTopic, cargoTopic, bit++));
      }
      container.appendChild(row);
    }
  });
}

// ***** RENDER *****

function popcount(n) {
  let c = 0;
  for (let i = 0; i < 32; i++) if ((n & (1 << i)) !== 0) c++;
  return c;
}

function render() {
  document.querySelectorAll(".hatch, .cargo").forEach((el) => {
    const on = (state[el.dataset.topic] & (1 << Number(el.dataset.bit))) !== 0;
    el.classList.toggle("on", on);
  });
  const total = topics.reduce((sum, t) => sum + popcount(state[t]), 0);
  document.querySelector(".points").innerText = total + " pts";
}

// ***** BUTTON BINDINGS *****
// Same touch/mouse guard as mainbot-2025's reefcontrols: a touchstart marks the session as touch
// so the synthetic mousedown that follows does not double-fire.

let isTouch = false;

function bind(element, callback) {
  const activate = (fromTouch, event) => {
    if (fromTouch) isTouch = true;
    if (isTouch === fromTouch) callback(event);
  };
  element.addEventListener("touchstart", (e) => activate(true, e));
  element.addEventListener("mousedown", (e) => activate(false, e));
  element.addEventListener("contextmenu", (e) => {
    e.preventDefault();
    activate(false, e);
  });
}

let lastMouseEvent = 0;
window.addEventListener("mousemove", () => {
  const now = Date.now();
  if (now - lastMouseEvent < 50) isTouch = false;
  lastMouseEvent = now;
});

// ***** STARTUP *****

window.addEventListener("load", () => {
  buildBays();
  render();

  ntClient.subscribe(
    topics.map((t) => toDashboardPrefix + t),
    false,
    false,
    0.02
  );
  topics.forEach((t) => ntClient.publishTopic(toRobotPrefix + t, "int"));
  ntClient.connect();
});
