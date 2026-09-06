// Charged Up (2023) dashboard logic. Round-trips the community grid with ChargedUpObserver.java
// over NT4:
//
//   grid_top / grid_middle / grid_bottom   int   9-bit bitfields, bit c = column c
//   coopertition                           bool  human toggle
//   links                                  int   read-only, robot counts three-in-a-row
//
// Tap a node to toggle its bit; the whole row int is published to <table>/ToRobot. The robot
// unpacks it, recomputes links, and mirrors every value back on <table>/ToDashboard -- the page
// renders from that echo, not from the local tap. Keep the prefixes and topic names below
// identical to CONFIG and the DashboardChannel names in ChargedUpObserver.java.

import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/ChargedUp/ToRobot/";
const toDashboardPrefix = "/ChargedUp/ToDashboard/";

const ROW_TOPIC = { top: "grid_top", middle: "grid_middle", bottom: "grid_bottom" };
const CUBE_COLS = new Set([1, 4, 7]);
const COOP_COLS = new Set([3, 4, 5]); // center section -- the coopertition zone

const nodeType = (row, col) =>
  row === "bottom" ? "hybrid" : CUBE_COLS.has(col) ? "cube" : "cone";

// Last value the robot reported, keyed by bare topic name.
const state = { grid_top: 0, grid_middle: 0, grid_bottom: 0, coopertition: false, links: 0 };

const ntClient = new NT4_Client(
  window.location.hostname,
  "ChargedUpObserver",
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

// ***** GRID GENERATION *****

function buildGrid() {
  document.querySelectorAll(".grid-row").forEach((rowEl) => {
    const row = rowEl.dataset.row;
    const topic = ROW_TOPIC[row];
    for (let col = 0; col < 9; col++) {
      const node = document.createElement("div");
      node.className = "node " + nodeType(row, col);
      if (COOP_COLS.has(col)) node.classList.add("coop-zone");
      node.dataset.topic = topic;
      node.dataset.bit = col;
      bind(node, () =>
        ntClient.addSample(toRobotPrefix + topic, state[topic] ^ (1 << col))
      );
      rowEl.appendChild(node);
    }
  });

  bind(document.querySelector(".coop"), () =>
    ntClient.addSample(toRobotPrefix + "coopertition", !state.coopertition)
  );
}

// ***** RENDER *****

function popcount(n) {
  let c = 0;
  for (let i = 0; i < 32; i++) if ((n & (1 << i)) !== 0) c++;
  return c;
}

function render() {
  document.querySelectorAll(".node").forEach((el) => {
    const on = (state[el.dataset.topic] & (1 << Number(el.dataset.bit))) !== 0;
    el.classList.toggle("on", on);
  });
  document.querySelector(".coop").classList.toggle("active", !!state.coopertition);

  const b = popcount(state.grid_bottom);
  const m = popcount(state.grid_middle);
  const t = popcount(state.grid_top);
  // links comes from the subscribed OUTPUT_ONLY topic -- not recomputed here.
  document.querySelector(".links").innerText = state.links + " links";
  // teleop node values 2 / 3 / 5 by row, +5 per link -- an approximation, same as the Java side.
  document.querySelector(".points").innerText =
    2 * b + 3 * m + 5 * t + 5 * state.links + " pts";
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
  buildGrid();
  render();

  ntClient.subscribe(
    ["grid_top", "grid_middle", "grid_bottom", "coopertition", "links"].map(
      (t) => toDashboardPrefix + t
    ),
    false,
    false,
    0.02
  );

  // Publish only the bidirectional channels back to the robot; links is OUTPUT_ONLY.
  ["grid_top", "grid_middle", "grid_bottom"].forEach((t) =>
    ntClient.publishTopic(toRobotPrefix + t, "int")
  );
  ntClient.publishTopic(toRobotPrefix + "coopertition", "boolean");

  ntClient.connect();
});
