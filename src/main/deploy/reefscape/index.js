// Reefscape (2025) dashboard logic. Round-trips the reef with ReefscapeObserver.java over NT4:
//
//   l2 / l3 / l4   int   12-bit branch bitfields (bit i = branch i, clockwise from the top)
//   l1             int   trough count (scalar, not a bitfield)
//   algae          int   6-bit bitfield
//   coop / rp_focus bool  human toggles
//   is_elims / rp_complete  bool  READ-ONLY -- the robot derives these and may override coop/rp_focus
//
// Tap a branch -> publish the whole selected-level int to <table>/ToRobot; the robot mirrors every
// value back on <table>/ToDashboard and the page renders from that echo. Keep the prefixes and
// topic names below identical to CONFIG and the DashboardChannel names in ReefscapeObserver.java.

import { NT4_Client } from "./NT4.js";

// ***** NETWORKTABLES *****

const toRobotPrefix = "/Reefscape/ToRobot/";
const toDashboardPrefix = "/Reefscape/ToDashboard/";

const CORAL_LEVELS = ["l4", "l3", "l2"]; // selectable; L1 is the separate trough counter
const state = {
  l1: 0,
  l2: 0,
  l3: 0,
  l4: 0,
  algae: 0,
  coop: false,
  rp_focus: false,
  is_elims: false,
  rp_complete: false,
};

let selectedLevel = "l4"; // local UI state -- which coral level the branches show/toggle

const ntClient = new NT4_Client(
  window.location.hostname,
  "ReefscapeObserver",
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

// ***** REEF GENERATION *****
// 12 branches evenly around a circle (bit 0 at the top, clockwise); 6 algae on an inner circle,
// offset to sit between branch pairs.

function place(el, angleDeg, radiusPct) {
  const a = (angleDeg - 90) * (Math.PI / 180);
  el.style.left = 50 + radiusPct * Math.cos(a) + "%";
  el.style.top = 50 + radiusPct * Math.sin(a) + "%";
}

function buildReef() {
  const reef = document.querySelector(".reef");

  for (let i = 0; i < 12; i++) {
    const branch = document.createElement("div");
    branch.className = "branch";
    branch.dataset.idx = i;
    place(branch, i * 30, 40);
    bind(branch, () =>
      ntClient.addSample(
        toRobotPrefix + selectedLevel,
        state[selectedLevel] ^ (1 << i)
      )
    );
    reef.appendChild(branch);
  }

  for (let i = 0; i < 6; i++) {
    const algae = document.createElement("div");
    algae.className = "algae";
    algae.dataset.idx = i;
    place(algae, 15 + i * 60, 19);
    bind(algae, () =>
      ntClient.addSample(toRobotPrefix + "algae", state.algae ^ (1 << i))
    );
    reef.appendChild(algae);
  }

  document.querySelectorAll(".level").forEach((btn) => {
    bind(btn, () => {
      selectedLevel = btn.dataset.level;
      render();
    });
  });

  bind(document.querySelector(".l1-minus"), () =>
    ntClient.addSample(toRobotPrefix + "l1", Math.max(0, state.l1 - 1))
  );
  bind(document.querySelector(".l1-plus"), () =>
    ntClient.addSample(toRobotPrefix + "l1", state.l1 + 1)
  );
  bind(document.querySelector(".coop"), () =>
    ntClient.addSample(toRobotPrefix + "coop", !state.coop)
  );
  bind(document.querySelector(".rp-focus"), () =>
    ntClient.addSample(toRobotPrefix + "rp_focus", !state.rp_focus)
  );
}

// ***** RENDER *****

function popcount(n) {
  let c = 0;
  for (let i = 0; i < 32; i++) if ((n & (1 << i)) !== 0) c++;
  return c;
}

function render() {
  document.querySelectorAll(".level").forEach((btn) =>
    btn.classList.toggle("active", btn.dataset.level === selectedLevel)
  );

  const level = state[selectedLevel];
  document.querySelectorAll(".branch").forEach((el) => {
    el.classList.toggle("on", (level & (1 << Number(el.dataset.idx))) !== 0);
  });
  document.querySelectorAll(".algae").forEach((el) => {
    el.classList.toggle("on", (state.algae & (1 << Number(el.dataset.idx))) !== 0);
  });

  document.querySelector(".l1-value").innerText = state.l1;
  document.querySelector(".coop").classList.toggle("active", !!state.coop);
  document.querySelector(".rp-focus").classList.toggle("active", !!state.rp_focus);

  const coral =
    state.l1 + popcount(state.l2) + popcount(state.l3) + popcount(state.l4);
  document.querySelector(".coral-count").innerText =
    coral + " coral · " + popcount(state.algae) + " algae";

  const rp = document.querySelector(".rp");
  rp.innerText = state.rp_complete ? "RP ✓" : "RP —";
  rp.classList.toggle("met", !!state.rp_complete);

  document.querySelector(".elims").hidden = !state.is_elims;
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
  buildReef();
  render();

  ntClient.subscribe(
    Object.keys(state).map((t) => toDashboardPrefix + t),
    false,
    false,
    0.02
  );

  // Publish only the bidirectional channels; is_elims / rp_complete are OUTPUT_ONLY.
  ["l1", "l2", "l3", "l4", "algae"].forEach((t) =>
    ntClient.publishTopic(toRobotPrefix + t, "int")
  );
  ["coop", "rp_focus"].forEach((t) =>
    ntClient.publishTopic(toRobotPrefix + t, "boolean")
  );

  ntClient.connect();
});
