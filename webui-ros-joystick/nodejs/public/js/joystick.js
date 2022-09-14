const JOYSTICK_COLOR = "#808080";
const LED_GREEN_COLOR = "#508040";
const LED_RED_COLOR = "#b02222";

var beginJoyPosX = 0;
var beginJoyPosY = 0;
var manager;
var lin;
var ang;
var joystick_timeout;
var velocity_repeat_delay = 100; // [ms]
var timerInstance;
var resize_tout;
var socket;
var alert_container;

function removeJoystick() {
  joystickContainer = document.getElementById("joystick");
  while (joystickContainer.hasChildNodes()) {
    joystickContainer.removeChild(joystickContainer.childNodes[0]);
  }
  if (!jQuery.isEmptyObject(manager)) {
    manager.destroy();
  }
}

function repeat_velcmd(v_lin, v_ang) {
  moveAction(v_lin, v_ang);
  joystick_timeout = setTimeout(function () {
    repeat_velcmd(lin, ang);
  }, velocity_repeat_delay);
}

function createJoystick(posX, posY, size) {

  let joystickContainer = document.querySelector('.joystickContainer')
  joystickContainer.style.top = (posY - size / 2) + "px";
  joystickContainer.style.left = (posX - size / 2) + "px";

  let ledSize = size / 10;
  let joystickSize = size - ledSize;

  let joystick = document.getElementById("joystick");
  joystick.style.width = joystickSize / 2 + "px";
  joystick.style.height = joystickSize / 2 + "px";
  joystick.style.top = size / 4 + "px";
  joystick.style.left = size / 4 + "px";

  let outerCircle = document.getElementById('outerCircle')
  outerCircle.style.width = joystickSize + "px";
  outerCircle.style.height = joystickSize + "px";
  outerCircle.style.top = ledSize / 2 + "px";
  outerCircle.style.left = ledSize / 2 + "px";
  outerCircle.style.background = JOYSTICK_COLOR;

  let innerCircle = document.getElementById('innerCircle')
  innerCircle.style.width = joystickSize / 2 + "px";
  innerCircle.style.height = joystickSize / 2 + "px";
  innerCircle.style.top = size / 4 + ledSize / 4 + "px";
  innerCircle.style.left = size / 4 + ledSize / 4 + "px";
  innerCircle.style.background = JOYSTICK_COLOR;

  let ledCircle = document.getElementById("ledCircle");
  ledCircle.style.width = size + "px";
  ledCircle.style.height = size + "px";
  ledCircle.style.top = "0px";
  ledCircle.style.left = "0px";
  ledCircle.style.backgroundColor = LED_RED_COLOR;

  var options = {
    zone: joystick,
    color: JOYSTICK_COLOR,
    size: joystickSize,
    fadeTime: 40,
  };
  manager = nipplejs.create(options);
  manager.on("start", async function (evt, nipple) {
    beginJoyPosX = nipple.position.x;
    beginJoyPosY = nipple.position.y;
    lin = 0;
    ang = 0;

    await new Promise(r => setTimeout(r, 30));
    innerCircle.hidden = true;
    outerCircle.hidden = true;
    // move led
    let ledPos = ledCircle.getBoundingClientRect();
    let relativePosY = (nipple.position.y - size / 2) - ledPos.top;
    let relativePosX = (nipple.position.x - size / 2) - ledPos.left;
    ledCircle.style.top = relativePosY + "px";
    ledCircle.style.left = relativePosX + "px";
  })
  manager.on("move", function (evt, nipple) {
    relativePosX = beginJoyPosX - nipple.position.x;
    relativePosY = beginJoyPosY - nipple.position.y;

    ang = mapRange(relativePosX, - joystickSize / 2, joystickSize / 2, -1, 1);
    lin = mapRange(relativePosY, - joystickSize / 2, joystickSize / 2, -1.1, 1.1);

    if (lin > 1.0) lin = 1.0;
    if (lin < -1.0) lin = -1.0;

    clearTimeout(joystick_timeout);
    moveAction(lin, ang);
    joystick_timeout = setTimeout(function () {
      repeat_velcmd(lin, ang);
    }, velocity_repeat_delay);
  });
  manager.on("end", async function () {
    clearTimeout(joystick_timeout);
    beginJoyPosX = 0;
    beginJoyPosY = 0;
    moveAction(0, 0, true);
    await new Promise(r => setTimeout(r, 20));
    innerCircle.hidden = false;
    outerCircle.hidden = false;
    ledCircle.style.top = "0px";
    ledCircle.style.left = "0px";
  });
}

function mapRange(value, in_min, in_max, out_min, out_max) {
  return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min);
}

function moveAction(linear, angular, stop = false) {
  let command = {
    lin: linear,
    ang: angular,
    stop: stop,
  };
  socket.emit("drive_command", command);
}
