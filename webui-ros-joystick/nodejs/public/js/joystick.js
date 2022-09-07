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

  let joystickContainer = document.querySelector('.joystick-container')
  joystickContainer.style.top = (posY - size / 2) + "px";
  joystickContainer.style.left = (posX - size / 2) + "px";

  let joystick = document.getElementById("joystick");
  joystick.style.width = size / 2 + "px";
  joystick.style.height = size / 2 + "px";
  joystick.style.top = size / 4 + "px";
  joystick.style.left = size / 4 + "px";

  let outerCircle = document.getElementById('outerCircle')
  outerCircle.style.width = size + "px";
  outerCircle.style.height = size + "px";
  outerCircle.style.top = 0 + "px";
  outerCircle.style.left = 0 + "px";

  let innerCircle = document.getElementById('innerCircle')
  innerCircle.style.width = size / 2 + "px";
  innerCircle.style.height = size / 2 + "px";
  innerCircle.style.top = size / 4 + "px";
  innerCircle.style.left = size / 4 + "px";

  var options = {
    zone: joystick,
    color: "#808080",
    size: size,
    fadeTime: 100,
  };
  manager = nipplejs.create(options);
  manager.on("start", async function (evt) {
    await new Promise(r => setTimeout(r, 50));
    innerCircle.hidden = true;
    outerCircle.hidden = true;
  })
  manager.on("move", function (evt, nipple) {
    if (beginJoyPosX == 0 && beginJoyPosY == 0) {
      beginJoyPosX = nipple.position.x;
      beginJoyPosY = nipple.position.y;
      lin = 0;
      ang = 0;
    } else {
      relativePosX = beginJoyPosX - nipple.position.x;
      relativePosY = beginJoyPosY - nipple.position.y;

      ang = mapRange(relativePosX, - size / 2, size / 2, -1, 1);
      lin = mapRange(relativePosY, - size / 2, size / 2, -1.1, 1.1);

      if (lin > 1.0) lin = 1.0;
      if (lin < -1.0) lin = -1.0;
    }

    clearTimeout(joystick_timeout);
    moveAction(lin, ang);
    joystick_timeout = setTimeout(function () {
      repeat_velcmd(lin, ang);
    }, velocity_repeat_delay);
  });
  manager.on("end", function () {
    clearTimeout(joystick_timeout);
    beginJoyPosX = 0;
    beginJoyPosY = 0;
    moveAction(0, 0, true);
    innerCircle.hidden = false;
    outerCircle.hidden = false;
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
