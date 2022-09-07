var beginJoyPosX = 0;
var beginJoyPosY = 0;
var maxJoyPos = 0;
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
  joystickContainer = document.getElementById("joystick");

  var options = {
    zone: joystickContainer,
    position: { left: posX + "px", top: posY + "px" },
    mode: "static",
    size: size,
    color: "#808080",
    restJoystick: true,
  };
  manager = nipplejs.create(options);
  manager.on("move", function (evt, nipple) {
    if (beginJoyPosX == 0 && beginJoyPosY == 0) {
      beginJoyPosX = nipple.position.x;
      beginJoyPosY = nipple.position.y;
      lin = 0;
      ang = 0;
    } else {
      let diffY = beginJoyPosY - nipple.position.y;
      let maxY_diff = maxJoyPos - (posY - beginJoyPosY);
      let minY_diff = maxJoyPos + (posY - beginJoyPosY);
      let diffX = beginJoyPosX - nipple.position.x;
      let maxX_diff = maxJoyPos - (posX - beginJoyPosX);
      let minX_diff = maxJoyPos + (posX - beginJoyPosX);
      if (diffY > 0) {
        lin = (diffY * 1.1) / maxY_diff;
      } else {
        lin = (diffY * 1.1) / minY_diff;
      }
      if (diffX > 0) {
        ang = diffX / maxX_diff;
      } else {
        ang = diffX / minX_diff;
      }
    }
    if (lin > 1.0) lin = 1.0;
    if (lin < -1.0) lin = -1.0;

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
    lin = 0;
    ang = 0;
    moveAction(0, 0, true);
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
