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

function createJoystick(pos_x, pos_y, size) {
  joystickContainer = document.getElementById("joystick");

  var options = {
    zone: joystickContainer,
    position: { left: pos_x + "px", top: pos_y + "px" },
    mode: "static",
    size: size,
    color: "#808080",
    restJoystick: true,
  };
  manager = nipplejs.create(options);
  manager.on("move", function (evt, nipple) {
    relativePosX = pos_x - nipple.position.x;
    relativePosY = pos_y - nipple.position.y;

    ang = mapRange(relativePosX, - size / 2, size / 2, -1, 1);
    lin = mapRange(relativePosY, - size / 2, size / 2, -1, 1);

    clearTimeout(joystick_timeout);
    moveAction(lin, ang);
    joystick_timeout = setTimeout(function () {
      repeat_velcmd(lin, ang);
    }, velocity_repeat_delay);
  });
  manager.on("end", function () {
    clearTimeout(joystick_timeout);
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
