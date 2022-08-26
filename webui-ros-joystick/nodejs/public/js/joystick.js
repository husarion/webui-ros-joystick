var joyPosY;
var beginJoyPosY = 0;
var joyPosX;
var beginJoyPosX = 0;
var manager;
var lin;
var ang;
var joystick_timeout;
var velocity_repeat_delay = 100; // [ms]
var max_joy_pos = 0;
var timerInstance;
var resize_tout;
var socket;
var alert_container;

$(window).resize(function () {
  setView();
});

function removeJoystick() {
  joystickContainer = document.getElementById("joystick");
  while (joystickContainer.hasChildNodes()) {
    joystickContainer.removeChild(joystickContainer.childNodes[0]);
  }
  if (!jQuery.isEmptyObject(manager)) {
    manager.destroy();
  }
}

function setView() {
  removeJoystick();
  joySize = 400;
  if (joySize > $(window).height()) {
    joySize = $(window).height();
  }
  if (joySize > $(window).width()) {
    joySize = $(window).width();
  }
  max_joy_pos = joySize / 3;
  createJoystick(
    $(window).width() / 2,
    $(window).height() / 2,
    (joySize * 2) / 3
  );
}

function repeat_velcmd(v_lin, v_ang) {
  moveAction(v_lin, v_ang);
  joystick_timeout = setTimeout(function () {
    repeat_velcmd(lin, ang);
  }, velocity_repeat_delay);
}

function createJoystick(x, y, d) {
  joystickContainer = document.getElementById("joystick");

  var options = {
    zone: joystickContainer,
    position: { left: x + "px", top: y + "px" },
    mode: "static",
    size: d,
    color: "#222222",
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
      let maxY_diff = max_joy_pos - ($(window).height() / 2 - beginJoyPosY);
      let minY_diff = max_joy_pos + ($(window).height() / 2 - beginJoyPosY);
      let diffX = beginJoyPosX - nipple.position.x;
      let maxX_diff = max_joy_pos - ($(window).width() / 2 - beginJoyPosX);
      let minX_diff = max_joy_pos + ($(window).width() / 2 - beginJoyPosX);
      if (diffY > 0) {
        lin = (diffY * 1.3) / maxY_diff;
      } else {
        lin = (diffY * 1.3) / minY_diff;
      }
      if (diffX > 0) {
        ang = (diffX * 1.0) / maxX_diff;
      } else {
        ang = (diffX * 1.0) / minX_diff;
      }
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
    moveAction(0, 0);
  });
}

function moveAction(linear, angular) {
  let command = {
    lin: linear,
    ang: angular,
  };
  socket.emit("drive_command", command);
}

window.onload = function () {
  console.log("onLoad triggered");
  alert_container = document.getElementById("alerts");
  socket = io();
  setView();
  socket.on("alert_states", function (alert_state) {
    let alerts_section = new String();
    alert_state.forEach((alert) => {
      if (alert._state == 2) {
        alerts_section += '<div class="alert alert-warning" role="alert">';
        alerts_section += alert._message;
        alerts_section += "</div>";
      } else if (alert._state == 3) {
        alerts_section += '<div class="alert alert-danger" role = "alert">';
        alerts_section += alert._message;
        alerts_section += "</div>";
      }
    });
    alert_container.innerHTML = alerts_section;
  });
};
