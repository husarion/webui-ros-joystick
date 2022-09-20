let offset = 0;
let createEStop = false;

window.onload = function () {
  console.log("onLoad triggered");
  alertContainer = document.getElementById("alerts");
  connectionWarn = document.getElementById("connectionWarn");
  button = document.getElementById("eStopButton");
  socket = io();

  socket.on("connect", function () {
    console.log("connected to server");
    connectionWarn.hidden = true;
    button.disabled = false;
  });

  socket.on("disconnect", function () {
    console.log("disconected from the server");
    joystick.hidden = true;
    connectionWarn.hidden = false;
    button.disabled = true;
  })

  socket.on("alert_states", function (alertState) {
    let alertsSection = new String();
    alertState.forEach((alert) => {
      if (alert._state == 2) {
        alertsSection += '<div class="alert alert-warning" role="alert">';
        alertsSection += alert._message;
        alertsSection += "</div>";
      } else if (alert._state == 3) {
        alertsSection += '<div class="alert alert-danger" role = "alert">';
        alertsSection += alert._message;
        alertsSection += "</div>";
      }
    });
    alertContainer.innerHTML = alertsSection;
  });

  socket.on("create_e_stop", function (state) {
    createEStop = state;
    let joystick = document.getElementById("joystick");
    let ledCircle = document.getElementById("ledCircle");
    if (createEStop) {
      setView();
      socket.on("e_stop_state", function (state) {
        let button = document.getElementById("eStopButton");
        let buttonInput = document.getElementById("eStopInput");

        if (!button.disabled) {
          if (!state) {
            setButton(button, buttonInput, "OFF");
            ledCircle.style.backgroundColor = LED_GREEN_COLOR;
            joystick.hidden = false;
          } else {
            setButton(button, buttonInput, "ON");
            ledCircle.style.backgroundColor = LED_RED_COLOR;
            joystick.hidden = true;
          }
        }
      });
    } else {
      offset = 0;
      removeButton();
      setView();
      socket.off("e_stop_state");
      joystick.hidden = false;
      ledCircle.hidden = true;
    }
  })
};

function setView() {
  if (createEStop) {
    let buttonSize = 100;
    let toolBarHeight = buttonSize * 1.2;
    offset = toolBarHeight / 2;
    let toolBar = document.querySelector(".toolBar");
    toolBar.style.height = toolBarHeight + "px";

    createButton(
      $(window).width() - toolBarHeight / 1.5,
      toolBarHeight / 2,
      buttonSize
    )
  }

  removeJoystick();
  let joySize = 400;
  if (joySize > $(window).height()) {
    joySize = $(window).height();
  }
  if (joySize > $(window).width()) {
    joySize = $(window).width();
  }
  createJoystick(
    $(window).width() / 2,
    $(window).height() / 2 + offset,
    (joySize * 2) / 3
  );
}

window.addEventListener("resize", setView);
window.addEventListener("focus", setView);
window.addEventListener("blur", removeJoystick);
// disable context menu
document.addEventListener('contextmenu', event => event.preventDefault());