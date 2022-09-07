const BUTTON_ON_COLOR = "#800000"
const BUTTON_OFF_COLOR = "#c00000"
const BUTTON_TEXT_COLOR = "#c4c4c4"
const BORDER_COLOR_GREY = "#777777"
const BORDER_COLOR_BLACK = "#272727"

function removeButton() {
  const eStopContainer = document.getElementsByClassName("eStopContainer");
  while (eStopContainer.length > 0) {
    eStopContainer[0].parentNode.removeChild(eStopContainer[0]);
  }
}

function createButton(posX, posY, size) {
  let button = document.getElementById("eStopButton");
  let eStopInput = document.getElementById("eStopInput");

  button.hidden = false;
  button.style.width = size + "px";
  button.style.height = size + "px";
  button.style.borderWidth = "16px";
  button.style.borderRadius = "50%";
  button.style.position = "absolute";
  button.style.left = posX + "px";
  button.style.top = posY + "px";
  button.style.borderStyle = "solid";
  button.style.fontSize = "40px";
  button.style.color = BUTTON_TEXT_COLOR;
  button.style.outline = 0;
  setButton(button, eStopInput, "ON");

  ledSize = 60;
  createLed(
    posX + size - ledSize + 45,
    posY + size - ledSize,
    ledSize
  );
}

function setButton(button, buttonInput, state) {
  buttonInput.value = state;
  if (state == "ON") {
    button.style.backgroundColor = BUTTON_ON_COLOR;
    setButtonBorder(button, "inset");
  } else {
    button.style.backgroundColor = BUTTON_OFF_COLOR;
    setButtonBorder(button, "outset");
  }
}

function setButtonBorder(button, type) {
  if (type == "inset") {
    button.style.borderBottomColor = BORDER_COLOR_GREY;
    button.style.borderRightColor = BORDER_COLOR_GREY;
    button.style.borderTopColor = BORDER_COLOR_BLACK;
    button.style.borderLeftColor = BORDER_COLOR_BLACK;
  }
  else if (type == "outset") {
    button.style.borderBottomColor = BORDER_COLOR_BLACK;
    button.style.borderRightColor = BORDER_COLOR_BLACK;
    button.style.borderTopColor = BORDER_COLOR_GREY;
    button.style.borderLeftColor = BORDER_COLOR_GREY;
  }
}

function createLed(posX, posY, size) {
  let led = document.getElementById("led");
  led.style.backgroundColor = "red";
  led.style.width = size + "px";
  led.style.height = size + "px";
  led.style.borderWidth = "5px"
  led.style.borderRadius = "50%";
  led.style.position = "absolute";
  led.style.left = posX + "px";
  led.style.top = posY + "px";
  led.style.borderStyle = "solid";
  led.style.borderColor = BORDER_COLOR_GREY;
}

function toggleButton(button) {
  let eStopInput = document.getElementById("eStopInput");
  if (eStopInput.value == "OFF") {
    setButton(button, eStopInput, "ON");
    buttonStateChange(button, eStopInput);
  } else {
    let eStopReset = confirm("Are you sure to disable emergency stop?");
    if (eStopReset) {
      setButton(button, eStopInput, "OFF");
      buttonStateChange(button, eStopInput);
    }
  }
}

function buttonStateChange(button, buttonInput) {
  button.disabled = true;
  if (buttonInput.value == "OFF") {
    socket.emit("e_stop_reset", (response) => {
      if (!response.success) {
        alert("Failed to reset E-STOP. \n" +
        "Check if physical E-STOP button is pressed. \n" +
        "You may also try to turn on and off physical E-STOP.");
        setButton(button, buttonInput, "ON");
      }
      button.disabled = false;
    });
  }
  else {
    socket.emit("e_stop_trigger", (response) => {
      if (!response.success) {
        alert("Failed to trigger e-stop");
        setButton(button, buttonInput, "OFF");
      }
      button.disabled = false;
    });
  }
}

