const BUTTON_ON_COLOR = "#508040" //"#378037"
const BUTTON_OFF_COLOR = "#a80000"
const BUTTON_TEXT_COLOR = "#c4c4c4"
const BORDER_COLOR_GREY = "#404040"
const BORDER_COLOR_BLACK = "#272727"

function removeButton() {
  const eStopContainer = document.getElementsByClassName("eStopContainer");
  while (eStopContainer.length > 0) {
    eStopContainer[0].parentNode.removeChild(eStopContainer[0]);
  }
}

function createButton(posX, posY, size) {
  let buttonContainer = document.querySelector(".eStopContainer");
  let button = document.getElementById("eStopButton");
  let buttonInput = document.getElementById("eStopInput");

  button.hidden = false;
  button.style.width = size + "px";
  button.style.height = size + "px";
  button.style.borderWidth = size / 12.5 + "px";
  button.style.borderRadius = "50%";
  button.style.position = "absolute";
  button.style.left = -size / 2 + "px";
  button.style.top = -size / 2 + "px";
  button.style.borderStyle = "solid";
  button.style.fontSize = size / 5 + "px";
  button.style.color = BUTTON_TEXT_COLOR;
  button.style.outline = 0;
  setButton(button, buttonInput, "ON");

  buttonContainer.style.left = posX + "px";
  buttonContainer.style.top = posY + "px";
}

function setButton(button, buttonInput, state) {
  buttonInput.value = state;
  if (state == "ON") {
    button.style.backgroundColor = BUTTON_ON_COLOR;
    button.textContent = "GO";
    setButtonBorder(button, "inset");
  } else {
    button.style.backgroundColor = BUTTON_OFF_COLOR;
    button.textContent = "E-STOP";
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

