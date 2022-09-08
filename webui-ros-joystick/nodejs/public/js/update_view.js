let offset = 0;
let create_e_stop = false;

window.onload = function () {
    console.log("onLoad triggered");
    alert_container = document.getElementById("alerts");
    socket = io();
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

    socket.on("create_e_stop", function (state) {
        create_e_stop = state;
        if (create_e_stop) {
            setView();
            socket.on("e_stop_state", function (state) {
                let button = document.getElementById("eStopButton");
                let buttonInput = document.getElementById("eStopInput");
                let ledCircle = document.getElementById("ledCircle");
                let joystick = document.getElementById("joystick");

                if (!state) {
                    if (!button.disabled)
                        setButton(button, buttonInput, "OFF");
                        ledCircle.style.backgroundColor = LED_GREEN_COLOR;
                        joystick.hidden = false;
                }
                else {
                    if (!button.disabled)
                        setButton(button, buttonInput, "ON");
                        ledCircle.style.backgroundColor = LED_RED_COLOR;
                        joystick.hidden = true;
                }
            });
        } else {
            offset = 0;
            removeButton();
            setView();
            socket.off("e_stop_state");
        }
    })
};

$(window).resize(function () {
    setView();
});

function setView() {
    if (create_e_stop) {
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