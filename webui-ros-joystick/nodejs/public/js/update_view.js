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

    socket.on("e_stop_state", function (state) {
        let led = document.getElementById("led");
        let button = document.getElementById("eStopButton");
        let buttonInput = document.getElementById("eStopInput");

        if (!state) {
            if (!button.disabled)
                setButton(button, buttonInput, "OFF");
            led.style.backgroundColor = "lime";
        }
        else {
            if (!button.disabled)
                setButton(button, buttonInput, "ON");
            led.style.backgroundColor = "red";
        }
    });
};

$(window).resize(function () {
    setView();
});

function setView() {
    removeJoystick();
    joySize = 400;
    if (joySize > $(window).height()) {
        joySize = $(window).height();
    }
    if (joySize > $(window).width()) {
        joySize = $(window).width();
    }
    createJoystick(
        $(window).width() / 2,
        $(window).height() / 2 - 130,
        (joySize * 2) / 3
    );

    buttonSize = 200;
    createButton(
        $(window).width() / 2 - buttonSize / 2,
        $(window).height() / 2 - buttonSize / 2 + 150,
        buttonSize
    );
}