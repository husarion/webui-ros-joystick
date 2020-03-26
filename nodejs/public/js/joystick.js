
var joyPosY;
var joyPosX;
var manager;
var lin;
var ang;
var joystick_timeout;
var velocity_repeat_delay = 100; // [ms]
var max_joy_pos = 0;
var timerInstance;
var resize_tout;
var socket;

$(window).resize(function () {
    setView();
});

function removeJoystick() {
    joystickContainer = document.getElementById('joystick');
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
    createJoystick($(window).width() / 2, $(window).height() / 2, joySize * 2 / 3);
}

function repeat_velcmd(v_lin, v_ang) {
    moveAction(v_lin, v_ang)
    joystick_timeout = setTimeout(function () { repeat_velcmd(lin, ang); }, velocity_repeat_delay);
}

function createJoystick(x, y, d) {
    joystickContainer = document.getElementById('joystick');

    var options = {
        zone: joystickContainer,
        position: { left: x + 'px', top: y + 'px' },
        mode: 'static',
        size: d,
        color: '#222222',
        restJoystick: true
    };
    manager = nipplejs.create(options);
    manager.on('move', function (evt, nipple) {
        var direction = nipple.angle.degree - 90;
        if (direction > 180) {
            direction = -(450 - nipple.angle.degree);
        }
        // 0,25 m/s max speed
        // 0,5 rad/s max rotation speed
        lin = Math.cos(direction / 57.29) * 0.25 * nipple.distance / max_joy_pos;
        ang = Math.sin(direction / 57.29) * 0.5 * nipple.distance / max_joy_pos;
        clearTimeout(joystick_timeout);
        moveAction(lin, ang);
        joystick_timeout = setTimeout(function () { repeat_velcmd(lin, ang); }, velocity_repeat_delay);
    });
    manager.on('end', function () {
        clearTimeout(joystick_timeout);
        moveAction(0, 0);
    });
}

function moveAction(linear, angular) {
    let command = {
        lin: linear,
        ang: angular
    }
    socket.emit('drive_command', command);
}

window.onload = function () {
    console.log("onLoad triggered");
    socket = io();
    setView();
};
