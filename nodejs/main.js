'use strict';

var app = require('express')();
var express = require('express');
var http = require('http').createServer(app);
var io = require('socket.io')(http);
const yargs = require('yargs');
const rosnodejs = require('rosnodejs');
const { inflate } = require('zlib');
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

const drive_msg = new geometry_msgs.Twist();
var cmd_vel_publisher;
var alerts = new Array();

var Alert = function (id, state, message) {
    this._id = id;
    this._state = state;
    this._message = message;
};

Alert.prototype.update = function (id, state, message) {
    this._id = id;
    this._state = state;
    this._message = message;
};

const default_publisher_options = {
    queueSize: 1,
    latching: false,
    throttleMs: 100
}

// Define alert states
const ALERT_OK = 1
const ALERT_WARN = 2
const ALERT_ERROR = 3

function update_alerts(alerts) {
    io.emit('alert_states', alerts)
}

const argv = yargs
    .option('linear_scale', {
        description: 'Scale for linear velocity',
        default: 1,
        alias: 'lin',
        type: 'number'
    })
    .option('angular_scale', {
        alias: 'ang',
        default: 1,
        description: 'Scale for angular velocity',
        type: 'number',
    })
    .option('wait', {
        alias: 'wait_nodes',
        default: '',
        description: 'Scale for angular velocity',
        type: 'array',
    })
    .help()
    .alias('help', 'h')
    .version(false)
    .argv;

console.log("Velocities will be scaled by: [", argv.linear_scale, ", ", argv.angular_scale, "]")

console.log("Wait for nodes: ", argv.wait)

argv.wait.forEach((wait_node) => {
    let alert = new Alert(wait_node, ALERT_ERROR, "Waiting for " + wait_node + " node to start");
    alerts.push(alert);
})

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/public/index.html');
});

app.use(express.static('public'))

io.on('connection', function (socket) {
    console.log('a user connected');
    update_alerts(alerts);
    socket.on('disconnect', function () {
        console.log('user disconnected');
    });

    socket.on('drive_command', function (drive_command) {
        drive_msg.linear.x = drive_command.lin * argv.linear_scale;
        drive_msg.angular.z = drive_command.ang * argv.angular_scale;
        cmd_vel_publisher.publish(drive_msg);
    });
});

http.listen(8000, function () {
    console.log('listening on *:8000');
});

rosnodejs.initNode('/rosnodejs')
    .then((rosNode) => {
        cmd_vel_publisher = rosNode.advertise('/cmd_vel', geometry_msgs.Twist, default_publisher_options);
        argv.wait.forEach((wait_node) => {
            let node_name = "/" + wait_node + "/get_loggers"
            rosNode.waitForService(node_name)
                .then(() => {
                    alerts.forEach((alert) => {
                        if (alert._id == wait_node) {
                            alert.update(wait_node, ALERT_OK, "Node " + node_name + " active");
                        }
                    })
                    update_alerts(alerts)
                });
        })
    })
    .catch((err) => {
        rosnodejs.log.error(err.stack);
    });
