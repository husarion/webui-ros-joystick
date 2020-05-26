'use strict';

var app = require('express')();
var express = require('express');
var http = require('http').createServer(app);
var io = require('socket.io')(http);
const yargs = require('yargs');
const rosnodejs = require('rosnodejs');
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

const drive_msg = new geometry_msgs.Twist();
var cmd_vel_publisher;

const default_publisher_options = {
    queueSize: 1,
    latching: false,
    throttleMs: 100
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
    .help()
    .alias('help', 'h')
    .version(false)
    .argv;

console.log("Velocities will be scaled by: [", argv.linear_scale, ", ", argv.angular_scale, "]")

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/public/index.html');
});

app.use(express.static('public'))

io.on('connection', function (socket) {
    console.log('a user connected');
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
    })
    .catch((err) => {
        rosnodejs.log.error(err.stack);
    });
