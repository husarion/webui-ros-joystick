"use strict";

var app = require("express")();
var express = require("express");
var http = require("http").createServer(app);
var io = require("socket.io")(http);
const yargs = require("yargs");
const rosnodejs = require("rosnodejs");
const { inflate } = require("zlib");
const geometry_msgs = rosnodejs.require("geometry_msgs").msg;
const std_msgs = rosnodejs.require("std_msgs").msg;
const std_srvs = rosnodejs.require("std_srvs").srv;

const drive_msg = new geometry_msgs.Twist();
var cmd_vel_publisher;
var e_stop_trigger_client;
var e_stop_reset_client;
var e_stop_subscriber;
var alerts = new Array();
let eStop = false;
let outputLinearVel = 0;
let outputAngularVel = 0;
let maxLinearAccel = 1.0;
let maxAngularAccel = 1.0;
let lastVelocityMsgTime = 0;

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
  throttleMs: 100,
};

// Define alert states
const ALERT_OK = 1;
const ALERT_WARN = 2;
const ALERT_ERROR = 3;

function update_alerts(alerts) {
  io.emit("alert_states", alerts);
}

const argv = yargs
  .option("linear_scale", {
    description: "Scale for linear velocity",
    default: 1,
    alias: "lin",
    type: "number",
  })
  .option("angular_scale", {
    alias: "ang",
    default: 1,
    description: "Scale for angular velocity",
    type: "number",
  })
  .option("wait", {
    alias: "wait_nodes",
    default: "",
    description: "Scale for angular velocity",
    type: "array",
  })
  .option("e_stop", {
    alias: "e_stop",
    default: false,
    description: "Spawn e-stop button",
    type: "boolean",
  })
  .help()
  .alias("help", "h")
  .version(false).argv;

console.log(
  "Velocities will be scaled by: [",
  argv.linear_scale,
  ", ",
  argv.angular_scale,
  "]"
);

console.log("Wait for nodes: ", argv.wait);

argv.wait.forEach((wait_node) => {
  if (wait_node != "") {
    let alert = new Alert(
      wait_node,
      ALERT_ERROR,
      "Waiting for " + wait_node + " node to start"
    );
    alerts.push(alert);
  }
});

app.get("/", function (req, res) {
  res.sendFile(__dirname + "/public/index.html");
});

app.use(express.static("public"));

io.on("connection", function (socket) {
  console.log("a user connected");
  update_alerts(alerts);
  socket.on("disconnect", function () {
    console.log("user disconnected");
  });

  socket.emit("create_e_stop", argv.e_stop);

  socket.on("e_stop_trigger", async (callback) => {
    let success = await handleCallTriggerService('/e_stop_trigger', e_stop_trigger_client, 5000);
    callback({
      success: success
    });
  });

  socket.on("e_stop_reset", async (callback) => {
    let success = await handleCallTriggerService('/e_stop_reset', e_stop_reset_client, 5000);
    callback({
      success: success
    });
  });

  socket.on("drive_command", function (drive_command) {
    if (!eStop) {
      if (!drive_command.stop) {
        let commandLinearVel = drive_command.lin * argv.linear_scale;
        let commandAngularVel = drive_command.ang * argv.angular_scale;
        outputLinearVel = accelLimiting(outputLinearVel, commandLinearVel, maxLinearAccel);
        outputAngularVel = accelLimiting(outputAngularVel, commandAngularVel, maxAngularAccel);
      } else {
        outputLinearVel = 0;
        outputAngularVel = 0;
      }
      drive_msg.linear.x = outputLinearVel;
      drive_msg.angular.z = outputAngularVel;
      lastVelocityMsgTime = rosnodejs.Time.toSeconds(rosnodejs.Time.now());
      cmd_vel_publisher.publish(drive_msg);
    }
  });
});

http.listen(8000, function () {
  console.log("listening on *:8000");
});

rosnodejs
  .initNode("/rosnodejs")
  .then((rosNode) => {

    cmd_vel_publisher = rosNode.advertise(
      "/cmd_vel",
      geometry_msgs.Twist,
      default_publisher_options
    );

    if (argv.e_stop) {
      e_stop_subscriber = rosNode.subscribe(
        "/e_stop",
        std_msgs.Bool,
        eStopCallback
      );
  
      e_stop_trigger_client = rosNode.serviceClient('/e_stop_trigger', std_srvs.Trigger);
      e_stop_reset_client = rosNode.serviceClient('/e_stop_reset', std_srvs.Trigger);
    }

    argv.wait.forEach((wait_node) => {
      let node_name = "/" + wait_node + "/get_loggers";
      rosNode.waitForService(node_name).then(() => {
        alerts.forEach((alert) => {
          if (alert._id == wait_node) {
            alert.update(wait_node, ALERT_OK, "Node " + node_name + " active");
          }
        });
        update_alerts(alerts);
      });
    });
  })
  .catch((err) => {
    rosnodejs.log.error(err.stack);
  });

var eStopCallback = (msg) => {
  eStop = msg.data;
  io.emit("e_stop_state", msg.data);
}

async function handleCallTriggerService(name, service, timeout) {
  try {
    let nh = rosnodejs.nodeHandle;
    if (await nh.waitForService(name, timeout)) {
      rosnodejs.log.info("Calling " + name + " service");
      let response = await service.call();
      rosnodejs.log.info(name + " service response: " + JSON.stringify(response));
      return response.success;
    } else {
      rosnodejs.log.error("Can't contact " + name + " service");
      return false;
    }
  }
  catch (error) {
    rosnodejs.log.error(error);
  }
}

function accelLimiting(currentVel, vel, maxAccel) {
  let outputVel = 0;
  let dt = 0.001;
  let currentTime = rosnodejs.Time.toSeconds(rosnodejs.Time.now());
  if (currentVel != 0) {
    dt = currentTime - lastVelocityMsgTime
  }

  if (vel >= currentVel) {
    outputVel = currentVel + maxAccel * dt;
    if (outputVel > vel) {
      outputVel = vel;
    }
  } else if (vel < currentVel) {
    outputVel = currentVel - maxAccel * dt;
    if (outputVel < vel) {
      outputVel = vel;
    }
  }

  return outputVel;
}
