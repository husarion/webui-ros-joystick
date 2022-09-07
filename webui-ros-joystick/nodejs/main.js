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
let outputLinVel = 0;
let outputAngVel = 0;
let maxLinVel;
let maxAngVel;
let maxLinAccel;
let maxAngAccel;
let eStopPresent;
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

function nodesStrToArray(nodes_str) {
  let nodes = nodes_str.split(" ");
  // remove empty fields
  let i = 0;
  while (i < nodes.length) {
    if (nodes[i] == "") {
      nodes.splice(i, 1);
    } else {
      ++i;
    }
  }

  nodes.forEach((wait_node) => {
    if (wait_node != "") {
      let alert = new Alert(
        wait_node,
        ALERT_ERROR,
        "Waiting for " + wait_node + " node to start"
      );
      alerts.push(alert);
    }
  });

  console.log("Wait for nodes: ", nodes);
  return nodes;
}

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

  socket.emit("create_e_stop", eStopPresent);

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
        let commandLinVel = drive_command.lin * maxLinVel;
        let commandAngVel = drive_command.ang * maxAngVel;
        outputLinVel = accelLimiting(outputLinVel, commandLinVel, maxLinAccel);
        outputAngVel = accelLimiting(outputAngVel, commandAngVel, maxAngAccel);
      } else {
        outputLinVel = 0;
        outputAngVel = 0;
      }
      drive_msg.linear.x = outputLinVel;
      drive_msg.angular.z = outputAngVel;
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
  .then(async (rosNode) => {

    cmd_vel_publisher = rosNode.advertise(
      "/cmd_vel",
      geometry_msgs.Twist,
      default_publisher_options
    );

    let privateNH = rosnodejs.getNodeHandle(rosNode.getNodeName());
    maxLinVel = await getRosParam(privateNH, "max_lin_vel", 1.0);
    maxAngVel = await getRosParam(privateNH, "max_ang_vel", 1.0);
    maxLinAccel = await getRosParam(privateNH, "max_lin_accel", 2.0);
    maxAngAccel = await getRosParam(privateNH, "max_ang_accel", 2.0);
    eStopPresent = await getRosParam(privateNH, "e_stop", false);
    let waitNodesStr = await getRosParam(privateNH, "wait_nodes", "");

    if (eStopPresent) {
      eStop = true;
      
      e_stop_subscriber = rosNode.subscribe(
        "/e_stop",
        std_msgs.Bool,
        eStopCallback
      );

      e_stop_trigger_client = rosNode.serviceClient('/e_stop_trigger', std_srvs.Trigger);
      e_stop_reset_client = rosNode.serviceClient('/e_stop_reset', std_srvs.Trigger);
    }

    let waitNodes = nodesStrToArray(waitNodesStr);
    waitNodes.forEach((wait_node) => {
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

async function getRosParam(nh, paramName, defaultValue = NaN) {
  let paramValue = defaultValue;
  await nh.hasParam(paramName).then(async (exists) => {
    if (exists) {
      await nh.getParam(paramName).then((value) => {
        paramValue = value;
      });
    }
  });
  rosnodejs.log.info(paramName + ": " + paramValue);
  return paramValue;
}

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
