"use strict";

var app = require("express")();
var express = require("express");
var http = require("http").createServer(app);
var io = require("socket.io")(http);
const yargs = require("yargs");
const rosnodejs = require("rosnodejs");
const { inflate } = require("zlib");
const geometryMsgs = rosnodejs.require("geometry_msgs").msg;
const stdMsgs = rosnodejs.require("std_msgs").msg;
const stdSrvs = rosnodejs.require("std_srvs").srv;

const driveMsg = new geometryMsgs.Twist();
var cmdVelPublisher;
var eStopTriggerClient;
var eStopResetClient;
var eStopSubscriber;
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

const defaultPublisherOptions = {
  queueSize: 1,
  latching: false,
  throttleMs: 100,
};

// Define alert states
const ALERT_OK = 1;
const ALERT_WARN = 2;
const ALERT_ERROR = 3;

function updateAlerts(alerts) {
  io.emit("alert_states", alerts);
}

function nodesStrToArray(nodesStr) {
  let nodes = nodesStr.split(" ");
  // remove empty fields
  let i = 0;
  while (i < nodes.length) {
    if (nodes[i] == "") {
      nodes.splice(i, 1);
    } else {
      ++i;
    }
  }

  nodes.forEach((waitNode) => {
    if (waitNode != "") {
      let alert = new Alert(
        waitNode,
        ALERT_ERROR,
        "Waiting for " + waitNode + " node to start"
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
  updateAlerts(alerts);
  socket.on("disconnect", function () {
    console.log("user disconnected");
  });

  socket.emit("create_e_stop", eStopPresent);

  socket.on("e_stop_trigger", async (callback) => {
    let success = await handleCallTriggerService('e_stop_trigger', eStopTriggerClient, 5000);
    callback({
      success: success
    });
  });

  socket.on("e_stop_reset", async (callback) => {
    let success = await handleCallTriggerService('e_stop_reset', eStopResetClient, 5000);
    callback({
      success: success
    });
  });

  socket.on("drive_command", function (driveCommand) {
    if (!eStop) {
      if (!driveCommand.stop) {
        let commandLinVel = driveCommand.lin * maxLinVel;
        let commandAngVel = driveCommand.ang * maxAngVel;
        outputLinVel = accelLimiting(outputLinVel, commandLinVel, maxLinAccel);
        outputAngVel = accelLimiting(outputAngVel, commandAngVel, maxAngAccel);
      } else {
        outputLinVel = 0;
        outputAngVel = 0;
      }
      driveMsg.linear.x = outputLinVel;
      driveMsg.angular.z = outputAngVel;
      lastVelocityMsgTime = rosnodejs.Time.toSeconds(rosnodejs.Time.now());
      cmdVelPublisher.publish(driveMsg);
    }
  });
});

http.listen(8000, function () {
  console.log("listening on *:8000");
});

rosnodejs
  .initNode("/rosnodejs")
  .then(async (rosNode) => {

    let privateNH = rosnodejs.getNodeHandle(rosNode.getNodeName());
    let waitNodesStr = await getRosParam(privateNH, "wait_nodes", "");
    let cmdVelTopic = await getRosParam(privateNH, "cmd_vel_topic", "/cmd_vel");
    maxLinVel = await getRosParam(privateNH, "max_lin_vel", 1.0);
    maxAngVel = await getRosParam(privateNH, "max_ang_vel", 1.0);
    maxLinAccel = await getRosParam(privateNH, "max_lin_accel", 2.0);
    maxAngAccel = await getRosParam(privateNH, "max_ang_accel", 2.0);
    eStopPresent = await getRosParam(privateNH, "e_stop", false);

    cmdVelPublisher = rosNode.advertise(
      cmdVelTopic,
      geometryMsgs.Twist,
      defaultPublisherOptions
    );

    if (eStopPresent) {
      eStop = true;

      eStopSubscriber = rosNode.subscribe(
        "e_stop",
        stdMsgs.Bool,
        eStopCallback
      );

      eStopTriggerClient = rosNode.serviceClient('e_stop_trigger', stdSrvs.Trigger);
      eStopResetClient = rosNode.serviceClient('e_stop_reset', stdSrvs.Trigger);
    }

    let waitNodes = nodesStrToArray(waitNodesStr);
    waitNodes.forEach((waitNode) => {
      let nodeName = "/" + waitNode + "/get_loggers";
      rosNode.waitForService(nodeName).then(() => {
        alerts.forEach((alert) => {
          if (alert._id == waitNode) {
            alert.update(waitNode, ALERT_OK, "Node " + nodeName + " active");
          }
        });
        updateAlerts(alerts);
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
