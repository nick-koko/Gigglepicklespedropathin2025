{
  "startPoint": {
    "x": 32.5,
    "y": 134.375,
    "heading": "linear",
    "startDeg": 180.0,
    "endDeg": 180.0,
    "locked": false
  },
  "lines": [
    {
      "id": "line-15d85f0abad3",
      "name": "firstshootpath_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 128.0
      },
      "controlPoints": [],
      "color": "#6ACDA5",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-7eda4c1af9e4",
      "name": "firstPickup_pickup1Pose",
      "endPoint": {
        "x": 40.0,
        "y": 86.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 128.0,
        "endDeg": 180.0
      },
      "controlPoints": [
        {
          "x": 46.5,
          "y": 97.0
        },
        {
          "x": 48.5,
          "y": 90.0
        }
      ],
      "color": "#BF8605",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-c294771f6e30",
      "name": "firstPickupEnd_pickup1EndPose",
      "endPoint": {
        "x": 24.0,
        "y": 86.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 180.0
      },
      "controlPoints": [],
      "color": "#BB86D5",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-c7991a0eb6ce",
      "name": "firstPickupEnd_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 128.0
      },
      "controlPoints": [],
      "color": "#CB6A76",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-d0ba3e5a133a",
      "name": "secondPickup_pickup2Pose",
      "endPoint": {
        "x": 40.0,
        "y": 60.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 128.0,
        "endDeg": 180.0
      },
      "controlPoints": [
        {
          "x": 47.0,
          "y": 91.5
        },
        {
          "x": 52.5,
          "y": 66.0
        }
      ],
      "color": "#DCDD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-b9f979039688",
      "name": "secondPickupEnd_pickup2EndPose",
      "endPoint": {
        "x": 23.5,
        "y": 60.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 180.0
      },
      "controlPoints": [],
      "color": "#B9A966",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-71c73f4c4e67",
      "name": "secondPickupEnd_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 128.0
      },
      "controlPoints": [
        {
          "x": 44.0,
          "y": 57.0
        },
        {
          "x": 44.0,
          "y": 57.0
        }
      ],
      "color": "#55A978",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "line-9702901def79",
      "name": "moveOffLineLever_offLineLever",
      "endPoint": {
        "x": 30.0,
        "y": 90.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 128.0,
        "endDeg": 180.0
      },
      "controlPoints": [],
      "color": "#8B8A95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-15d85f0abad3"
    },
    {
      "kind": "path",
      "lineId": "line-7eda4c1af9e4"
    },
    {
      "kind": "path",
      "lineId": "line-c294771f6e30"
    },
    {
      "kind": "path",
      "lineId": "line-c7991a0eb6ce"
    },
    {
      "kind": "path",
      "lineId": "line-d0ba3e5a133a"
    },
    {
      "kind": "path",
      "lineId": "line-b9f979039688"
    },
    {
      "kind": "path",
      "lineId": "line-71c73f4c4e67"
    },
    {
      "kind": "path",
      "lineId": "line-9702901def79"
    }
  ],
  "settings": {
    "xVelocity": 78.0,
    "yVelocity": 62.5,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-03T22:31:47.312Z"
}