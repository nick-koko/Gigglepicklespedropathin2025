{
  "startPoint": {
    "x": 32.5,
    "y": 134.375,
    "heading": "linear",
    "startDeg": 270.0,
    "endDeg": 270.0,
    "locked": false
  },
  "lines": [
    {
      "id": "line-c5e0e8d3b533",
      "name": "firstshootpath_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270.0,
        "endDeg": 225.0
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
      "id": "line-e7215c4e45d1",
      "name": "firstPickup_pickup1Pose",
      "endPoint": {
        "x": 40.0,
        "y": 86.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 225.0,
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
      "id": "line-9f6e882caa35",
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
      "id": "line-f7286c4f158d",
      "name": "firstPickupEnd_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 225.0
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
      "id": "line-faa27c0a2e1f",
      "name": "secondPickup_pickup2Pose",
      "endPoint": {
        "x": 40.0,
        "y": 60.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 225.0,
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
      "id": "line-7316a6b203fc",
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
      "id": "line-ad71bb019a9b",
      "name": "secondPickupEnd_scorePoseClose",
      "endPoint": {
        "x": 33.0,
        "y": 107.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180.0,
        "endDeg": 225.0
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
      "id": "line-2a61665153c1",
      "name": "moveOffLineLever_offLineLever",
      "endPoint": {
        "x": 30.0,
        "y": 90.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 225.0,
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
      "lineId": "line-c5e0e8d3b533"
    },
    {
      "kind": "path",
      "lineId": "line-e7215c4e45d1"
    },
    {
      "kind": "path",
      "lineId": "line-9f6e882caa35"
    },
    {
      "kind": "path",
      "lineId": "line-f7286c4f158d"
    },
    {
      "kind": "path",
      "lineId": "line-faa27c0a2e1f"
    },
    {
      "kind": "path",
      "lineId": "line-7316a6b203fc"
    },
    {
      "kind": "path",
      "lineId": "line-ad71bb019a9b"
    },
    {
      "kind": "path",
      "lineId": "line-2a61665153c1"
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
  "timestamp": "2026-04-09T03:04:51.880Z"
}