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
      "id": "line-777108b190d4",
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
      "id": "line-6fe416195feb",
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
      "id": "line-85be119f9beb",
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
      "id": "line-593e5decb06f",
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
      "id": "line-ebb5b4f6a1bb",
      "name": "moveOffLineClose_offLineClose",
      "endPoint": {
        "x": 50.0,
        "y": 117.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 128.0,
        "endDeg": 135.0
      },
      "controlPoints": [],
      "color": "#DCDD95",
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
      "lineId": "line-777108b190d4"
    },
    {
      "kind": "path",
      "lineId": "line-6fe416195feb"
    },
    {
      "kind": "path",
      "lineId": "line-85be119f9beb"
    },
    {
      "kind": "path",
      "lineId": "line-593e5decb06f"
    },
    {
      "kind": "path",
      "lineId": "line-ebb5b4f6a1bb"
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
  "timestamp": "2026-02-03T22:31:47.310Z"
}