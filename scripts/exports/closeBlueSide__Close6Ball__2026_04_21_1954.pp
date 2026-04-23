{
  "startPoint": {
    "x": 31.5,
    "y": 134.0,
    "heading": "linear",
    "startDeg": 270.0,
    "endDeg": 270.0,
    "locked": false
  },
  "lines": [
    {
      "id": "line-df096abd234f",
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
      "id": "line-d0e446a62984",
      "name": "firstPickup_pickup1Pose",
      "endPoint": {
        "x": 40.0,
        "y": 88.0,
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
      "id": "line-55832514bbdd",
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
      "id": "line-a1cc5eab38db",
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
      "id": "line-f72f4765189f",
      "name": "moveOffLineClose_offLineClose",
      "endPoint": {
        "x": 50.0,
        "y": 117.0,
        "heading": "linear",
        "reverse": false,
        "startDeg": 225.0,
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
      "lineId": "line-df096abd234f"
    },
    {
      "kind": "path",
      "lineId": "line-d0e446a62984"
    },
    {
      "kind": "path",
      "lineId": "line-55832514bbdd"
    },
    {
      "kind": "path",
      "lineId": "line-a1cc5eab38db"
    },
    {
      "kind": "path",
      "lineId": "line-f72f4765189f"
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
  "timestamp": "2026-04-21T23:54:59.168Z"
}