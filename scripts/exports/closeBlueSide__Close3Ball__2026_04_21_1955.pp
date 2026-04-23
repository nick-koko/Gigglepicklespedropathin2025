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
      "id": "line-e788826c57f6",
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
      "id": "line-19852076480c",
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
      "color": "#BF8605",
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
      "lineId": "line-e788826c57f6"
    },
    {
      "kind": "path",
      "lineId": "line-19852076480c"
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
  "timestamp": "2026-04-21T23:55:28.528Z"
}