{
  "startPoint": {
    "x": 20.25,
    "y": 119.5,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-g2goue6akxf",
      "name": "Path 1",
      "endPoint": {
        "x": 40,
        "y": 101,
        "heading": "linear",
        "startDeg": 143,
        "endDeg": 135
      },
      "controlPoints": [],
      "color": "#65A6AA",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpp62evw-a578ko",
      "name": "Path 2",
      "endPoint": {
        "x": 46.65059345689283,
        "y": 82.58362014184094,
        "heading": "linear",
        "reverse": false,
        "degrees": 0,
        "startDeg": 135,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 57.84367345126419,
          "y": 83.59119751349321
        }
      ],
      "color": "#65A6AA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpp636k0-n8aj3a",
      "name": "Path 3",
      "endPoint": {
        "x": 15.95209947549294,
        "y": 82.57368539456102,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#65A6AA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 120,
          "y": 141.5
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-g2goue6akxf"
    },
    {
      "kind": "path",
      "lineId": "mpp62evw-a578ko"
    },
    {
      "kind": "path",
      "lineId": "mpp636k0-n8aj3a"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mpp60vkn-p3gmzv",
      "name": "Main Chain",
      "color": "#65A6AA",
      "lineIds": [
        "line-g2goue6akxf",
        "mpp62evw-a578ko",
        "mpp636k0-n8aj3a"
      ]
    }
  ],
  "settings": {
    "xVelocity": 69,
    "yVelocity": 48,
    "aVelocity": 2.9845130209103035,
    "kFriction": 0.3,
    "rWidth": 17.7,
    "rHeight": 13.4,
    "safetyMargin": 1,
    "maxVelocity": 69,
    "maxAcceleration": 60,
    "maxDeceleration": 60,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": true,
    "onionLayerSpacing": 3,
    "onionColor": "#9ffcfe",
    "onionNextPointOnly": false,
    "showHeadingArrow": true,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-05-29T06:03:07.174Z"
}