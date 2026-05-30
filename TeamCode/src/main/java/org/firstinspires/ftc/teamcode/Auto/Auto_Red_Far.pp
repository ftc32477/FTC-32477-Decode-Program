{
  "startPoint": {
    "x": 89.3,
    "y": 8.8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": true
  },
  "lines": [
    {
      "id": "line-yfr5e1biz7r",
      "name": "Path 1",
      "endPoint": {
        "x": 84,
        "y": 12,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 60
      },
      "controlPoints": [],
      "color": "#98AD55",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps5t1b4-pz3pol",
      "name": "Path 2",
      "endPoint": {
        "x": 111.2,
        "y": 6.7,
        "heading": "linear",
        "reverse": false,
        "startDeg": 60,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#98AD55",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps5wm6j-4ckipr",
      "name": "Path 3",
      "endPoint": {
        "x": 134,
        "y": 6.7,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#98AD55",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps5zrx7-8rqr3z",
      "name": "Path 4",
      "endPoint": {
        "x": 129,
        "y": 6.7,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#98AD55",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps61df4-ms9012",
      "name": "Path 7",
      "endPoint": {
        "x": 134,
        "y": 6.7,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#98AD55",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps62gbx-koxq0v",
      "name": "Path 8",
      "endPoint": {
        "x": 84,
        "y": 12,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 60
      },
      "controlPoints": [],
      "color": "#98AD55",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mps67hi5-uhri2l",
      "name": "Path 9",
      "endPoint": {
        "x": 137.3,
        "y": 8.8,
        "heading": "linear",
        "reverse": false,
        "startDeg": 60,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#98AD55",
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
      "lineId": "line-yfr5e1biz7r"
    },
    {
      "kind": "wait",
      "id": "mps5sg1x-ww7s6u",
      "name": "Wait",
      "durationMs": 3000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mps5t1b4-pz3pol"
    },
    {
      "kind": "path",
      "lineId": "mps5wm6j-4ckipr"
    },
    {
      "kind": "path",
      "lineId": "mps5zrx7-8rqr3z"
    },
    {
      "kind": "path",
      "lineId": "mps61df4-ms9012"
    },
    {
      "kind": "path",
      "lineId": "mps62gbx-koxq0v"
    },
    {
      "kind": "wait",
      "id": "mps63f0a-x57oig",
      "name": "Wait",
      "durationMs": 3000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mps67hi5-uhri2l"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mps5ngic-02f22o",
      "name": "Main Chain",
      "color": "#98AD55",
      "lineIds": [
        "line-yfr5e1biz7r",
        "mps5t1b4-pz3pol",
        "mps5wm6j-4ckipr",
        "mps5zrx7-8rqr3z",
        "mps61df4-ms9012",
        "mps62gbx-koxq0v",
        "mps67hi5-uhri2l"
      ]
    }
  ],
  "settings": {
    "xVelocity": 69,
    "yVelocity": 48,
    "aVelocity": 2.9845130209103035,
    "kFriction": 0.3,
    "rWidth": 17.5,
    "rHeight": 13.5,
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
  "timestamp": "2026-05-30T20:41:52.770Z"
}