#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iterator>
#include <algorithm>
#include "nlohmann/json.hpp"
#include "common/RabbitMQConfig.h"
#include "common/PathUtils.h"
#include "external_sender_helpers.h"

int main() {
    RabbitMQConfig cfg = RabbitMQConfigFromEnv({});
    std::string exch = std::getenv("EXT_EXCHANGE")? std::getenv("EXT_EXCHANGE"): cfg.dispToAlgoExchange;
    std::string queue = std::getenv("EXT_QUEUE")? std::getenv("EXT_QUEUE"): cfg.dispToAlgoQueue;
    std::string rkey  = std::getenv("EXT_ROUTING_KEY")? std::getenv("EXT_ROUTING_KEY"): cfg.dispToAlgoRoutingKey;
    std::string bindingKey = std::getenv("EXT_BINDING_KEY")? std::getenv("EXT_BINDING_KEY"): cfg.dispToAlgoBindingKey;
    std::string exchType = std::getenv("EXT_EXCHANGE_TYPE")? std::getenv("EXT_EXCHANGE_TYPE"): "fanout";
    int messageTtlMs = 0;
    if (const char* ttlEnv = std::getenv("EXT_MESSAGE_TTL_MS")) {
        try { messageTtlMs = std::max(0, std::stoi(ttlEnv)); } catch (...) {}
    }

    // 默认数据（当文件不存在或加载失败时使用）
    static const std::string defaultData = R"JSON({
  "schedulingRequestId": "REQ_20251119_114102",
  "requestTimestamp": "2025-11-19T11:41:02.388Z",
  "requestType": "批量",
  "taskNumber": 50,
  "triggerContext": {
    "type": "周期检查触发"
  },
  "triggerAgvId": [],
  "batchConfig": {
    "maxBatchSize": 50
  },
  "timeoutMs": 2000,
  "environmentContext": {
    "mapVersion": "1",
    "systemLoad": 54,
    "systemLoadFactor": 0.4
  },
  "candidateTasks": [
    {
      "taskId": "TASK_001",
      "priority": 2,
      "expectedStartTime": "2025-11-19T11:41:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:16:02+00:00Z",
      "minBatteryLevel": 40,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "BDD43174BBEF652A",
          "point": {
            "x": 237200,
            "y": 201200,
            "angle": 90,
            "nodeId": "2184"
          },
          "estimatedDuration": 113,
          "pathConstraints": [],
          "maxSpeed": 139
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_002",
      "priority": 4,
      "expectedStartTime": "2025-11-19T11:46:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:44:02+00:00Z",
      "minBatteryLevel": 37,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BB2E3D72155D160",
          "point": {
            "x": 253475,
            "y": 315221,
            "angle": 180,
            "nodeId": "3464"
          },
          "estimatedDuration": 135,
          "pathConstraints": [],
          "maxSpeed": 130
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "9B2F0A0A4849B067",
          "point": {
            "x": 197465,
            "y": 330024,
            "angle": 0,
            "nodeId": "2022"
          },
          "estimatedDuration": 125,
          "pathConstraints": [],
          "maxSpeed": 128
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_003",
      "priority": 5,
      "expectedStartTime": "2025-11-19T11:51:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:30:02+00:00Z",
      "minBatteryLevel": 31,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "A56067BFD007DF52",
          "point": {
            "x": 250597,
            "y": 266524,
            "angle": 0,
            "nodeId": "4950"
          },
          "estimatedDuration": 65,
          "pathConstraints": [],
          "maxSpeed": 144
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "9ACED84932A7227A",
          "point": {
            "x": 255000,
            "y": 280466,
            "angle": 0,
            "nodeId": "1251"
          },
          "estimatedDuration": 143,
          "pathConstraints": [],
          "maxSpeed": 131
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B0D8FC6E36FD57F5",
          "point": {
            "x": 259165,
            "y": 205400,
            "angle": 180,
            "nodeId": "3078"
          },
          "estimatedDuration": 64,
          "pathConstraints": [],
          "maxSpeed": 114
        },
        {
          "sequence": 4,
          "pointType": "DROPOFF",
          "location": "9E1DFF6526F3D1E8",
          "point": {
            "x": 281336,
            "y": 314630,
            "angle": 270,
            "nodeId": "2698"
          },
          "estimatedDuration": 54,
          "pathConstraints": [],
          "maxSpeed": 117
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_004",
      "priority": 9,
      "expectedStartTime": "2025-11-19T11:56:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:48:02+00:00Z",
      "minBatteryLevel": 38,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "9B8CC7DC26DE44E8",
          "point": {
            "x": 263345,
            "y": 297233,
            "angle": 0,
            "nodeId": "2435"
          },
          "estimatedDuration": 139,
          "pathConstraints": [],
          "maxSpeed": 142
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_005",
      "priority": 2,
      "expectedStartTime": "2025-11-19T12:01:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:15:02+00:00Z",
      "minBatteryLevel": 34,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "AC3BD12AD7A75AF2",
          "point": {
            "x": 249350,
            "y": 207200,
            "angle": 0,
            "nodeId": "4781"
          },
          "estimatedDuration": 101,
          "pathConstraints": [],
          "maxSpeed": 124
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "BEF19F3AE0508A59",
          "point": {
            "x": 278616,
            "y": 309619,
            "angle": 0,
            "nodeId": "2806"
          },
          "estimatedDuration": 63,
          "pathConstraints": [],
          "maxSpeed": 113
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_006",
      "priority": 5,
      "expectedStartTime": "2025-11-19T12:06:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:06:02+00:00Z",
      "minBatteryLevel": 38,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "BF1EDE27EDEC0A17",
          "point": {
            "x": 226790,
            "y": 373000,
            "angle": 90,
            "nodeId": "4030"
          },
          "estimatedDuration": 87,
          "pathConstraints": [],
          "maxSpeed": 131
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "939110C8D39FF36F",
          "point": {
            "x": 265560,
            "y": 228427,
            "angle": 270,
            "nodeId": "2008"
          },
          "estimatedDuration": 112,
          "pathConstraints": [],
          "maxSpeed": 127
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "915E8D3F36CF7413",
          "point": {
            "x": 202811,
            "y": 226433,
            "angle": 0,
            "nodeId": "4211"
          },
          "estimatedDuration": 55,
          "pathConstraints": [],
          "maxSpeed": 136
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_007",
      "priority": 7,
      "expectedStartTime": "2025-11-19T12:11:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:33:02+00:00Z",
      "minBatteryLevel": 30,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "AC22F06C4D6878F4",
          "point": {
            "x": 158233,
            "y": 276422,
            "angle": 0,
            "nodeId": "4685"
          },
          "estimatedDuration": 75,
          "pathConstraints": [],
          "maxSpeed": 114
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "AF178459027E9C5D",
          "point": {
            "x": 231200,
            "y": 221475,
            "angle": 270,
            "nodeId": "1705"
          },
          "estimatedDuration": 92,
          "pathConstraints": [],
          "maxSpeed": 117
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_008",
      "priority": 4,
      "expectedStartTime": "2025-11-19T12:16:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:09:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "9C8332F2279DD770",
          "point": {
            "x": 223934,
            "y": 371050,
            "angle": 0,
            "nodeId": "4745"
          },
          "estimatedDuration": 148,
          "pathConstraints": [],
          "maxSpeed": 127
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_009",
      "priority": 5,
      "expectedStartTime": "2025-11-19T12:21:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:09:02+00:00Z",
      "minBatteryLevel": 30,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "95300F66F3DFD71C",
          "point": {
            "x": 201200,
            "y": 218000,
            "angle": 180,
            "nodeId": "1140"
          },
          "estimatedDuration": 44,
          "pathConstraints": [],
          "maxSpeed": 149
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "B8935FC26C1841FB",
          "point": {
            "x": 262200,
            "y": 276569,
            "angle": 180,
            "nodeId": "1434"
          },
          "estimatedDuration": 66,
          "pathConstraints": [],
          "maxSpeed": 102
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "91D7CEEA237BDEBE",
          "point": {
            "x": 210800,
            "y": 206000,
            "angle": 180,
            "nodeId": "739"
          },
          "estimatedDuration": 108,
          "pathConstraints": [],
          "maxSpeed": 129
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_010",
      "priority": 9,
      "expectedStartTime": "2025-11-19T12:26:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:23:02+00:00Z",
      "minBatteryLevel": 37,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "9F9A859BEA9DBF54",
          "point": {
            "x": 223100,
            "y": 398235,
            "angle": 90,
            "nodeId": "4639"
          },
          "estimatedDuration": 94,
          "pathConstraints": [],
          "maxSpeed": 143
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_011",
      "priority": 3,
      "expectedStartTime": "2025-11-19T12:31:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T12:58:02+00:00Z",
      "minBatteryLevel": 36,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BCCFA4FD8DFA997D",
          "point": {
            "x": 263175,
            "y": 221677,
            "angle": 270,
            "nodeId": "3740"
          },
          "estimatedDuration": 43,
          "pathConstraints": [],
          "maxSpeed": 123
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "88C62FB46543233E",
          "point": {
            "x": 196366,
            "y": 322580,
            "angle": 270,
            "nodeId": "3800"
          },
          "estimatedDuration": 93,
          "pathConstraints": [],
          "maxSpeed": 126
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "9E06DF9333D59147",
          "point": {
            "x": 252327,
            "y": 233873,
            "angle": 0,
            "nodeId": "2165"
          },
          "estimatedDuration": 45,
          "pathConstraints": [],
          "maxSpeed": 131
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_012",
      "priority": 5,
      "expectedStartTime": "2025-11-19T12:36:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:24:02+00:00Z",
      "minBatteryLevel": 37,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "AC3FD1B2763CC14C",
          "point": {
            "x": 229113,
            "y": 222585,
            "angle": 0,
            "nodeId": "4140"
          },
          "estimatedDuration": 58,
          "pathConstraints": [],
          "maxSpeed": 126
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "B4322EC68CE1E413",
          "point": {
            "x": 236000,
            "y": 222585,
            "angle": 180,
            "nodeId": "2318"
          },
          "estimatedDuration": 64,
          "pathConstraints": [],
          "maxSpeed": 116
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "90C9D60C394F0547",
          "point": {
            "x": 206468,
            "y": 244376,
            "angle": 90,
            "nodeId": "3847"
          },
          "estimatedDuration": 129,
          "pathConstraints": [],
          "maxSpeed": 148
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "A92BBDFEE099309D",
          "point": {
            "x": 262005,
            "y": 278069,
            "angle": 180,
            "nodeId": "4625"
          },
          "estimatedDuration": 58,
          "pathConstraints": [],
          "maxSpeed": 123
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_013",
      "priority": 2,
      "expectedStartTime": "2025-11-19T12:41:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:21:02+00:00Z",
      "minBatteryLevel": 37,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8BB0E70C2396C620",
          "point": {
            "x": 248345,
            "y": 238500,
            "angle": 270,
            "nodeId": "2448"
          },
          "estimatedDuration": 35,
          "pathConstraints": [],
          "maxSpeed": 110
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "90C2A265B6B213E",
          "point": {
            "x": 225200,
            "y": 208400,
            "angle": 180,
            "nodeId": "827"
          },
          "estimatedDuration": 139,
          "pathConstraints": [],
          "maxSpeed": 128
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_014",
      "priority": 2,
      "expectedStartTime": "2025-11-19T12:46:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:40:02+00:00Z",
      "minBatteryLevel": 32,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "89ACBA1044F4FB47",
          "point": {
            "x": 255000,
            "y": 270724,
            "angle": 270,
            "nodeId": "4932"
          },
          "estimatedDuration": 49,
          "pathConstraints": [],
          "maxSpeed": 141
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "BA7C4EC21537897B",
          "point": {
            "x": 211860,
            "y": 233873,
            "angle": 180,
            "nodeId": "1852"
          },
          "estimatedDuration": 74,
          "pathConstraints": [],
          "maxSpeed": 143
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_015",
      "priority": 2,
      "expectedStartTime": "2025-11-19T12:51:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:42:02+00:00Z",
      "minBatteryLevel": 39,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "B1C65C1A7CA6E2C1",
          "point": {
            "x": 226200,
            "y": 387222,
            "angle": 90,
            "nodeId": "4650"
          },
          "estimatedDuration": 72,
          "pathConstraints": [],
          "maxSpeed": 109
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "8EAC03C5209918D4",
          "point": {
            "x": 226100,
            "y": 402015,
            "angle": 0,
            "nodeId": "4495"
          },
          "estimatedDuration": 61,
          "pathConstraints": [],
          "maxSpeed": 133
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "9F2A3DF7B85D45D0",
          "point": {
            "x": 228885,
            "y": 353580,
            "angle": 0,
            "nodeId": "3638"
          },
          "estimatedDuration": 45,
          "pathConstraints": [],
          "maxSpeed": 143
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_016",
      "priority": 7,
      "expectedStartTime": "2025-11-19T12:56:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:26:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "B66CB267648BD72B",
          "point": {
            "x": 234447,
            "y": 282269,
            "angle": 180,
            "nodeId": "4490"
          },
          "estimatedDuration": 32,
          "pathConstraints": [],
          "maxSpeed": 109
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_017",
      "priority": 7,
      "expectedStartTime": "2025-11-19T13:01:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:38:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "AAC4688175153B76",
          "point": {
            "x": 197360,
            "y": 263472,
            "angle": 180,
            "nodeId": "3301"
          },
          "estimatedDuration": 38,
          "pathConstraints": [],
          "maxSpeed": 137
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "83CE6C61D4DEAB45",
          "point": {
            "x": 286489,
            "y": 294962,
            "angle": 0,
            "nodeId": "1514"
          },
          "estimatedDuration": 139,
          "pathConstraints": [],
          "maxSpeed": 150
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_018",
      "priority": 2,
      "expectedStartTime": "2025-11-19T13:06:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:44:02+00:00Z",
      "minBatteryLevel": 40,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "AAEAA26B05A42C58",
          "point": {
            "x": 262265,
            "y": 218810,
            "angle": 90,
            "nodeId": "3012"
          },
          "estimatedDuration": 144,
          "pathConstraints": [],
          "maxSpeed": 111
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_019",
      "priority": 8,
      "expectedStartTime": "2025-11-19T13:11:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:47:02+00:00Z",
      "minBatteryLevel": 32,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "940A1DE0F27DBBBF",
          "point": {
            "x": 278736,
            "y": 294962,
            "angle": 270,
            "nodeId": "2711"
          },
          "estimatedDuration": 134,
          "pathConstraints": [],
          "maxSpeed": 149
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "8CB773E8D0A4831C",
          "point": {
            "x": 284982,
            "y": 241474,
            "angle": 180,
            "nodeId": "1455"
          },
          "estimatedDuration": 87,
          "pathConstraints": [],
          "maxSpeed": 149
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "BF5CB00AABE89329",
          "point": {
            "x": 295389,
            "y": 294274,
            "angle": 180,
            "nodeId": "5066"
          },
          "estimatedDuration": 112,
          "pathConstraints": [],
          "maxSpeed": 100
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_020",
      "priority": 4,
      "expectedStartTime": "2025-11-19T13:16:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:03:02+00:00Z",
      "minBatteryLevel": 40,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8BED48D0C8B43E6F",
          "point": {
            "x": 251108,
            "y": 234873,
            "angle": 90,
            "nodeId": "2183"
          },
          "estimatedDuration": 38,
          "pathConstraints": [],
          "maxSpeed": 121
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "957245AD64BC97D5",
          "point": {
            "x": 286489,
            "y": 251074,
            "angle": 90,
            "nodeId": "1472"
          },
          "estimatedDuration": 121,
          "pathConstraints": [],
          "maxSpeed": 145
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_021",
      "priority": 3,
      "expectedStartTime": "2025-11-19T13:21:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:40:02+00:00Z",
      "minBatteryLevel": 30,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "AC5FA59BB0B23A12",
          "point": {
            "x": 196366,
            "y": 328730,
            "angle": 90,
            "nodeId": "3806"
          },
          "estimatedDuration": 45,
          "pathConstraints": [],
          "maxSpeed": 148
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "9E66A8E1994B8A55",
          "point": {
            "x": 221065,
            "y": 222585,
            "angle": 180,
            "nodeId": "2261"
          },
          "estimatedDuration": 55,
          "pathConstraints": [],
          "maxSpeed": 118
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B9843213D436482C",
          "point": {
            "x": 265855,
            "y": 291017,
            "angle": 270,
            "nodeId": "2839"
          },
          "estimatedDuration": 111,
          "pathConstraints": [],
          "maxSpeed": 110
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "8BC3621349879843",
          "point": {
            "x": 214400,
            "y": 206000,
            "angle": 90,
            "nodeId": "742"
          },
          "estimatedDuration": 109,
          "pathConstraints": [],
          "maxSpeed": 118
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_022",
      "priority": 4,
      "expectedStartTime": "2025-11-19T13:26:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:43:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "961C973A107255DA",
          "point": {
            "x": 212000,
            "y": 221475,
            "angle": 90,
            "nodeId": "1721"
          },
          "estimatedDuration": 125,
          "pathConstraints": [],
          "maxSpeed": 123
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "96CC6245E1E4661D",
          "point": {
            "x": 282092,
            "y": 233893,
            "angle": 0,
            "nodeId": "4462"
          },
          "estimatedDuration": 145,
          "pathConstraints": [],
          "maxSpeed": 120
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "A608AFF2F2AA98DF",
          "point": {
            "x": 237590,
            "y": 403489,
            "angle": 180,
            "nodeId": "3957"
          },
          "estimatedDuration": 91,
          "pathConstraints": [],
          "maxSpeed": 111
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_023",
      "priority": 6,
      "expectedStartTime": "2025-11-19T13:31:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T13:46:02+00:00Z",
      "minBatteryLevel": 39,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BF5C0F51A60D486B",
          "point": {
            "x": 226400,
            "y": 219175,
            "angle": 0,
            "nodeId": "1800"
          },
          "estimatedDuration": 118,
          "pathConstraints": [],
          "maxSpeed": 144
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "B833915DE3857318",
          "point": {
            "x": 224272,
            "y": 232723,
            "angle": 270,
            "nodeId": "3748"
          },
          "estimatedDuration": 41,
          "pathConstraints": [],
          "maxSpeed": 116
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "89A327195F95F48C",
          "point": {
            "x": 252997,
            "y": 273524,
            "angle": 90,
            "nodeId": "4892"
          },
          "estimatedDuration": 48,
          "pathConstraints": [],
          "maxSpeed": 108
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "A58842D1EBAAC527",
          "point": {
            "x": 199827,
            "y": 244376,
            "angle": 0,
            "nodeId": "3195"
          },
          "estimatedDuration": 91,
          "pathConstraints": [],
          "maxSpeed": 144
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_024",
      "priority": 8,
      "expectedStartTime": "2025-11-19T13:36:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:17:02+00:00Z",
      "minBatteryLevel": 36,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BDBA525B0AC93150",
          "point": {
            "x": 246225,
            "y": 289574,
            "angle": 90,
            "nodeId": "2137"
          },
          "estimatedDuration": 129,
          "pathConstraints": [],
          "maxSpeed": 110
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "B8996D7DCB60AE2B",
          "point": {
            "x": 261460,
            "y": 230747,
            "angle": 90,
            "nodeId": "2002"
          },
          "estimatedDuration": 64,
          "pathConstraints": [],
          "maxSpeed": 101
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "AB25820447D9CAF5",
          "point": {
            "x": 213200,
            "y": 216800,
            "angle": 270,
            "nodeId": "1109"
          },
          "estimatedDuration": 130,
          "pathConstraints": [],
          "maxSpeed": 119
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "9B05EC12838830B2",
          "point": {
            "x": 197515,
            "y": 238354,
            "angle": 180,
            "nodeId": "3231"
          },
          "estimatedDuration": 67,
          "pathConstraints": [],
          "maxSpeed": 104
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_025",
      "priority": 5,
      "expectedStartTime": "2025-11-19T13:41:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:19:02+00:00Z",
      "minBatteryLevel": 31,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "93303F85E000A50",
          "point": {
            "x": 177460,
            "y": 221840,
            "angle": 0,
            "nodeId": "10"
          },
          "estimatedDuration": 144,
          "pathConstraints": [],
          "maxSpeed": 129
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "AE73389CE5591DA8",
          "point": {
            "x": 166840,
            "y": 229845,
            "angle": 0,
            "nodeId": "347"
          },
          "estimatedDuration": 54,
          "pathConstraints": [],
          "maxSpeed": 150
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "AC2591E51ADF10F",
          "point": {
            "x": 170990,
            "y": 229845,
            "angle": 90,
            "nodeId": "341"
          },
          "estimatedDuration": 144,
          "pathConstraints": [],
          "maxSpeed": 134
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_026",
      "priority": 8,
      "expectedStartTime": "2025-11-19T13:46:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:34:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BAD3E6A04DFCE7D7",
          "point": {
            "x": 171640,
            "y": 225340,
            "angle": 270,
            "nodeId": "113"
          },
          "estimatedDuration": 116,
          "pathConstraints": [],
          "maxSpeed": 140
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "B6F25F1F7E97DA7C",
          "point": {
            "x": 157570,
            "y": 231345,
            "angle": 270,
            "nodeId": "1757"
          },
          "estimatedDuration": 89,
          "pathConstraints": [],
          "maxSpeed": 117
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_027",
      "priority": 3,
      "expectedStartTime": "2025-11-19T13:51:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:37:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "85A452DC2AFEAFD6",
          "point": {
            "x": 255000,
            "y": 314066,
            "angle": 0,
            "nodeId": "1307"
          },
          "estimatedDuration": 76,
          "pathConstraints": [],
          "maxSpeed": 103
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "9233C3A88A03E289",
          "point": {
            "x": 196310,
            "y": 266557,
            "angle": 0,
            "nodeId": "4442"
          },
          "estimatedDuration": 40,
          "pathConstraints": [],
          "maxSpeed": 113
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B38749288EA821F2",
          "point": {
            "x": 204800,
            "y": 207200,
            "angle": 270,
            "nodeId": "769"
          },
          "estimatedDuration": 88,
          "pathConstraints": [],
          "maxSpeed": 130
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_028",
      "priority": 4,
      "expectedStartTime": "2025-11-19T13:56:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:46:02+00:00Z",
      "minBatteryLevel": 31,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "9AF8B04EBB490F63",
          "point": {
            "x": 226847,
            "y": 308074,
            "angle": 0,
            "nodeId": "2498"
          },
          "estimatedDuration": 124,
          "pathConstraints": [],
          "maxSpeed": 144
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "BCCA6DCE53866121",
          "point": {
            "x": 286489,
            "y": 326753,
            "angle": 90,
            "nodeId": "1541"
          },
          "estimatedDuration": 99,
          "pathConstraints": [],
          "maxSpeed": 106
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "A129CFA412DEA44F",
          "point": {
            "x": 225240,
            "y": 249720,
            "angle": 180,
            "nodeId": "2538"
          },
          "estimatedDuration": 115,
          "pathConstraints": [],
          "maxSpeed": 132
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "9468F8BFCD23CB7F",
          "point": {
            "x": 265855,
            "y": 318572,
            "angle": 90,
            "nodeId": "2854"
          },
          "estimatedDuration": 133,
          "pathConstraints": [],
          "maxSpeed": 101
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_029",
      "priority": 2,
      "expectedStartTime": "2025-11-19T14:01:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:15:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "A429F92D26EBBAC4",
          "point": {
            "x": 258000,
            "y": 306866,
            "angle": 180,
            "nodeId": "1909"
          },
          "estimatedDuration": 61,
          "pathConstraints": [],
          "maxSpeed": 120
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "955CA04D7D0F8C7E",
          "point": {
            "x": 243200,
            "y": 213200,
            "angle": 180,
            "nodeId": "1007"
          },
          "estimatedDuration": 138,
          "pathConstraints": [],
          "maxSpeed": 102
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "A7C81684491ABEAB",
          "point": {
            "x": 204800,
            "y": 214400,
            "angle": 0,
            "nodeId": "1017"
          },
          "estimatedDuration": 88,
          "pathConstraints": [],
          "maxSpeed": 114
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_030",
      "priority": 7,
      "expectedStartTime": "2025-11-19T14:06:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:53:02+00:00Z",
      "minBatteryLevel": 32,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "AB262C6FBC76FDBA",
          "point": {
            "x": 166340,
            "y": 221840,
            "angle": 270,
            "nodeId": "27"
          },
          "estimatedDuration": 103,
          "pathConstraints": [],
          "maxSpeed": 149
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "BC4098F15EBCB341",
          "point": {
            "x": 176960,
            "y": 229845,
            "angle": 180,
            "nodeId": "330"
          },
          "estimatedDuration": 98,
          "pathConstraints": [],
          "maxSpeed": 132
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B606620057416A28",
          "point": {
            "x": 157570,
            "y": 224840,
            "angle": 270,
            "nodeId": "461"
          },
          "estimatedDuration": 122,
          "pathConstraints": [],
          "maxSpeed": 145
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_031",
      "priority": 5,
      "expectedStartTime": "2025-11-19T14:11:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:55:02+00:00Z",
      "minBatteryLevel": 39,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "9C2BDEED35879CCE",
          "point": {
            "x": 259165,
            "y": 207555,
            "angle": 270,
            "nodeId": "3081"
          },
          "estimatedDuration": 75,
          "pathConstraints": [],
          "maxSpeed": 107
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_032",
      "priority": 3,
      "expectedStartTime": "2025-11-19T14:16:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:49:02+00:00Z",
      "minBatteryLevel": 30,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "B51FADFF2EBAFA1B",
          "point": {
            "x": 256500,
            "y": 298466,
            "angle": 270,
            "nodeId": "1282"
          },
          "estimatedDuration": 34,
          "pathConstraints": [],
          "maxSpeed": 127
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "99722C0C46DE24BA",
          "point": {
            "x": 205979,
            "y": 247417,
            "angle": 90,
            "nodeId": "3297"
          },
          "estimatedDuration": 61,
          "pathConstraints": [],
          "maxSpeed": 144
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B2F20555455C95A3",
          "point": {
            "x": 220034,
            "y": 371050,
            "angle": 180,
            "nodeId": "4748"
          },
          "estimatedDuration": 129,
          "pathConstraints": [],
          "maxSpeed": 140
        },
        {
          "sequence": 4,
          "pointType": "DROPOFF",
          "location": "9A4F73EAD0DBF1EB",
          "point": {
            "x": 231474,
            "y": 232723,
            "angle": 180,
            "nodeId": "4123"
          },
          "estimatedDuration": 31,
          "pathConstraints": [],
          "maxSpeed": 107
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_033",
      "priority": 7,
      "expectedStartTime": "2025-11-19T14:21:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:44:02+00:00Z",
      "minBatteryLevel": 37,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "B7C5EF700329E729",
          "point": {
            "x": 249645,
            "y": 239900,
            "angle": 180,
            "nodeId": "2360"
          },
          "estimatedDuration": 97,
          "pathConstraints": [],
          "maxSpeed": 133
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "89F2D5F80D4682DA",
          "point": {
            "x": 225447,
            "y": 312874,
            "angle": 180,
            "nodeId": "3900"
          },
          "estimatedDuration": 114,
          "pathConstraints": [],
          "maxSpeed": 100
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "A89867A05F2ED08A",
          "point": {
            "x": 277416,
            "y": 294569,
            "angle": 180,
            "nodeId": "2770"
          },
          "estimatedDuration": 50,
          "pathConstraints": [],
          "maxSpeed": 125
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "BDB629AE8BAE85BC",
          "point": {
            "x": 265800,
            "y": 276569,
            "angle": 90,
            "nodeId": "1437"
          },
          "estimatedDuration": 91,
          "pathConstraints": [],
          "maxSpeed": 150
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_034",
      "priority": 1,
      "expectedStartTime": "2025-11-19T14:26:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:57:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "96E621F32ED21074",
          "point": {
            "x": 284982,
            "y": 284735,
            "angle": 90,
            "nodeId": "3564"
          },
          "estimatedDuration": 133,
          "pathConstraints": [],
          "maxSpeed": 143
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_035",
      "priority": 7,
      "expectedStartTime": "2025-11-19T14:31:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:04:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "9D3145DD11B50378",
          "point": {
            "x": 179370,
            "y": 275057,
            "angle": 0,
            "nodeId": "4666"
          },
          "estimatedDuration": 36,
          "pathConstraints": [],
          "maxSpeed": 138
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "9DC1EB3DBBDFCA3F",
          "point": {
            "x": 238400,
            "y": 218000,
            "angle": 270,
            "nodeId": "1170"
          },
          "estimatedDuration": 72,
          "pathConstraints": [],
          "maxSpeed": 141
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_036",
      "priority": 1,
      "expectedStartTime": "2025-11-19T14:36:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:57:02+00:00Z",
      "minBatteryLevel": 39,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8EED8EDE26E9B44A",
          "point": {
            "x": 218734,
            "y": 372500,
            "angle": 0,
            "nodeId": "4750"
          },
          "estimatedDuration": 41,
          "pathConstraints": [],
          "maxSpeed": 103
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "8DA9DA1A488960E",
          "point": {
            "x": 255000,
            "y": 340466,
            "angle": 180,
            "nodeId": "1351"
          },
          "estimatedDuration": 38,
          "pathConstraints": [],
          "maxSpeed": 132
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "BFA87EF614F0733A",
          "point": {
            "x": 220082,
            "y": 276830,
            "angle": 90,
            "nodeId": "5011"
          },
          "estimatedDuration": 60,
          "pathConstraints": [],
          "maxSpeed": 125
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_037",
      "priority": 9,
      "expectedStartTime": "2025-11-19T14:41:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:41:02+00:00Z",
      "minBatteryLevel": 40,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8C641CE8D1238BF9",
          "point": {
            "x": 201200,
            "y": 204800,
            "angle": 90,
            "nodeId": "696"
          },
          "estimatedDuration": 77,
          "pathConstraints": [],
          "maxSpeed": 108
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_038",
      "priority": 3,
      "expectedStartTime": "2025-11-19T14:46:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T14:59:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8D2AC454AD7CE2D2",
          "point": {
            "x": 168990,
            "y": 221840,
            "angle": 0,
            "nodeId": "26"
          },
          "estimatedDuration": 112,
          "pathConstraints": [],
          "maxSpeed": 114
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "B54AEA5CD867CB49",
          "point": {
            "x": 169990,
            "y": 222340,
            "angle": 0,
            "nodeId": "59"
          },
          "estimatedDuration": 118,
          "pathConstraints": [],
          "maxSpeed": 105
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "8CE95A5AD6F92C54",
          "point": {
            "x": 157570,
            "y": 229845,
            "angle": 180,
            "nodeId": "466"
          },
          "estimatedDuration": 74,
          "pathConstraints": [],
          "maxSpeed": 128
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_039",
      "priority": 7,
      "expectedStartTime": "2025-11-19T14:51:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:43:02+00:00Z",
      "minBatteryLevel": 39,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "8998544A9B3BAC3C",
          "point": {
            "x": 250597,
            "y": 260924,
            "angle": 180,
            "nodeId": "4989"
          },
          "estimatedDuration": 94,
          "pathConstraints": [],
          "maxSpeed": 134
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "BB64E3E1F0E43C5F",
          "point": {
            "x": 249568,
            "y": 230607,
            "angle": 90,
            "nodeId": "3341"
          },
          "estimatedDuration": 48,
          "pathConstraints": [],
          "maxSpeed": 102
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_040",
      "priority": 7,
      "expectedStartTime": "2025-11-19T14:56:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:29:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "A5806C2954B9A6D3",
          "point": {
            "x": 284982,
            "y": 339953,
            "angle": 270,
            "nodeId": "1562"
          },
          "estimatedDuration": 45,
          "pathConstraints": [],
          "maxSpeed": 132
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "8AA284DA7C710DA0",
          "point": {
            "x": 226790,
            "y": 383000,
            "angle": 180,
            "nodeId": "4040"
          },
          "estimatedDuration": 89,
          "pathConstraints": [],
          "maxSpeed": 144
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "B4A25307ADCABF27",
          "point": {
            "x": 253475,
            "y": 278019,
            "angle": 270,
            "nodeId": "2156"
          },
          "estimatedDuration": 144,
          "pathConstraints": [],
          "maxSpeed": 131
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_041",
      "priority": 5,
      "expectedStartTime": "2025-11-19T15:01:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:31:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "A5F967FC99AC1D2B",
          "point": {
            "x": 191545,
            "y": 219545,
            "angle": 90,
            "nodeId": "4380"
          },
          "estimatedDuration": 120,
          "pathConstraints": [],
          "maxSpeed": 100
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "B5F3DBE33ABCFB7E",
          "point": {
            "x": 237012,
            "y": 301727,
            "angle": 0,
            "nodeId": "4790"
          },
          "estimatedDuration": 65,
          "pathConstraints": [],
          "maxSpeed": 129
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_042",
      "priority": 7,
      "expectedStartTime": "2025-11-19T15:06:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:26:02+00:00Z",
      "minBatteryLevel": 31,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "8D5B0674ECCD279E",
          "point": {
            "x": 194870,
            "y": 241371,
            "angle": 0,
            "nodeId": "5141"
          },
          "estimatedDuration": 61,
          "pathConstraints": [],
          "maxSpeed": 132
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "A6DA2FB382C288AE",
          "point": {
            "x": 248197,
            "y": 266524,
            "angle": 180,
            "nodeId": "4948"
          },
          "estimatedDuration": 99,
          "pathConstraints": [],
          "maxSpeed": 117
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_043",
      "priority": 3,
      "expectedStartTime": "2025-11-19T15:11:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:39:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "9D8BC5F8A2CFBFC7",
          "point": {
            "x": 212000,
            "y": 219675,
            "angle": 270,
            "nodeId": "1803"
          },
          "estimatedDuration": 137,
          "pathConstraints": [],
          "maxSpeed": 101
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "A48DDF54FDE11EFB",
          "point": {
            "x": 193763,
            "y": 230325,
            "angle": 270,
            "nodeId": "3065"
          },
          "estimatedDuration": 110,
          "pathConstraints": [],
          "maxSpeed": 121
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_044",
      "priority": 3,
      "expectedStartTime": "2025-11-19T15:16:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:32:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "B5955F746ED3DB8E",
          "point": {
            "x": 234447,
            "y": 327253,
            "angle": 180,
            "nodeId": "2953"
          },
          "estimatedDuration": 149,
          "pathConstraints": [],
          "maxSpeed": 123
        },
        {
          "sequence": 2,
          "pointType": "DROPOFF",
          "location": "8899FA13A7A6D628",
          "point": {
            "x": 230897,
            "y": 275074,
            "angle": 270,
            "nodeId": "3122"
          },
          "estimatedDuration": 64,
          "pathConstraints": [],
          "maxSpeed": 111
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "8F0CED4AD8E6A0F6",
          "point": {
            "x": 253510,
            "y": 342866,
            "angle": 0,
            "nodeId": "2988"
          },
          "estimatedDuration": 50,
          "pathConstraints": [],
          "maxSpeed": 135
        },
        {
          "sequence": 4,
          "pointType": "DROPOFF",
          "location": "B7647B1508EE6214",
          "point": {
            "x": 276216,
            "y": 271642,
            "angle": 0,
            "nodeId": "3731"
          },
          "estimatedDuration": 72,
          "pathConstraints": [],
          "maxSpeed": 131
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_045",
      "priority": 4,
      "expectedStartTime": "2025-11-19T15:21:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:37:02+00:00Z",
      "minBatteryLevel": 34,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "A74AF6943684C9FC",
          "point": {
            "x": 256500,
            "y": 259095,
            "angle": 0,
            "nodeId": "1230"
          },
          "estimatedDuration": 119,
          "pathConstraints": [],
          "maxSpeed": 141
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_046",
      "priority": 6,
      "expectedStartTime": "2025-11-19T15:26:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T16:08:02+00:00Z",
      "minBatteryLevel": 35,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "BD066D11FDABFFC9",
          "point": {
            "x": 284982,
            "y": 327953,
            "angle": 180,
            "nodeId": "1542"
          },
          "estimatedDuration": 107,
          "pathConstraints": [],
          "maxSpeed": 111
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "8F1DCB5EF89F1274",
          "point": {
            "x": 234447,
            "y": 325753,
            "angle": 270,
            "nodeId": "3710"
          },
          "estimatedDuration": 148,
          "pathConstraints": [],
          "maxSpeed": 148
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_047",
      "priority": 9,
      "expectedStartTime": "2025-11-19T15:31:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T15:49:02+00:00Z",
      "minBatteryLevel": 32,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "BBFFC5BD8476CD41",
          "point": {
            "x": 253475,
            "y": 314021,
            "angle": 180,
            "nodeId": "3463"
          },
          "estimatedDuration": 103,
          "pathConstraints": [],
          "maxSpeed": 112
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "980F39968358FCC",
          "point": {
            "x": 251998,
            "y": 200775,
            "angle": 270,
            "nodeId": "4886"
          },
          "estimatedDuration": 41,
          "pathConstraints": [],
          "maxSpeed": 137
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "95D667FEFE27C961",
          "point": {
            "x": 210800,
            "y": 216800,
            "angle": 270,
            "nodeId": "1107"
          },
          "estimatedDuration": 74,
          "pathConstraints": [],
          "maxSpeed": 125
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_048",
      "priority": 1,
      "expectedStartTime": "2025-11-19T15:36:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T16:00:02+00:00Z",
      "minBatteryLevel": 30,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "8DA48B788B424411",
          "point": {
            "x": 232197,
            "y": 260924,
            "angle": 0,
            "nodeId": "1651"
          },
          "estimatedDuration": 123,
          "pathConstraints": [],
          "maxSpeed": 114
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "9A54DC0B35BB4F90",
          "point": {
            "x": 196366,
            "y": 332624,
            "angle": 270,
            "nodeId": "4806"
          },
          "estimatedDuration": 44,
          "pathConstraints": [],
          "maxSpeed": 115
        },
        {
          "sequence": 3,
          "pointType": "PICKUP",
          "location": "A10975ED561CA794",
          "point": {
            "x": 225447,
            "y": 290318,
            "angle": 90,
            "nodeId": "4842"
          },
          "estimatedDuration": 83,
          "pathConstraints": [],
          "maxSpeed": 137
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_049",
      "priority": 8,
      "expectedStartTime": "2025-11-19T15:41:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T16:17:02+00:00Z",
      "minBatteryLevel": 31,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "PICKUP",
          "location": "AAEDBF240B9AFCEA",
          "point": {
            "x": 223777,
            "y": 250955,
            "angle": 90,
            "nodeId": "4076"
          },
          "estimatedDuration": 76,
          "pathConstraints": [],
          "maxSpeed": 116
        },
        {
          "sequence": 2,
          "pointType": "PICKUP",
          "location": "9448A96B8EAB695B",
          "point": {
            "x": 263325,
            "y": 327723,
            "angle": 180,
            "nodeId": "2574"
          },
          "estimatedDuration": 127,
          "pathConstraints": [],
          "maxSpeed": 148
        },
        {
          "sequence": 3,
          "pointType": "DROPOFF",
          "location": "BC15BD8B36C952F1",
          "point": {
            "x": 238200,
            "y": 276569,
            "angle": 270,
            "nodeId": "1394"
          },
          "estimatedDuration": 64,
          "pathConstraints": [],
          "maxSpeed": 118
        },
        {
          "sequence": 4,
          "pointType": "PICKUP",
          "location": "8DA9DA1A488960E",
          "point": {
            "x": 255000,
            "y": 340466,
            "angle": 180,
            "nodeId": "1351"
          },
          "estimatedDuration": 74,
          "pathConstraints": [],
          "maxSpeed": 113
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    },
    {
      "taskId": "TASK_050",
      "priority": 5,
      "expectedStartTime": "2025-11-19T15:46:02+00:00Z",
      "expectedCompletionTime": "2025-11-19T16:13:02+00:00Z",
      "minBatteryLevel": 33,
      "subTasks": [
        {
          "sequence": 1,
          "pointType": "DROPOFF",
          "location": "A1BC224923A00045",
          "point": {
            "x": 200000,
            "y": 215600,
            "angle": 270,
            "nodeId": "1055"
          },
          "estimatedDuration": 108,
          "pathConstraints": [],
          "maxSpeed": 140
        }
      ],
      "agvRequirements": {
        "vehicleTypes": []
      }
    }
  ],
  "agvStatusList": [
    {
      "deviceId": "AGV01",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 280036,
      "y": 293562,
      "angle": 270,
      "speed": 0,
      "nodeId": "2706",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 82,
      "endurance": 140,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV02",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 236000,
      "y": 219175,
      "angle": 0,
      "speed": 0,
      "nodeId": "1802",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 77,
      "endurance": 131,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV03",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 217064,
      "y": 273374,
      "angle": 90,
      "speed": 0,
      "nodeId": "3116",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 91,
      "endurance": 95,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV04",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 245847,
      "y": 334800,
      "angle": 180,
      "speed": 0,
      "nodeId": "4568",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 93,
      "endurance": 114,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV05",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 195000,
      "y": 227725,
      "angle": 270,
      "speed": 0,
      "nodeId": "1767",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 72,
      "endurance": 90,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV06",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 234797,
      "y": 258124,
      "angle": 0,
      "speed": 0,
      "nodeId": "1637",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 92,
      "endurance": 144,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV07",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 253475,
      "y": 316421,
      "angle": 270,
      "speed": 0,
      "nodeId": "3465",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 81,
      "endurance": 118,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV08",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 255000,
      "y": 268695,
      "angle": 0,
      "speed": 0,
      "nodeId": "1205",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 71,
      "endurance": 135,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV09",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 200713,
      "y": 262472,
      "angle": 270,
      "speed": 0,
      "nodeId": "3279",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 71,
      "endurance": 136,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV10",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 268182,
      "y": 322873,
      "angle": 270,
      "speed": 0,
      "nodeId": "1597",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 95,
      "endurance": 143,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV11",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 242000,
      "y": 213200,
      "angle": 270,
      "speed": 0,
      "nodeId": "1006",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 89,
      "endurance": 101,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV12",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 240475,
      "y": 323600,
      "angle": 0,
      "speed": 0,
      "nodeId": "3585",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 98,
      "endurance": 150,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV13",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 188515,
      "y": 276582,
      "angle": 0,
      "speed": 0,
      "nodeId": "4049",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 78,
      "endurance": 124,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV14",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 164180,
      "y": 227840,
      "angle": 270,
      "speed": 0,
      "nodeId": "227",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 94,
      "endurance": 134,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV15",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 230323,
      "y": 223585,
      "angle": 180,
      "speed": 0,
      "nodeId": "4133",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 73,
      "endurance": 91,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV16",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 264505,
      "y": 297233,
      "angle": 270,
      "speed": 0,
      "nodeId": "2412",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 91,
      "endurance": 120,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV17",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 255000,
      "y": 348866,
      "angle": 180,
      "speed": 0,
      "nodeId": "1365",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 92,
      "endurance": 103,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV18",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 193310,
      "y": 315591,
      "angle": 90,
      "speed": 0,
      "nodeId": "587",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 77,
      "endurance": 148,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV19",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 234447,
      "y": 312927,
      "angle": 0,
      "speed": 0,
      "nodeId": "2911",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 91,
      "endurance": 121,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    },
    {
      "deviceId": "AGV20",
      "mapId": 0,
      "curArea": "",
      "connection": true,
      "x": 228520,
      "y": 234873,
      "angle": 0,
      "speed": 0,
      "nodeId": "2207",
      "nextDestinationPoint": {},
      "curTrailPoints": [],
      "taskId": "",
      "taskStatus": 0,
      "taskProgress": 0,
      "estimatedDuration": 0,
      "batteryLevel": 87,
      "endurance": 144,
      "load": false,
      "errorCode": 0,
      "updateTime": 1763552462
    }
  ]
})JSON";

    auto ensureSubTaskIds = [](nlohmann::json& payload) {
        if (!payload.contains("candidateTasks") || !payload["candidateTasks"].is_array()) return;
        for (auto& task : payload["candidateTasks"]) {
            std::string taskId;
            try {
                if (task.contains("taskId")) taskId = task["taskId"].get<std::string>();
            } catch (...) {
                taskId.clear();
            }
            if (!task.contains("subTasks") || !task["subTasks"].is_array()) continue;
            auto& subs = task["subTasks"];
            for (size_t idx = 0; idx < subs.size(); ++idx) {
                auto& st = subs[idx];
                int seq = -1;
                try {
                    if (st.contains("sequence")) seq = st["sequence"].get<int>();
                } catch (...) {
                    seq = -1;
                }
                if (seq <= 0) {
                    seq = static_cast<int>(idx + 1);
                    st["sequence"] = seq;
                }
                std::string subId;
                try {
                    if (st.contains("subTaskId")) subId = st["subTaskId"].get<std::string>();
                } catch (...) {
                    subId.clear();
                }
                if (subId.empty()) {
                    if (!taskId.empty()) subId = taskId + "#" + std::to_string(seq);
                    else subId = std::string("SUBTASK_") + std::to_string(seq);
                    st["subTaskId"] = subId;
                }
            }
        }
    };

    // 尝试从文件加载数据，否则使用默认数据
    std::string data;
    std::string payloadFile = "script/generated_test_payload1.json";
    auto searchRoots = PathUtils::commonRoots();
    PathUtils::addRootIfSet(searchRoots, "AGV_SCHED_ROOT");
    auto resolvedPath = PathUtils::resolveExistingPath(payloadFile, searchRoots);
    
    if (resolvedPath) {
        std::ifstream file(*resolvedPath);
        if (file.is_open()) {
            data.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            file.close();
            std::cout << "ExternalSender: loaded payload from file: " << *resolvedPath << " (size=" << data.size() << " bytes)" << std::endl;
        } else {
            std::cerr << "ExternalSender: warning: cannot open file " << *resolvedPath << ", using default data" << std::endl;
            data = defaultData;
        }
    } else {
        std::cout << "ExternalSender: file " << payloadFile << " not found, using default data" << std::endl;
        data = defaultData;
    }

    try {
        auto payloadJson = nlohmann::json::parse(data);
        ensureSubTaskIds(payloadJson);
        data = payloadJson.dump();
    } catch (const std::exception& e) {
        std::cerr << "ExternalSender: warning: payload JSON parse failed (" << e.what() << "), sending raw data" << std::endl;
    }

    std::cout << "ExternalSender: publish target exchange='" << exch
              << "' queue='" << queue << "' rkey='" << rkey
              << "' host=" << cfg.host << ":" << cfg.port
              << " vhost=" << cfg.vhost << std::endl;

    ExternalSender::PublishOptions opt;
    opt.exchange = exch;
    opt.exchangeType = exchType;
    opt.queue = queue;
    opt.routingKey = rkey;
    opt.bindingKey = bindingKey.empty() ? "#" : bindingKey;
    opt.messageTtlMs = messageTtlMs;
    bool ok = ExternalSender::PublishPayload(cfg, opt, data);
    return ok ? 0 : 2;
}
