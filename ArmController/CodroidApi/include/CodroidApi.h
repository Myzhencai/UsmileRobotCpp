#pragma once

#include "Request.h"

namespace c2 {

class CodroidApi {
public:
    CodroidApi(const std::string& host, const std::string& port) : _request(host, port) {
    }

    ~CodroidApi() {
    }

    /**
     * 发送用户命令.
     *
     * \param cmd 参考UserCommand说明
     * \param timeout 超时等待时间
     * \return
     */
    Response sendUserCommand(UserCommand cmd, int timeout = 30) {
        json param     = json::array();
        json paramitem = {
            {"path",  "Robot/Control/command"},
            {"value", (int)cmd               }
        };
        param.push_back(paramitem);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取机器人状态.
     *
     * \param timeout
     * \return Response.data 是整数json，可通过 int state = res.data; 拿到数值，
     * 数值定义参考Define.h文件中 enum class RobotState
     * None    = -1,  // 未知
     * Init    = 0,   // 初始化
     * StandBy = 1,   // 已下电
     * Ready   = 2,   // 手动模式
     * Rescue  = 3,   // 救援模式
     * Auto    = 4,   // 自动模式
     * Error   = 6,   // 机器人出错
     */
    Response getRobotState(int timeout = 5) {
        std::string path   = "Robot/Control/state";
        json        param  = {path};
        auto        future = _request.send("common", "getparam", param, timeout);
        Response    res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"][path];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取打包位.
     *
     * \param timeout
     * \return Response.data 是json数组，double[6] 各关节旋转角度，单位度（deg）
     */
    Response getPackPosition(int timeout = 5) {
        std::string path   = "Robot/Parameter/Mechanism/packingPosition";
        json        param  = {path};
        auto        future = _request.send("common", "getparam", param, timeout);
        Response    res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"][path];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 通过MovJonit运动到Home位.
     *
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    Response goHome(double speed = 60, double acc = 80, int timeout = 30) {
        Point point;
        point.apos.jntPos[0] = _homePosition[0];
        point.apos.jntPos[1] = _homePosition[1];
        point.apos.jntPos[2] = _homePosition[2];
        point.apos.jntPos[3] = _homePosition[3];
        point.apos.jntPos[4] = _homePosition[4];
        point.apos.jntPos[5] = _homePosition[5];

        return movJ(point, speed, acc, timeout);
    }

    /**
     * 设置Home位置.
     *
     * \param position double[6] 各关节旋转角度，单位度（deg）
     */
    void setHomePosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy_s(_homePosition, size, position, size);
    }

    /**
     * 通过MovJonit运动到打包位.
     *
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    Response goPack(double speed = 60, double acc = 80, int timeout = 30) {
        Point point;
        point.apos.jntPos[0] = _packPosition[0];
        point.apos.jntPos[1] = _packPosition[1];
        point.apos.jntPos[2] = _packPosition[2];
        point.apos.jntPos[3] = _packPosition[3];
        point.apos.jntPos[4] = _packPosition[4];
        point.apos.jntPos[5] = _packPosition[5];

        return movJ(point, speed, acc, timeout);
    }

    /**
     * 设置打包位置.
     *
     * \param position double[6] 各关节旋转角度，单位度（deg）
     */
    void setPackPosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy_s(_packPosition, size, position, size);
    }

    /**
     * 关节空间运动.
     *
     * \param position 目标位置，类型为double[6]，数组分别为轴1~轴6的旋转角度，单位度（deg），例如[0,0,90,0,90,0]
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    Response movJ(const Point& position, double speed = 60, double acc = 80, int timeout = 30) {
        json param              = json::object();
        param["type"]           = "movj";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  speed},
            {"stcp",  0    },
            {"sori",  0    },
            {"sexjl", 0    },
            {"sexjr", 0    }
        };

        param["acc"] = {
            {"aper",  acc},
            {"atcp",  0  },
            {"aori",  0  },
            {"aexjl", 0  },
            {"aexjr", 0  }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 多段关节空间运动.
     *
     * \param segements 多段路径
     * \param timeout 超时等待时间
     * \return
     */
    Response movJointSegments(const MovJointSegments& segments, int timeout = 30) {
        json param      = json::object();
        param["type"]   = "movJoint";
        param["points"] = json::array();

        for (auto& segment : segments.segments) {
            param["points"].emplace_back();
            json& data = param["points"].back();

            switch (segment.targetPosition.type) {
                case PointType::Joint:
                    data["target"]["type"] = "apos";
                    APos::toJson(data["target"]["apos"], &segment.targetPosition.apos);
                    break;

                case PointType::Cart:
                    data["target"]["type"] = "cpos";
                    CPos::toJson(data["target"]["cpos"], &segment.targetPosition.cpos);
                    break;

                default:
                    break;
            }

            data["speed"] = {
                {"sper",  segment.speed.joint},
                {"stcp",  0                  },
                {"sori",  0                  },
                {"sexjl", 0                  },
                {"sexjr", 0                  }
            };

            data["acc"] = {
                {"aper",  segment.acc.joint},
                {"atcp",  0                },
                {"aori",  0                },
                {"aexjl", 0                },
                {"aexjr", 0                }
            };

            switch (segment.zoneType) {
                case ZoneType::Fine:
                    data["zone"]["type"] = "FINE";
                    break;

                case ZoneType::Relative:
                    data["zone"]["type"] = "FINE";
                    break;

                default:
                    throw std::string("Zontype error");
                    break;
            }

            data["zone"]["data"] = {
                {"zper",    segment.zone.per},
                {"zdis",    segment.zone.dis},
                {"zvconst", 0               }
            };
        }

        auto     future = _request.send("common", "movMulti", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 关节空间运动，不等待返回.
     *
     * \param position 目标位置，类型为double[6]，数组分别为轴1~轴6的旋转角度，单位度（deg），例如[0,0,90,0,90,0]
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    void movJNoResult(const Point& position, double speed = 60, double acc = 80, int timeout = 30) {
        json param              = json::object();
        param["type"]           = "movj";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  speed},
            {"stcp",  0    },
            {"sori",  0    },
            {"sexjl", 0    },
            {"sexjr", 0    }
        };

        param["acc"] = {
            {"aper",  acc},
            {"atcp",  0  },
            {"aori",  0  },
            {"aexjl", 0  },
            {"aexjr", 0  }
        };

        _request.send("common", "mov", param, timeout);
    }

    /**
     * 直线运动.
     *
     * \param position 目标位置，类型为double[6]，表示[x,y,z,rx,ry,rz]，
     *      (x,y,z)表示笛卡尔空间的位置，单位毫米（mm），（rx,ry,rz）表示绕3个坐标轴的旋转角度，单位度（deg）
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    Response movL(const Point& position, Speed speed = {0, 250, 80}, Acc acc = {0, 1200, 320}, int timeout = 30) {
        json param              = json::object();
        param["type"]           = "movl";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  0        },
            {"stcp",  speed.tcp},
            {"sori",  speed.ori},
            {"sexjl", 0        },
            {"sexjr", 0        }
        };

        param["acc"] = {
            {"aper",  0      },
            {"atcp",  acc.tcp},
            {"aori",  acc.ori},
            {"aexjl", 0      },
            {"aexjr", 0      }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 圆弧运动.
     *
     * \param targetPosition 目标位置，类型为double[6]，表示[x,y,z,rx,ry,rz]，
     *      (x,y,z)表示笛卡尔空间的位置，单位毫米（mm），（rx,ry,rz）表示绕3个坐标轴的旋转角度，单位度（deg）
     * \param middlePosition 中间位置，数据类型同目标位置
     * \param speed 运动速度，单位度/秒
     * \param acc 运动加速度，单位度/秒平方
     * \param timeout 超时等待时间
     * \return
     */
    Response movC(const Point& middlePosition,
                  const Point& targetPosition,
                  Speed   speed   = {0, 250, 80},
                  Acc     acc     = {0, 1200, 320},
                  int     timeout = 30) {
        json param              = json::object();
        param["type"]           = "movc";

        switch (targetPosition.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &targetPosition.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &targetPosition.cpos);
                break;

            default:
                break;
        }

        switch (middlePosition.type) {
            case PointType::Joint:
                param["middle"]["type"] = "apos";
                APos::toJson(param["middle"]["apos"], &middlePosition.apos);
                break;

            case PointType::Cart:
                param["middle"]["type"] = "cpos";
                CPos::toJson(param["middle"]["cpos"], &middlePosition.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  0        },
            {"stcp",  speed.tcp},
            {"sori",  speed.ori},
            {"sexjl", 0        },
            {"sexjr", 0        }
        };

        param["acc"] = {
            {"aper",  0      },
            {"atcp",  acc.tcp},
            {"aori",  acc.ori},
            {"aexjl", 0      },
            {"aexjr", 0      }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 多段笛卡尔空间运动.
     *
     * \param segements 多段路径
     * \param timeout 超时等待时间
     * \return
     */
    Response movCartSegments(const MovCartSegments& segments, int timeout = 30) {
        json param      = json::object();
        param["type"]   = "movCart";
        param["points"] = json::array();

        for (auto& segment : segments.segments) {
            // std::cout << "type: " << (int)segment.type << std::endl;
            // std::cout << "ZoneType: " << (int)segment.zoneType << std::endl;

            param["points"].emplace_back();
            json& data = param["points"].back();

            switch (segment.type) {
                case MovType::MovL:
                    data["type"] = "movl";
                    break;

                case MovType::MovC:
                    data["type"] = "movc";
                    break;

                case MovType::MovCircle:
                    data["type"] = "movcircle";
                    break;
                default:
                    throw std::string("MovType error");
                    break;
            }

            switch (segment.targetPosition.type) {
                case PointType::Joint:
                    data["target"]["type"] = "apos";
                    APos::toJson(data["target"]["apos"], &segment.targetPosition.apos);
                    break;

                case PointType::Cart:
                    data["target"]["type"] = "cpos";
                    CPos::toJson(data["target"]["cpos"], &segment.targetPosition.cpos);
                    break;

                default:
                    break;
            }

            if ((segment.type == MovType::MovC) || (segment.type == MovType::MovCircle)) {
                switch (segment.middlePosition.type) {
                    case PointType::Joint:
                        data["middle"]["type"] = "apos";
                        APos::toJson(data["target"]["apos"], &segment.middlePosition.apos);
                        break;

                    case PointType::Cart:
                        data["middle"]["type"] = "cpos";
                        CPos::toJson(data["target"]["cpos"], &segment.middlePosition.cpos);
                        break;

                    default:
                        break;
                }
            }

            data["speed"] = {
                {"sper",  segment.speed.joint},
                {"stcp",  segment.speed.tcp  },
                {"sori",  segment.speed.ori  },
                {"sexjl", 0                  },
                {"sexjr", 0                  }
            };

            data["acc"] = {
                {"aper",  segment.acc.joint},
                {"atcp",  segment.acc.tcp  },
                {"aori",  segment.acc.ori  },
                {"aexjl", 0                },
                {"aexjr", 0                }
            };

            switch (segment.zoneType) {
                case ZoneType::Fine:
                    data["zone"]["type"] = "FINE";
                    break;

                case ZoneType::Relative:
                    data["zone"]["type"] = "RELATIVE";
                    break;

                case ZoneType::Absolute:
                    data["zone"]["type"] = "ABSOLUTE";
                    break;

                default:
                    throw std::string("Zontype error");
                    break;
            }

            data["zone"]["data"] = {
                {"zper",    segment.zone.per},
                {"zdis",    segment.zone.dis},
                {"zvconst", 0               }
            };
        }

        auto     future = _request.send("common", "movMulti", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 让机器人停止运动.
     *
     * \param timeout
     * \return
     */
    Response stopMov(int timeout = 30) {
        auto     future = _request.send("common", "stopMov", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 关节点动. 该请求发送后，如果要保持点动持续，需要持续调keepJog接口，否则点动持续1s后会自动停止
     *
     * \param jogIndex 需要点动的轴，范围1~6
     * \param jogSpeed 点动速度
     * \param direction 点动方向
     * \param timeout 超时等待时间
     * \return
     */
    Response jointJog(int jogIndex, JogSpeed jogSpeed, Direction direction, int timeout = 30) {
        json param = json::array();
        json mode  = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 1                      }
        };
        param.push_back(mode);
        json speed = {
            {"path",  "Robot/Control/jogSpeed"      },
            {"value", (int)direction * (int)jogSpeed}
        };
        param.push_back(speed);
        json index = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", jogIndex                }
        };
        param.push_back(index);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 末端点动. 该请求发送后，如果要保持点动持续，需要持续调keepJog接口，否则点动持续1s后会自动停止
     *
     * \param jogIndex 1~6 分别对应 x,y,z,rx,ry,rz
     * \param jogSpeed 点动速度
     * \param direction 点动方向
     * \param timeout 超时等待时间
     * \return
     */
    Response tcpJog(int jogIndex, JogSpeed jogSpeed, Direction direction, int timeout = 30) {
        json param = json::array();
        json mode  = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 2                      }
        };
        param.push_back(mode);
        json speed = {
            {"path",  "Robot/Control/jogSpeed"      },
            {"value", (int)direction * (int)jogSpeed}
        };
        param.push_back(speed);
        json index = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", jogIndex                }
        };
        param.push_back(index);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 保持当前点动. 以500毫秒左右的频率持续调该接口，可保持点动持续
     *
     * \param timeout
     * \return
     */
    Response keepJog(int timeout = 30) {
        json param     = json::array();
        json paramitem = {
            {"path",  "Robot/Control/commandHeart"},
            {"value",
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                 .count()                         }
        };
        param.push_back(paramitem);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 停止点动
     *
     * \param timeout
     * \return
     */
    Response stopJog(int timeout = 30) {
        json param   = json::array();
        json jogMode = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 0                      }
        };
        param.push_back(jogMode);
        json jogSpeed = {
            {"path",  "Robot/Control/jogSpeed"},
            {"value", 0                       }
        };
        param.push_back(jogSpeed);
        json jogIndex = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", 0                       }
        };
        param.push_back(jogIndex);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取当前关节位置
     *
     * \param timeout
     * \return Response.data是长度为6的json数组，表示1~6关节的旋转角度
     * 可通过 vector<double> p = res.data 获得，
     * 或double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response getJointPosition(int timeout = 30) {
        auto     future = _request.send("common", "getCurAPos", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {
                    data["data"]["jntpos1"].get<double>(),
                    data["data"]["jntpos2"].get<double>(),
                    data["data"]["jntpos3"].get<double>(),
                    data["data"]["jntpos4"].get<double>(),
                    data["data"]["jntpos5"].get<double>(),
                    data["data"]["jntpos6"].get<double>(),
                };
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取当前笛卡尔坐标位置
     *
     * \param timeout
     * \return Response.data是长度为6的json数组，[x,y,z,rx,ry,rz]
     * 可通过 vector<double> p = res.data 获得，
     * 或double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response getCartPosition(int timeout = 30) {
        auto     future = _request.send("common", "getCurCPos", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {data["data"]["x"].get<double>(),
                            data["data"]["y"].get<double>(),
                            data["data"]["z"].get<double>(),
                            data["data"]["a"].get<double>(),
                            data["data"]["b"].get<double>(),
                            data["data"]["c"].get<double>()};
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 根据笛卡尔位置获取关节角
     *
     * \param cpos 需要逆解的笛卡尔坐标
     * \param refAPos 执行逆解时的参考角
     * \param timeout
     * \return Response.data是长度为6的json数组，表示1~6关节的旋转角度
     * 可通过 vector<double> p = res.data 获得，
     * 或double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response cposToAPos(const CPos& cpos, const APos refAPos, Tool* tool = nullptr, UserCoor* coor = nullptr,  int timeout = 30) {
        json param = json::object();

        CPos::toJson(param["cpos"], &cpos);
        APos::toJson(param["apos"], &refAPos);

        auto     future = _request.send("common", "cpostoapos", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {
                    data["data"]["apos"]["jntpos1"].get<double>(),
                    data["data"]["apos"]["jntpos2"].get<double>(),
                    data["data"]["apos"]["jntpos3"].get<double>(),
                    data["data"]["apos"]["jntpos4"].get<double>(),
                    data["data"]["apos"]["jntpos5"].get<double>(),
                    data["data"]["apos"]["jntpos6"].get<double>(),
                    data["data"]["apos"]["jntpos7"].get<double>(),
                };
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 设置数字输出端口值.
     *
     * \param port 端口号
     * \param val 端口值，只能是0或1
     * \param timeout 超市等待时间
     * \return
     */
    Response setDO(int port, int val, int timeout = 30) {
        json param = {
            {"port", port},
            {"val",  val }
        };

        auto     future = _request.send("common", "setDO", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取数字输入端口值.
     *
     * \param port 端口号
     * \param timeout
     * \return Response.data 是整数json，可通过 int val = res.data; 拿到数值,值为0或1
     */
    Response getDI(int port, int timeout = 30) {
        json param = {
            {"port", port}
        };
        auto     future = _request.send("common", "getDI", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 设置数字输出端口值.
     *
     * \param port 端口号
     * \param val 端口值，只能是0或1
     * \param timeout 超市等待时间
     * \return
     */
    Response setDOGroup(int startPort, int endPort, int val, int timeout = 30) {
        json param = {
            {"startPort", startPort},
            {"endPort",   endPort  },
            {"val",       val      }
        };

        auto     future = _request.send("common", "setDOGroup", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取数字输入端口值.
     *
     * \param port 端口号
     * \param timeout
     * \return Response.data 是整数json，可通过 int val = res.data; 拿到数值,值为端口从低到高对应的二进制数的十进制值
     */
    Response getDIGroup(int startPort, int endPort, int timeout = 30) {
        json param = {
            {"startPort", startPort},
            {"endPort",   endPort  }
        };
        auto     future = _request.send("common", "getDIGroup", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 设置当前坐标系.
     *
     * \param varName 坐标系变量名
     *      注: 该变量名用于在UI界面显示，该变量最好是已有变量，且参数与与下面下发的保持一致，以避免造成一些错误
     * \param coor 坐标系参数
     * \param timeout
     * \return
     */
    Response setCurrentCoor(const std::string& varName, UserCoor coor, int timeout = 30) {
        json param;

        param["key"]   = varName;
        json& coorNode = param["value"]["USERCOOR"];
        coorNode["x"]  = coor.x;
        coorNode["y"]  = coor.y;
        coorNode["z"]  = coor.z;
        coorNode["a"]  = coor.a;
        coorNode["b"]  = coor.b;
        coorNode["c"]  = coor.c;

        auto     future = _request.send("common", "setcurusercoor", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 设置当前工具.
     *
     * \param varName 工具变量名
     *      注: 该变量名用于在UI界面显示，该变量最好是已有变量，且参数与与下面下发的保持一致，以避免造成一些错误
     * \param tool 工具参数
     * \param timeout
     * \return
     */
    Response setCurrentTool(const std::string& varName, Tool tool, int timeout = 30) {
        json param;

        param["key"]   = varName;
        json& toolNode = param["value"]["TOOL"];
        toolNode["x"]  = tool.x;
        toolNode["y"]  = tool.y;
        toolNode["z"]  = tool.z;
        toolNode["a"]  = tool.a;
        toolNode["b"]  = tool.b;
        toolNode["c"]  = tool.c;

        LoadDyn& dyn     = tool.dyn;
        json&    dynNode = toolNode["dyn"];
        dynNode["m"]     = dyn.M;

        json& posNode = dynNode["pos"];
        posNode["mx"] = dyn.pos.Mx;
        posNode["my"] = dyn.pos.My;
        posNode["mz"] = dyn.pos.Mz;

        json& tensorNode  = dynNode["tensor"];
        tensorNode["ixx"] = dyn.it.Ixx;
        tensorNode["ixy"] = dyn.it.Ixy;
        tensorNode["ixz"] = dyn.it.Ixz;
        tensorNode["iyy"] = dyn.it.Iyy;
        tensorNode["iyz"] = dyn.it.Iyz;
        tensorNode["izz"] = dyn.it.Izz;

        auto     future = _request.send("common", "setcurtool", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 设置当前负载.
     *
     * \param varName 负载变量名
     *      注: 该变量名用于在UI界面显示，该变量最好是已有变量，且参数与与下面下发的保持一致，以避免造成一些错误
     * \param dyn 负载参数
     * \param timeout
     * \return
     */
    Response setCurrentPayload(const std::string& varName, LoadDyn dyn, int timeout = 30) {
        json param;

        param["key"]  = varName;
        json& dynNode = param["value"]["PAYLOAD"]["dyn"];
        dynNode["m"]  = dyn.M;

        json& posNode = dynNode["pos"];
        posNode["mx"] = dyn.pos.Mx;
        posNode["my"] = dyn.pos.My;
        posNode["mz"] = dyn.pos.Mz;

        json& tensorNode  = dynNode["tensor"];
        tensorNode["ixx"] = dyn.it.Ixx;
        tensorNode["ixy"] = dyn.it.Ixy;
        tensorNode["ixz"] = dyn.it.Ixz;
        tensorNode["iyy"] = dyn.it.Iyy;
        tensorNode["iyz"] = dyn.it.Iyz;
        tensorNode["izz"] = dyn.it.Izz;

        auto     future = _request.send("common", "setcurpayload", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 获取工程状态
     *
     * \param projectName 工程名（避免使用中文字符）
     * \param taskName 程序名（避免使用中文字符）
     * \param label 标签名（避免使用中文字符）
     * \param timeout
     * \return std::string
     * IDLE: 空闲，没有工程在运行
     * LOADING: 正在加载工程
     * RUNNING: 正在运行工程
     * PAUSE: 工程已暂停
     * ERROR: 工程运行出错
     */
    Response getProjectState(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "getProjectState", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 运行指定工程
     *
     * \param projectName 工程名（避免使用中文字符）
     * \param taskName 程序名（避免使用中文字符）
     * \param label 标签名（避免使用中文字符）
     * \param timeout
     * \return
     */
    Response runProject(const std::string& projectName,
                        const std::string& taskName = "",
                        const std::string& label    = "",
                        int                timeout  = 30) {
        json param = {
            {"projectName", projectName},
            {"taskName",    taskName   },
            {"taskName",    label      }
        };
        auto     future = _request.send("projexecute", "run", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 运行最后打开的工程
     *
     * \param timeout
     * \return
     */
    Response runLastProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "runLast", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 停止运行工程
     *
     * \param timeout
     * \return
     */
    Response stopProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "stop", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 暂停工程运行
     *
     * \param timeout
     * \return
     */
    Response pauseProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "pause", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 恢复工程运行
     *
     * \param timeout
     * \return
     */
    Response resumeProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "resume", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
			if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 初始化RS485，建立连接
     *
     * \param type 指定接口, "RS485": keba控制器RS485接口，"EC2RS485": 机械臂末端RS485接口
     * \param baudrate 波特率
     * \param stopBit 停止位, 0: 1停止位; 1: 1 1/2停止位; 2: 2停止位
     * \param parity 奇偶校验 0: 无; 1: 奇校验 2: 偶校验
     * \param dataBit 数据位，注：机械臂末端RS485接口的数据位固定为8，不可更改
	 * \param timeout 等待超时时间，单位秒
	 * \return
     */
    Response rs485Init(std::string type, int baudrate, int stopBit = 0, int parity = 0, int dataBit = 8, int timeout = 30) {
        json param = {
            {"baudrate", baudrate},
            {"stopBit", stopBit},
            {"parity", parity},
            {"dataBit", dataBit}
        };
        auto future = _request.send(type, "init", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 通过RS485发送数据
     *
     * \param type 指定接口, "RS485": keba控制器RS485接口，"EC2RS485": 机械臂末端RS485接口
     * \param msg 发送的数据，列表中按顺序表示每个字节的数值
     * \param timeout 等待超时时间，单位秒
     * \return
     */
    Response rs485Write(std::string type, std::vector<UINT8> msg, int timeout = 30) {
        json dataArray = json::array();
        std::for_each(msg.cbegin(), msg.cend(), [&](const UINT8& data) {
            dataArray.push_back(data);
            });
        auto future = _request.send(type, "write", dataArray, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 清空读缓存
     *
     * \param type 指定接口, "RS485": keba控制器RS485接口，"EC2RS485": 机械臂末端RS485接口
     * \param timeout 等待超时时间，单位秒
     * \return
     */
    Response rs485FlushReadBuffer(std::string type, int timeout = 30) {
        json param = {

        };
        auto future = _request.send(type, "flushReadBuffer", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 读取RS485收到的数据
     *
     * \param type 指定接口, "RS485": keba控制器RS485接口，"EC2RS485": 机械臂末端RS485接口
     * \param length 读取的字节数
     * \param timeout 等待超时时间，单位秒
     * \return
     */
    Response rs485Read(std::string type, int length, int timeout = 30) {
        json param = {
            {"length", length},
            {"timeout", timeout * 1000000},
        };
        auto future = _request.send(type, "read", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
            else {
                int dataArrayLength = data["data"].size();
                std::vector<INT32> dataArray;
                dataArray.reserve(dataArrayLength);
                for (int i = 0; i < dataArrayLength; ++i) {
                    dataArray.push_back(data["data"].at(i).get<UINT32>());
                }

                res.data = dataArray;
            }
        }

        return res;
    }

private:
    Request _request;

    double _homePosition[6] = {0, 0, 90, 0, 90, 0};
    double _packPosition[6] = {89, 0, 148.3, -31.7, 181, 180};
};
}  // namespace c2
