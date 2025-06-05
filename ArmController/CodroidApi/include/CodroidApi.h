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
     * �����û�����.
     *
     * \param cmd �ο�UserCommand˵��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ��ȡ������״̬.
     *
     * \param timeout
     * \return Response.data ������json����ͨ�� int state = res.data; �õ���ֵ��
     * ��ֵ����ο�Define.h�ļ��� enum class RobotState
     * None    = -1,  // δ֪
     * Init    = 0,   // ��ʼ��
     * StandBy = 1,   // ���µ�
     * Ready   = 2,   // �ֶ�ģʽ
     * Rescue  = 3,   // ��Ԯģʽ
     * Auto    = 4,   // �Զ�ģʽ
     * Error   = 6,   // �����˳���
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
     * ��ȡ���λ.
     *
     * \param timeout
     * \return Response.data ��json���飬double[6] ���ؽ���ת�Ƕȣ���λ�ȣ�deg��
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
     * ͨ��MovJonit�˶���Homeλ.
     *
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ����Homeλ��.
     *
     * \param position double[6] ���ؽ���ת�Ƕȣ���λ�ȣ�deg��
     */
    void setHomePosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy_s(_homePosition, size, position, size);
    }

    /**
     * ͨ��MovJonit�˶������λ.
     *
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ���ô��λ��.
     *
     * \param position double[6] ���ؽ���ת�Ƕȣ���λ�ȣ�deg��
     */
    void setPackPosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy_s(_packPosition, size, position, size);
    }

    /**
     * �ؽڿռ��˶�.
     *
     * \param position Ŀ��λ�ã�����Ϊdouble[6]������ֱ�Ϊ��1~��6����ת�Ƕȣ���λ�ȣ�deg��������[0,0,90,0,90,0]
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ��ιؽڿռ��˶�.
     *
     * \param segements ���·��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * �ؽڿռ��˶������ȴ�����.
     *
     * \param position Ŀ��λ�ã�����Ϊdouble[6]������ֱ�Ϊ��1~��6����ת�Ƕȣ���λ�ȣ�deg��������[0,0,90,0,90,0]
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ֱ���˶�.
     *
     * \param position Ŀ��λ�ã�����Ϊdouble[6]����ʾ[x,y,z,rx,ry,rz]��
     *      (x,y,z)��ʾ�ѿ����ռ��λ�ã���λ���ף�mm������rx,ry,rz����ʾ��3�����������ת�Ƕȣ���λ�ȣ�deg��
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * Բ���˶�.
     *
     * \param targetPosition Ŀ��λ�ã�����Ϊdouble[6]����ʾ[x,y,z,rx,ry,rz]��
     *      (x,y,z)��ʾ�ѿ����ռ��λ�ã���λ���ף�mm������rx,ry,rz����ʾ��3�����������ת�Ƕȣ���λ�ȣ�deg��
     * \param middlePosition �м�λ�ã���������ͬĿ��λ��
     * \param speed �˶��ٶȣ���λ��/��
     * \param acc �˶����ٶȣ���λ��/��ƽ��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ��εѿ����ռ��˶�.
     *
     * \param segements ���·��
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * �û�����ֹͣ�˶�.
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
     * �ؽڵ㶯. �������ͺ����Ҫ���ֵ㶯��������Ҫ������keepJog�ӿڣ�����㶯����1s����Զ�ֹͣ
     *
     * \param jogIndex ��Ҫ�㶯���ᣬ��Χ1~6
     * \param jogSpeed �㶯�ٶ�
     * \param direction �㶯����
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ĩ�˵㶯. �������ͺ����Ҫ���ֵ㶯��������Ҫ������keepJog�ӿڣ�����㶯����1s����Զ�ֹͣ
     *
     * \param jogIndex 1~6 �ֱ��Ӧ x,y,z,rx,ry,rz
     * \param jogSpeed �㶯�ٶ�
     * \param direction �㶯����
     * \param timeout ��ʱ�ȴ�ʱ��
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
     * ���ֵ�ǰ�㶯. ��500�������ҵ�Ƶ�ʳ������ýӿڣ��ɱ��ֵ㶯����
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
     * ֹͣ�㶯
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
     * ��ȡ��ǰ�ؽ�λ��
     *
     * \param timeout
     * \return Response.data�ǳ���Ϊ6��json���飬��ʾ1~6�ؽڵ���ת�Ƕ�
     * ��ͨ�� vector<double> p = res.data ��ã�
     * ��double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
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
     * ��ȡ��ǰ�ѿ�������λ��
     *
     * \param timeout
     * \return Response.data�ǳ���Ϊ6��json���飬[x,y,z,rx,ry,rz]
     * ��ͨ�� vector<double> p = res.data ��ã�
     * ��double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
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
     * ���ݵѿ���λ�û�ȡ�ؽڽ�
     *
     * \param cpos ��Ҫ���ĵѿ�������
     * \param refAPos ִ�����ʱ�Ĳο���
     * \param timeout
     * \return Response.data�ǳ���Ϊ6��json���飬��ʾ1~6�ؽڵ���ת�Ƕ�
     * ��ͨ�� vector<double> p = res.data ��ã�
     * ��double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
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
     * ������������˿�ֵ.
     *
     * \param port �˿ں�
     * \param val �˿�ֵ��ֻ����0��1
     * \param timeout ���еȴ�ʱ��
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
     * ��ȡ��������˿�ֵ.
     *
     * \param port �˿ں�
     * \param timeout
     * \return Response.data ������json����ͨ�� int val = res.data; �õ���ֵ,ֵΪ0��1
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
     * ������������˿�ֵ.
     *
     * \param port �˿ں�
     * \param val �˿�ֵ��ֻ����0��1
     * \param timeout ���еȴ�ʱ��
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
     * ��ȡ��������˿�ֵ.
     *
     * \param port �˿ں�
     * \param timeout
     * \return Response.data ������json����ͨ�� int val = res.data; �õ���ֵ,ֵΪ�˿ڴӵ͵��߶�Ӧ�Ķ���������ʮ����ֵ
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
     * ���õ�ǰ����ϵ.
     *
     * \param varName ����ϵ������
     *      ע: �ñ�����������UI������ʾ���ñ�����������б������Ҳ������������·��ı���һ�£��Ա������һЩ����
     * \param coor ����ϵ����
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
     * ���õ�ǰ����.
     *
     * \param varName ���߱�����
     *      ע: �ñ�����������UI������ʾ���ñ�����������б������Ҳ������������·��ı���һ�£��Ա������һЩ����
     * \param tool ���߲���
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
     * ���õ�ǰ����.
     *
     * \param varName ���ر�����
     *      ע: �ñ�����������UI������ʾ���ñ�����������б������Ҳ������������·��ı���һ�£��Ա������һЩ����
     * \param dyn ���ز���
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
     * ��ȡ����״̬
     *
     * \param projectName ������������ʹ�������ַ���
     * \param taskName ������������ʹ�������ַ���
     * \param label ��ǩ��������ʹ�������ַ���
     * \param timeout
     * \return std::string
     * IDLE: ���У�û�й���������
     * LOADING: ���ڼ��ع���
     * RUNNING: �������й���
     * PAUSE: ��������ͣ
     * ERROR: �������г���
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
     * ����ָ������
     *
     * \param projectName ������������ʹ�������ַ���
     * \param taskName ������������ʹ�������ַ���
     * \param label ��ǩ��������ʹ�������ַ���
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
     * �������򿪵Ĺ���
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
     * ֹͣ���й���
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
     * ��ͣ��������
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
     * �ָ���������
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
     * ��ʼ��RS485����������
     *
     * \param type ָ���ӿ�, "RS485": keba������RS485�ӿڣ�"EC2RS485": ��е��ĩ��RS485�ӿ�
     * \param baudrate ������
     * \param stopBit ֹͣλ, 0: 1ֹͣλ; 1: 1 1/2ֹͣλ; 2: 2ֹͣλ
     * \param parity ��żУ�� 0: ��; 1: ��У�� 2: żУ��
     * \param dataBit ����λ��ע����е��ĩ��RS485�ӿڵ�����λ�̶�Ϊ8�����ɸ���
	 * \param timeout �ȴ���ʱʱ�䣬��λ��
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
     * ͨ��RS485��������
     *
     * \param type ָ���ӿ�, "RS485": keba������RS485�ӿڣ�"EC2RS485": ��е��ĩ��RS485�ӿ�
     * \param msg ���͵����ݣ��б��а�˳���ʾÿ���ֽڵ���ֵ
     * \param timeout �ȴ���ʱʱ�䣬��λ��
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
     * ��ն�����
     *
     * \param type ָ���ӿ�, "RS485": keba������RS485�ӿڣ�"EC2RS485": ��е��ĩ��RS485�ӿ�
     * \param timeout �ȴ���ʱʱ�䣬��λ��
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
     * ��ȡRS485�յ�������
     *
     * \param type ָ���ӿ�, "RS485": keba������RS485�ӿڣ�"EC2RS485": ��е��ĩ��RS485�ӿ�
     * \param length ��ȡ���ֽ���
     * \param timeout �ȴ���ʱʱ�䣬��λ��
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
