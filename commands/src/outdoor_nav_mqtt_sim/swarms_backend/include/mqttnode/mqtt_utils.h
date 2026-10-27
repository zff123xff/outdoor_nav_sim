//
// Created by ljy on 12/17/23.
//

#ifndef BACKEND_CLUSTER_MQTT_UTILS_H
#define BACKEND_CLUSTER_MQTT_UTILS_H

#include "json/json.h"
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <msg_set_msgs/msg/multi_task.hpp>
#include <msg_set_msgs/msg/jump_task.hpp>
#include <msg_set_msgs/msg/cover_param.hpp>
#include <msg_set_msgs/msg/cover_task.hpp>
#include <msg_set_msgs/msg/cover_index.hpp>
#include <msg_set_msgs/msg/global_param.hpp>
#include <msg_set_msgs/msg/multi_task_confirm.hpp>
#include <msg_set_msgs/msg/cover_confirm.hpp>
#include <msg_set_msgs/msg/targets_pose.hpp>
#include <msg_set_msgs/msg/adjust_tasks.hpp>
#include <fstream>
#include <experimental/filesystem>
#include <chrono>
#include <iomanip>
#include <unistd.h>
#include <limits.h>

#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <openssl/ossl_typ.h>

#include "mqtt_response.h"
#include "../mavlink_ros/uav_state.h"
#include "msg_set_msgs/msg/multi_goal.hpp"

#define RAD_TO_DEG 57.29577951308232 // 180.0/3.141592653589793238463

class ImgUtils
{
public:
    static std::string bufferToBase64(const void *data, int len)
    {
        // 创建BIO对象
        BIO *b64 = BIO_new(BIO_f_base64());
        BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);

        // 创建BIO对象
        BIO *mem = BIO_new(BIO_s_mem());

        // 将BIO对象链接起来
        BIO_push(b64, mem);

        // 写入数据到BIO对象中
        BIO_write(b64, data, len);

        // 冲洗BIO对象
        BIO_flush(b64);

        // 获取BIO对象中的数据
        BUF_MEM *bptr = nullptr;
        BIO_get_mem_ptr(b64, &bptr);

        // 创建输出字符串
        std::string output(bptr->data, bptr->length);

        // 释放资源
        BIO_free_all(b64);

        return output;
    }
};

class JsonUtils
{
public:
    static float format(float f, int digits = 2)
    {
        try
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(digits) << f; // 保留2位小数
            return std::stof(ss.str());
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(std::string &message, std::string key = "data")
    {
        try
        {
            Json::Value root;
            root[key] = message;
            Json::StyledWriter writer;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(int &message, std::string key = "data")
    {
        try
        {
            Json::Value root;
            root[key] = message;
            Json::StyledWriter writer;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const std_msgs::msg::Int32MultiArray::ConstPtr &msg, std::string key = "data")
    {
        try
        {
            Json::Value root;
            Json::Value fields(Json::arrayValue);
            for (const auto &field : msg->data)
            {
                fields.append(field);
            }
            root[key] = fields;
            Json::StyledWriter writer;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const mavros_msgs::msg::State::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::Value statePoint;
            Json::StyledWriter writer;
            statePoint["connected"] = msg->connected;
            statePoint["armed"] = msg->armed;
            statePoint["guided"] = msg->guided;
            statePoint["manual_input"] = msg->manual_input;
            statePoint["mode"] = msg->mode;
            root[key] = statePoint;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const sensor_msgs::msg::NavSatFix::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::Value navSatFix;
            Json::StyledWriter writer;
            navSatFix["status"] = msg->status.status;
            navSatFix["service"] = msg->status.service;
            navSatFix["latitude"] = msg->latitude;
            navSatFix["longitude"] = msg->longitude;
            navSatFix["altitude"] = msg->altitude;
            root[key] = navSatFix;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const msg_set_msgs::msg::TargetsPose::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::Value targetGps;
            Json::Value targetsGps(Json::arrayValue);
            Json::StyledWriter writer;
            // for (size_t i = 0; i < msg.size(); ++i) {
            //     const auto &target = (*msg)[i];  // 使用 [] 访问数组中的元素
            //     targetGps["latitude"] = target.lat;
            //     targetGps["longitude"] = target.lon;
            //     targetGps["altitude"] = target.alt;
            //     targetGps["type"] = target.label;
            //     targetsGps.append(targetGps);
            // }

            for (const auto &target : msg->targets_pose)
            {
                targetGps["latitude"] = target.lat;
                targetGps["longitude"] = target.lon;
                targetGps["altitude"] = target.alt;
                targetGps["type"] = target.label;
                targetGps["id"] = target.id;
                targetsGps.append(targetGps);
            }
            root[key] = targetsGps;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const std_msgs::msg::UInt8MultiArray::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::Value fields(Json::arrayValue);
            Json::StyledWriter writer;
            for (const auto &field : msg->data)
            {
                fields.append(field);
            }
            root[key] = fields;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const std_msgs::msg::UInt8::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            root[key] = msg->data;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const std_msgs::msg::Bool::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            root[key] = msg->data ? 1 : 0;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const msg_set_msgs::msg::CoverIndex::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value passWayPoint;
            passWayPoint["groupId"] = msg->group_id;
            passWayPoint["currentPointId"] = msg->point_index;
            root[key] = passWayPoint;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const msg_set_msgs::msg::MultiTaskConfirm::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value mTaskConfirm;
            mTaskConfirm["group_id"] = msg->group_id;
            mTaskConfirm["drones_size"] = msg->drones_size;
            mTaskConfirm["jtask_hash"] = msg->jtask_hash;
            mTaskConfirm["mtask_hash"] = msg->mtask_hash;
            root[key] = mTaskConfirm;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const msg_set_msgs::msg::CoverConfirm::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value coverConfirm;
            coverConfirm["group_id"] = msg->group_id;
            coverConfirm["drones_size"] = msg->drones_size;
            coverConfirm["point_index"] = msg->point_index;
            coverConfirm["cover_hash"] = msg->cover_hash;
            root[key] = coverConfirm;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const msg_set_msgs::msg::CoverParam::ConstPtr &msg, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value coverParam;
            coverParam["velocity"] = msg->velocity;
            coverParam["waypoint_radius"] = msg->waypoint_radius;
            root[key] = coverParam;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(const MqttResponseCode code, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value rtContent;
            rtContent["code"] = (int)code;
            root[key] = rtContent;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string jsonValue(UAVState &state, std::string key = "data")
    {
        try
        {

            Json::Value root;
            Json::StyledWriter writer;
            Json::Value statePoint;
            Json::Value navSatFix;
            Json::Value rtContent;

            if (state.px4State_.isUpdated)
            {
                statePoint["connected"] = state.px4State_.connected;
                statePoint["armed"] = state.px4State_.armed;
                statePoint["guided"] = state.px4State_.guided;
                statePoint["manual_input"] = state.px4State_.manual_input;
                statePoint["mode"] = state.px4State_.mode;
                rtContent["px4State"] = statePoint;
            }

            if (state.globalPosition_.isUpdated)
            {
                navSatFix["status"] = state.globalPosition_.status;
                navSatFix["service"] = state.globalPosition_.service;
                navSatFix["latitude"] = state.globalPosition_.latitude;
                navSatFix["longitude"] = state.globalPosition_.longitude;
                navSatFix["altitude"] = state.globalPosition_.altitude;
                rtContent["global_position"] = navSatFix;
            }

            if (state.robotMajorState_.isUpdated)
            {
                rtContent["robotMajorState"] = state.robotMajorState_.state;
            }

            root[key] = rtContent;
            return writer.write(root);
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static int32_t msgIntValue(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asInt();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static int8_t msgInt8Value(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asInt();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static uint8_t msgUInt8Value(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asInt();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static double msgDoubleValue(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asDouble();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static float msgFloatValue(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asFloat();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std::string msgStrValue(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asString();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static bool msgBoolValue(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            return root[key].asBool();
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::AdjustTasks jsonAdjustTask(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            msg_set_msgs::msg::AdjustTasks adjustTask;
            adjustTask.task_id = root[key]["taskId"].asString();
            for (const auto &id : root[key]["dronesId"])
            {
                adjustTask.drones_id.push_back(id.asInt());
            }

            return adjustTask;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static mavros_msgs::msg::WaypointList jsonPath(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            mavros_msgs::msg::WaypointList waypointList;
            int frame = root[key]["frame"].asInt();
            float altitude = root[key]["altitude"].asFloat();
            const Json::Value &pathPoints = root[key]["path"];
            int index = 0;
            waypointList.waypoints.resize(pathPoints.size());
            for (const auto &pathPoint : pathPoints)
            {
                mavros_msgs::msg::Waypoint waypoint;
                waypoint.frame = frame;
                waypoint.z_alt = altitude;
                waypoint.x_lat = pathPoint["x_lat"].asFloat();
                waypoint.y_long = pathPoint["y_lon"].asFloat();
                waypoint.param1 = pathPoint["h"].asFloat();
                waypointList.waypoints[index] = waypoint;
                index++;
            }
            return waypointList;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static float jsonRelayCtl(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            float relayHeight = root[key]["relay_height"].asFloat();
            return relayHeight;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::MultiTask jsonTaskStream(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            msg_set_msgs::msg::MultiTask multiTask;
            const Json::Value &multiTaskData = root[key]["multiTask"];

            multiTask.group_id = multiTaskData["groupId"].asString();
            multiTask.edited_task = multiTaskData["editTaskId"].asString();

            const Json::Value &drones = multiTaskData["dronesId"];
            multiTask.drones_id.resize(drones.size());
            int index = 0;
            for (const auto &drone : drones)
            {
                multiTask.drones_id[index] = drone.asInt();
                ++index;
            }

            const Json::Value &fences = multiTaskData["fences"];
            multiTask.fence.resize(fences.size());
            index = 0;
            for (const auto &fence : fences)
            {
                multiTask.fence[index].type = fence["areaType"].asInt();
                const Json::Value &areaDatas = fence["areaData"];
                multiTask.fence[index].data.resize(areaDatas.size());
                int count = 0;
                for (const auto &areaData : areaDatas)
                {
                    multiTask.fence[index].data[count] = areaData.asFloat();
                    ++count;
                }
                ++index;
            }

            const Json::Value &threatAreas = multiTaskData["threatAreas"];
            multiTask.threat_area.resize(threatAreas.size());
            index = 0;
            for (const auto &threatArea : threatAreas)
            {
                multiTask.threat_area[index].type = threatArea["areaType"].asInt();
                const Json::Value &areaDatas = threatArea["areaData"];
                multiTask.threat_area[index].data.resize(areaDatas.size());
                int count = 0;
                for (const auto &areaData : areaDatas)
                {
                    multiTask.threat_area[index].data[count] = areaData.asFloat();
                    ++count;
                }
                ++index;
            }

            const Json::Value &taskStreams = multiTaskData["taskStream"];
            multiTask.task_stream.resize(taskStreams.size());
            index = 0;
            for (const auto &taskStream : taskStreams)
            {
                multiTask.task_stream[index].task_type = taskStream["taskType"].asInt();
                multiTask.task_stream[index].task_id = taskStream["taskId"].asString();
                multiTask.task_stream[index].task_area.type = taskStream["areaType"].asInt();

                const Json::Value &areaDatas = taskStream["areaData"];
                multiTask.task_stream[index].task_area.data.resize(areaDatas.size());
                int count = 0;
                for (const auto &areaData : areaDatas)
                {
                    multiTask.task_stream[index].task_area.data[count] = areaData.asFloat();
                    ++count;
                }

                const Json::Value &uavIds = taskStream["dronesId"];
                multiTask.task_stream[index].drones_id.resize(uavIds.size());
                count = 0;
                for (const auto &uavId : uavIds)
                {
                    multiTask.task_stream[index].drones_id[count] = uavId.asInt();
                    ++count;
                }

                multiTask.task_stream[index].high_height = taskStream["highHeight"].asFloat();
                multiTask.task_stream[index].low_height = taskStream["lowHeight"].asFloat();
                multiTask.task_stream[index].cruise_height = taskStream["cruisingHeight"].asFloat();
                multiTask.task_stream[index].cruise_speed = taskStream["cruisingSpeed"].asFloat();
                multiTask.task_stream[index].task_mode = taskStream["taskMode"].asInt();

                const Json::Value &trackingDis = taskStream["trackingDis"];
                multiTask.task_stream[index].tracking_dis.resize(trackingDis.size());
                count = 0;
                for (const auto &trackingDistance : trackingDis)
                {
                    multiTask.task_stream[index].tracking_dis[count] = trackingDistance.asInt();
                    ++count;
                }

                const Json::Value &arrivePath = taskStream["arrivePath"];
                multiTask.task_stream[index].arrive_path.waypoints.resize(arrivePath.size());
                count = 0;
                for (const auto &point : arrivePath)
                {
                    mavros_msgs::msg::Waypoint waypoint;
                    waypoint.x_lat = point["x_lat"].asFloat();
                    waypoint.y_long = point["y_lon"].asFloat();
                    waypoint.z_alt = point["z_alt"].asFloat();
                    multiTask.task_stream[index].arrive_path.waypoints[count] = waypoint;
                    ++count;
                }

                const Json::Value &targetTypes = taskStream["targetTypeArray"];
                multiTask.task_stream[index].target_type.resize(targetTypes.size());
                count = 0;
                for (const auto &targetType : targetTypes)
                {
                    multiTask.task_stream[index].target_type[count] = targetType.asInt();
                    ++count;
                }

                multiTask.task_stream[index].search_count = taskStream["searchCount"].asInt();
                multiTask.task_stream[index].search_duration = taskStream["searchDuration"].asFloat();
                multiTask.task_stream[index].attack_speed = taskStream["attackSpeed"].asFloat();
                multiTask.task_stream[index].move_speed = taskStream["moveSpeed"].asFloat();
                multiTask.task_stream[index].auto_attack = taskStream["autoAttack"].asBool();
                multiTask.task_stream[index].safe_mode = taskStream["safeMode"].asBool();
                multiTask.task_stream[index].takeoff_mode = taskStream["takeoffMode"].asInt();
                multiTask.task_stream[index].move_mode = taskStream["moveMode"].asInt();
                multiTask.task_stream[index].gather_mode = taskStream["gatherMode"].asInt();
                multiTask.task_stream[index].search_mode = taskStream["searchMode"].asInt();
                multiTask.task_stream[index].tracking_mode = taskStream["trackingMode"].asInt();
                multiTask.task_stream[index].attack_mode = taskStream["attackMode"].asInt();
                multiTask.task_stream[index].formation_shape = taskStream["swaramFormation"].asInt();
                multiTask.task_stream[index].hazard_level = taskStream["hazardLevel"].asInt();
                ++index;
            }

            return multiTask;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::JumpTask jsonTaskStreamJump(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            msg_set_msgs::msg::JumpTask jumpTask;
            const Json::Value &jumpTaskData = root[key];

            jumpTask.task_id = jumpTaskData["taskIndex"].asString();

            const Json::Value &path = jumpTaskData["path"];
            jumpTask.arrive_path.waypoints.resize(path.size());
            int count = 0;
            for (const auto &point : path)
            {
                mavros_msgs::msg::Waypoint waypoint;
                waypoint.x_lat = point["x_lat"].asFloat();
                waypoint.y_long = point["y_lon"].asFloat();
                waypoint.z_alt = point["z_alt"].asFloat();
                jumpTask.arrive_path.waypoints[count] = waypoint;
                ++count;
            }

            return jumpTask;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std_msgs::msg::Bool jsonTaskPause(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            std_msgs::msg::Bool stream_pause;
            const Json::Value &streamPauseData = root[key];
            stream_pause.data = streamPauseData.asBool();

            return stream_pause;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::CoverTask jsonCoverTask(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            msg_set_msgs::msg::CoverTask coverTask;
            const Json::Value &coverTaskData = root[key];
            coverTask.group_id = coverTaskData["groupId"].asInt();
            coverTask.group_relay_id = coverTaskData["groupRelayId"].asInt();

            const Json::Value &drones = coverTaskData["dronesId"];
            coverTask.drones_id.resize(drones.size());
            int index = 0;
            for (const auto &drone : drones)
            {
                coverTask.drones_id[index] = drone.asInt();
                ;
                ++index;
            }

            const Json::Value &path_points = coverTaskData["singlePath"];
            coverTask.path_points.waypoints.resize(path_points.size());
            index = 0;
            for (const auto &point : path_points)
            {
                mavros_msgs::msg::Waypoint waypoint;
                waypoint.x_lat = point["x_lat"].asFloat();
                waypoint.y_long = point["y_lon"].asFloat();
                waypoint.z_alt = point["z_alt"].asFloat();
                waypoint.command = point["pointId"].asInt();
                coverTask.path_points.waypoints[index] = waypoint;
                ++index;
            }

            coverTask.param.velocity = coverTaskData["velocity"].asFloat();
            coverTask.param.waypoint_radius = coverTaskData["wayPointRadius"].asFloat();
            coverTask.param.height = coverTaskData["height"].asFloat();

            return coverTask;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static mavros_msgs::msg::WaypointList jsonCoverPath(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            mavros_msgs::msg::WaypointList coverPath;
            const Json::Value &coverPathData = root[key];
            const Json::Value &path_points = coverPathData["singlePath"];
            coverPath.waypoints.resize(path_points.size());
            int index = 0;
            for (const auto &point : path_points)
            {
                mavros_msgs::msg::Waypoint waypoint;
                waypoint.x_lat = point["x_lat"].asFloat();
                waypoint.y_long = point["y_lon"].asFloat();
                waypoint.z_alt = point["z_alt"].asFloat();
                coverPath.waypoints[index] = waypoint;
                ++index;
            }

            return coverPath;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::CoverParam jsonCoverParam(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            msg_set_msgs::msg::CoverParam coverParam;
            const Json::Value &coverParamData = root[key];
            coverParam.velocity = coverParamData["velocity"].asFloat();
            coverParam.waypoint_radius = coverParamData["wayPointRadius"].asFloat();
            coverParam.height = coverParamData["height"].asFloat();

            return coverParam;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std_msgs::msg::Float32MultiArray jsonCoverGoalPoint(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            std_msgs::msg::Float32MultiArray goalPoint;
            goalPoint.data.resize(3);
            const Json::Value &goalPointData = root[key];
            goalPoint.data[0] = goalPointData["x_lat"].asFloat();
            goalPoint.data[1] = goalPointData["y_lon"].asFloat();
            goalPoint.data[2] = goalPointData["z_alt"].asFloat();
            return goalPoint;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std_msgs::msg::Int32MultiArray jsonToInt32Array(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);
            std_msgs::msg::Int32MultiArray data;
            const Json::Value &dataArray = root[key];
            for (const auto &value : dataArray)
            {
                data.data.push_back(value.asInt());
            }
            return data;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::GlobalParam jsonGlobalParam(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            msg_set_msgs::msg::GlobalParam globalParam;
            const Json::Value &globalParamData = root[key];
            globalParam.missioncancel_action = globalParamData["missionCancelAction"].asInt();
            const Json::Value &lowBatteryFailsafe = globalParamData["lowBatteryFailsafe"];
            globalParam.lowbattery_failsafeaction = lowBatteryFailsafe["failsafeAction"].asInt();
            globalParam.battery_warnlevel = lowBatteryFailsafe["batteryWarnLevel"].asInt();
            globalParam.battery_failsafelevel = lowBatteryFailsafe["batteryFailsafeLevel"].asInt();
            globalParam.battery_emergencylevel = lowBatteryFailsafe["batteryEmergencyLevel"].asInt();
            const Json::Value &dataLinkLossFailsafe = globalParamData["dataLinkLossFailsafe"];
            globalParam.linkloss_failsafeaction = dataLinkLossFailsafe["failsafeAction"].asInt();
            globalParam.auto_attack = dataLinkLossFailsafe["autoAttack"].asBool();
            globalParam.linkloss_timeout = dataLinkLossFailsafe["dataLinkLossTimeout"].asInt();
            return globalParam;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std_msgs::msg::Bool jsonCancelTask(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            std_msgs::msg::Bool cancelTask;
            const Json::Value &cancelTaskData = root[key];
            cancelTask.data = cancelTaskData["cancel"].asBool();
            return cancelTask;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static std_msgs::msg::Int8MultiArray jsonDronesAdjust(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            std_msgs::msg::Int8MultiArray dronesId;
            const Json::Value &dronesIdData = root[key];
            dronesId.data.resize(dronesIdData.size());
            int count = 0;
            for (const auto &id : dronesIdData)
            {
                dronesId.data[count] = id.asInt();
                count++;
            }
            return dronesId;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static sensor_msgs::msg::NavSatFix jsonrefGps(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            sensor_msgs::msg::NavSatFix refGps;
            const Json::Value &refGpsDate = root[key];
            refGps.latitude = refGpsDate[0].asFloat();
            refGps.longitude = refGpsDate[1].asFloat();

            return refGps;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    static msg_set_msgs::msg::MultiGoal jsonMultiGoal(const std::string &message, std::string key = "data")
    {
        try
        {
            Json::Reader reader;
            Json::Value root;
            reader.parse(message, root);

            msg_set_msgs::msg::MultiGoal multiGoalMsg;
            multiGoalMsg.is_gps_aid = true; // 根据业务逻辑调整
            multiGoalMsg.is_gps_hgt = true; // 根据业务逻辑调整
            const Json::Value &multiGoal = root[key];
            for (const auto &point : multiGoal)
            {
                msg_set_msgs::msg::MultiGoalPoint multiGoalPoint;
                multiGoalPoint.x_or_lat = point["x_lat"].asDouble();
                multiGoalPoint.y_or_lon = point["y_lon"].asDouble();
                multiGoalPoint.z_or_alt = point["z_alt"].asDouble();
                multiGoalMsg.multi_goal_points.push_back(multiGoalPoint);
            }
            return multiGoalMsg;
        }
        catch (const std::exception &e)
        {
            std::string exceptionMsg = getExceptionMsg(e.what(), __FILE__, __LINE__, __FUNCTION__);
            std::cerr << exceptionMsg << std::endl;
        }
    }

    /**
     *
     * @param msg exception message
     * @param file the file of  current exception
     * @param line the line of  current exception
     * @param func the function of  current exception
     * @return
     */

    static std::string getExceptionMsg(const std::string msg, const std::string &file, int line,
                                       const std::string &func)
    {
        std::stringstream ss;
        ss << "File: " << file << ", Line: " << line << ", Function: " << func << " - ";
        ss << "Message: " << msg;
        return ss.str();
    }

    ~JsonUtils() {};
};

class FileUtils
{
public:
    static std::string getExecutablePath()
    {
        std::string path;
        try
        {
            std::string str;
#ifdef _WIN32
            char buffer[MAX_PATH];
            GetModuleFileName(NULL, buffer, MAX_PATH);
            str = buffer;
#elif __linux__
            char buffer[PATH_MAX];
            ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (len != -1)
            {
                buffer[len] = '\0';
                str = buffer;
            }
#endif
            size_t lastSlashPos = str.find_last_of("/\\");
            path = str.substr(0, lastSlashPos);
            return path;
        }
        catch (const std::exception &e)
        {
            std::cerr << "获取可执行文件路径失败: " << e.what() << std::endl;
            return path;
        }
    }

    ~FileUtils() {};
};

#endif // BACKEND_CLUSTER_MQTT_UTILS_H
