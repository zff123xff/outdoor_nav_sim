# config

#### algo_id.yaml：算法及对应ID

#### topic_key.yaml：各模块话题总结及对应的key

#### mapping_rules.yaml：ros1与ros2消息转换对应规则

#### profiles.xml: 通信本地回环、话题Qos配置



## topic_key

- 核心库、算法、模块等话题严格按照topic_key.yaml定义

#### 使用方法1

在CMakeLists.txt中增加两行

```
find_package(yaml-cpp REQUIRED)
target_link_libraries(your_node_name yaml-cpp)
```

在.h头文件中增加

```
#include <yaml-cpp/yaml.h>
```

在.cpp中通过以下方法使用

```
std::string config_path = "src/msg_set_msgs/config/topic_key.yaml";
YAML::Node  config      = YAML::LoadFile(config_path);

举例
multi_goal_sub_ = this->create_subscription<msg_set_msgs::msg::MultiGoal>(config["algo"]["MOVE_POSITION"]["input_topic"].as<std::string>(), qos_default,
std::bind(&MovePosition::multiGoalCallback, this, _1));
```

#### 使用方法2

在CMakeLists.txt中增加

```
find_package(yaml-cpp REQUIRED)
target_link_libraries(your_node_name yaml-cpp)

find_package(ament_index_cpp REQUIRED)
ament_target_dependencies(your_node_name ament_index_cpp)
```

在.h头文件中增加

```
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
```

在.cpp中通过以下方法使用

```
std::string msg_set_path   = ament_index_cpp::get_package_share_directory("msg_set_msgs");
std::string topic_key_path = msg_set_path + "/config/topic_key.yaml";
YAML::Node  config         = YAML::LoadFile(topic_key_path);
```

