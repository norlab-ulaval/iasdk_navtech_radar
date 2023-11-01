cmake_minimum_required(VERSION 3.16)

project(sdk_networking C CXX)

add_library(networking STATIC
    # Framework
    ${PROJECT_SOURCE_DIR}/core/socket/Socket.cpp
    ${PROJECT_SOURCE_DIR}/core/socket/TCP_socket.cpp
    ${PROJECT_SOURCE_DIR}/core/socket/UDP_socket.cpp
    ${PROJECT_SOURCE_DIR}/core/utility/IP_address.cpp
    ${PROJECT_SOURCE_DIR}/core/utility/net_conversion.cpp

    # CAT-240
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/client/Cat240_client.cpp
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/event_handling/Cat240_events.cpp
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/server/Cat240_server.cpp
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/protocol/Cat240_message.cpp
  
    # Colossus
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/client/Colossus_client.cpp
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/event_handling/Colossus_events.cpp
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/protocol/Colossus_network_message.cpp
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/test/Colossus_test.cpp
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/test/Colossus_test_runner.cpp
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/test/Colossus_test_cases.cpp

    # NMEA
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/protocol/NMEA_message.cpp
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/protocol/NMEA_parser.cpp
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/client/NMEA_client.cpp
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/server/NMEA_server.cpp
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/event_handling/NMEA_events.cpp
)

set(NETWORKING_INC_DIRS
    ${PROJECT_SOURCE_DIR}

    ${PROJECT_SOURCE_DIR}/core
    ${PROJECT_SOURCE_DIR}/core/socket
    ${PROJECT_SOURCE_DIR}/core/tls
    ${PROJECT_SOURCE_DIR}/core/client
    ${PROJECT_SOURCE_DIR}/core/connection
    ${PROJECT_SOURCE_DIR}/core/connection/bytestream
    ${PROJECT_SOURCE_DIR}/core/connection/datagram
    ${PROJECT_SOURCE_DIR}/core/connection/messaging
    ${PROJECT_SOURCE_DIR}/core/connection/secure
    ${PROJECT_SOURCE_DIR}/core/server
    ${PROJECT_SOURCE_DIR}/core/utility
    ${PROJECT_SOURCE_DIR}/core/event_handling

    ${PROJECT_SOURCE_DIR}/protocol
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/client
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/server
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/protocol
    ${PROJECT_SOURCE_DIR}/protocol/CAT_240/event_handling

    ${PROJECT_SOURCE_DIR}/protocol/Colossus/client
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/server
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/protocol
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/event_handling
    ${PROJECT_SOURCE_DIR}/protocol/Colossus/test

    ${PROJECT_SOURCE_DIR}/protocol/NMEA/client
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/server
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/protocol
    ${PROJECT_SOURCE_DIR}/protocol/NMEA/event_handling
)

target_include_directories(networking INTERFACE
${NETWORKING_INC_DIRS}
)

target_include_directories(networking PRIVATE
    ${NETWORKING_INC_DIRS}
)

target_link_libraries(networking PRIVATE
    utility
    protobuf
    navigation
)

