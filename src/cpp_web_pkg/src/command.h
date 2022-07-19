//
// 2022-06-09, jjuiddong
// robot command definition
//  - RobotCmd interface
//  - service communication
//
//  - RobotCmd Interface
//      int32 type
//      string name
//      uint16[] ui0
//      uint16[] ui1
//      uint32[] i0
//      float32[] f0
//      string[] s0
//      char[] c0
// ---
//      int32 type
//      string name
//      uint16[] ui0
//      uint16[] ui1
//      uint32[] i0
//      float32[] f0
//      string[] s0
//      char[] c0
//      int32 result
//
#pragma once


namespace robot {
    enum class eCommandType
    {
        None,
        ReqLogin = 1000,
        AckLogin,
        ReqReady,
        AckReady,
        ReqStop,
        AckStop,
        EmptyWork,
        AckEmptyWork,
        EmergencyStop,
        AckEmergencyStop,
        ReqMove,
        AckMove,
        AckMoveEnd,
        ReqRemoteMove,
        AckRemoteMove,
        AckRemoteMoveEnd,
        ReqMoveCancel,
        AckMoveCancel,
        ReqWait,
        AckWait,
        ReqWork,
        AckWork,
        AckWorkEnd,
        ReqUntil,
        AckUntil,
        AwakeUntil,
        ReqMessage,
        AckMessage,
    };
}

// Request Login
//  - Robot -> Server
//      - type: robot::eCommandType::ReqLogin
//      - name: robot name
//      - s0[0]: password
//
// Ack Login
//  - Server -> Robot
//      - type: robot::eCommandType::AckLogin
//      - result: 1=success, others:fail
//
// Request Ready
//  - Robot -> Server
//      - type: robot::eCommandType::ReqReady
//      - name: robot name
//
// Ack Ready
//  - Server -> Robot
//      - type: robot::eCommandType::AckReady
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Stop
//  - Server -> Robot
//      - type: robot::eCommandType::ReqStop
//      - name: robot name
//      - i0[0]: reason
// 
// Ack Stop
//  - Robot -> Server
//      - type: robot::eCommandType::AckStop
//      - name: robot name
//      - result: 1=success, others:fail
// 
// Empty Work
//  - Robot -> Server
//      - type: robot::eCommandType::EmptyWork
//      - name: robot name
//
// Ack Empty Work
//  - Server -> Robot
//      - type: robot::eCommandType::AckEmptyWork
//      - name: robot name
//      - result: 1=success, others:fail
//
// EmergencyStop
//  - Server -> Robot
//      - type: robot::eCommandType::EmergencyStop
//      - name: robot name
//      - i0[0]: reason
//
// AckEmergencyStop
//  - Robot -> Server
//      - type: robot::eCommandType::AckEmergencyStop
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Move
//  - Server -> Robot
//      - type: robot::eCommandType::ReqMove
//      - name: robot name
//      - ui0: path, vertex index array
//      - ui1: wait time, waitting time array (10 milliseconds unit)
// 
// Ack Move
//  - Robot -> Server
//      - type: robot::eCommandType::AckMove
//      - name: robot name
//      - result: 1=success, others:fail
//
// Ack Move End
//  - Robot -> Server
//      - type: robot::eCommandType::AckMoveEnd
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Remote Move
//  - Server -> Robot
//      - type: robot::eCommandType::ReqRemoteMove
//      - name: robot name
//      - f0[0]: move time
//      - f0[1]: linear velocity
//      - f0[2]: angular velocity
//
// Ack Remote Move
//  - Robot -> Server
//      - type: robot::eCommandType::AckRemoteMove
//      - name: robot name
//      - result: 1=success, others:fail
//
// Ack Remote MoveEnd
//  - Robot -> Server
//      - type: robot::eCommandType::AckRemoteMoveEnd
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Move Cancel
//  - Server -> Robot
//      - type: robot::eCommandType::ReqMoveCancel
//      - name: robot name
//
// Ack Move Cancel
//  - Robot -> Server
//      - type: robot::eCommandType::AckMoveCancel
//      - name: robot name
//      - i0[0]: stop vertex index
//      - result: 1=success, others:fail
//
// Request Wait
//  - Server -> Robot
//      - type: robot::eCommandType::ReqWait
//      - name: robot name
//
// Ack Wait
//  - Robot -> Server
//      - type: robot::eCommandType::AckWait
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Work
//  - Server -> Robot
//      - type: robot::eCommandType::ReqWork
//      - name: robot name
//      - i0[0]: work type
//      - i0[1]: work vertex index
//
// Ack Work
//  - Robot -> Server
//      - type: robot::eCommandType::AckWork
//      - name: robot name
//      - result: 1=success, others:fail
//
// Ack WorkEnd
//  - Robot -> Server
//      - type: robot::eCommandType::AckWorkEnd
//      - name: robot name
//      - result: 1=success, others:fail
//
// Request Until
//  - Server -> Robot
//      - type: robot::eCommandType::ReqUntil
//      - name: robot name
//      - i0[0]: vm id
//      - i0[1]: waitting time
//      - s0[0]: start message
//      - s0[1]: finish message
//
// Ack Until
//  - Robot -> Server
//      - type: robot::eCommandType::AckUntil
//      - name: robot name
//      - i0[0]: vm id
//      - result: 1=success, others:fail
//
// Awake Until
//  - Robot -> Server
//      - type: robot::eCommandType::AwakeUntil
//      - name: robot name
//      - i0[0]: state
//      - result: 1=success, others:fail
//
// Request Message
//  - Server -> Robot
//      - type: robot::eCommandType::ReqMessage
//      - name: robot name
//      - i0[0]: vm id
//      - s0[0]: sender name
//      - s0[1]: message
//
// Ack Message
//  - Robot -> Server
//      - type: robot::eCommandType::AckMessage
//      - name: robot name
//      - i0[0]: vm id
//      - result: 1=success, others:fail
//
