# Re-implmenet RDC using miniROS

There are the following modules for the minimal RDC:

### **StateNode**

WebDriver and stateMachine is under this node. It also manage keybinding. It controls the whole RDC state.

**Internal state**:
- ALIGN: Aligning robot to gello's position. (SLOW)
- ENABLE: Enable the moving of robot. (The FilIO is off).
- RECORD: Recording state. (The FilIO is on).
- STOP: Pause the robot (Can interrupt anytime).
- DRAG: Enter the drag mode, when you can drag the robot.

### **RobotNode**

**Internal state**:
- INIT
- ALIGN
- ENABLE
- DISABLE
- DRAG

Gello & Robot is under this node. It has a *statesub* to subscribe the rdc state. It has a *statesender* to update the internal state. It has another sender to send to FileNode.

### **FileNode**

**Internal state**:
- ON
- SAVE
- DROP
- OFF

Cameras & RobotStateRecver is under this node. It has a *statesub* to subscribe the rdc state. It has a *statesender* to update the internal state. It saves the data locally on the fly.