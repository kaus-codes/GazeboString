#!/usr/bin/env python3
"""Idempotent controller startup: verify state after uncertain RPC replies."""
import json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers,LoadController,ConfigureController,SwitchController

rclpy.init();node=Node('sorting_controller_initializer')
clients={kind:node.create_client(cls,'/controller_manager/'+name) for kind,cls,name in [('list',ListControllers,'list_controllers'),('load',LoadController,'load_controller'),('configure',ConfigureController,'configure_controller'),('switch',SwitchController,'switch_controller')]}
def call(client,request,timeout=6):
    future=client.call_async(request);end=time.monotonic()+timeout
    while rclpy.ok() and not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    return future.result() if future.done() else None
try:
    for client in clients.values():
        if not client.wait_for_service(timeout_sec=45):raise RuntimeError('Controller service unavailable: '+client.srv_name)
    for name in ['joint_state_broadcaster','arm_controller']:
        deadline=time.monotonic()+65;confirmed=False
        while time.monotonic()<deadline:
            listed=call(clients['list'],ListControllers.Request())
            if listed is None:continue
            states={c.name:c.state for c in listed.controller};state=states.get(name)
            if state=='active':confirmed=True;print(name,'verified active',flush=True);break
            if state is None:
                req=LoadController.Request();req.name=name;call(clients['load'],req)
            elif state=='unconfigured':
                req=ConfigureController.Request();req.name=name;call(clients['configure'],req)
            elif state=='inactive':
                req=SwitchController.Request();req.activate_controllers=[name];req.strictness=SwitchController.Request.BEST_EFFORT;req.activate_asap=True;req.timeout.sec=5;call(clients['switch'],req,8)
            else:rclpy.spin_once(node,timeout_sec=.1)
        if not confirmed:raise RuntimeError('Unable to verify active controller '+name)
    listed=call(clients['list'],ListControllers.Request());report={'passed':True,'controllers':{c.name:c.state for c in listed.controller} if listed else {}}
    Path(os.environ.get('SORTING_RUN_DIR','/project/results')+'/controllers_verified.json').write_text(json.dumps(report,indent=2))
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
