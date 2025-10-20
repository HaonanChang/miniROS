"""
Use Gello's action to control the exoskeleton.
"""
import time
import numpy as np
from mini_ros.sim.mjx_envs.exo_g1_env import ExoG1Env
from mini_ros.devices.trackers.record3d_tracker import Record3DTracker


if __name__ == "__main__":
    env = ExoG1Env(control_base=True)
    tracker = Record3DTracker()
    tracker.initialize(reader_config=None)
    
    env.reset()
    step_count = 0
    start_time = time.time()    
    # Reanchor the tracker to the current pose
    tracker.reanchor()
    while not env.should_exit:
        # Randomize action
        action = np.zeros(env.num_dof)
        action[0:7] = tracker.get_state()
        print(action[:3])
        env.step(action)
        env.render()
        step_count += 1

        # Print periodic status updates
        if step_count % 1000 == 0:
            elapsed = time.time() - start_time
            hz = step_count / elapsed
            print(f"[Info]: Steps: {step_count}, Rate: {hz:.1f} Hz")
        
        time.sleep(0.01)
    env.close()